#!/usr/bin/env python3
"""
pCARS2 MEB → GLB Converter  (with texture support)
=====================================================
Combines multiple decrypted .meb mesh parts from a pCARS2 vehicle into a
single GLB (binary glTF 2.0) file.

Each .meb file → one glTF Mesh, with one Primitive per material submesh.
When a texture directory is supplied, DDS diffuse textures are embedded as
PNG images (loaded via Pillow) and assigned to each Primitive via PBR materials.

Usage
-----
  # Geometry only (no textures):
  python meb_to_glb.py --dir extracted/Toyota_GT86 --car toyota_gt86 --out gt86.glb

  # With textures (extracted BFF directory):
  python meb_to_glb.py --dir extracted/Toyota_GT86 --car toyota_gt86 --out gt86.glb

  # The tool auto-discovers BMT/DDS files under --dir.

Requires: pygltflib, numpy
Optional: Pillow (for texture embedding — pip install Pillow)
"""

from __future__ import annotations

import argparse
import importlib.util
import io
import os
import re
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pygltflib as gltf

# ── Pillow (optional — for texture support) ──────────────────────────────────
try:
    from PIL import Image as _PILImage
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ── Load meb_reader sibling ───────────────────────────────────────────────────
_HERE = Path(__file__).parent
_MEB_PATH = _HERE / "meb_reader.py"
if not _MEB_PATH.exists():
    sys.exit("[ERROR] meb_reader.py not found next to this script")
_spec = importlib.util.spec_from_file_location("meb_reader", str(_MEB_PATH))
_meb_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_meb_mod)
MebFile = _meb_mod.MebFile


# ─────────────────────────────────────────────────────────────────────────────
# LOD-A part selection
# ─────────────────────────────────────────────────────────────────────────────

_SKIP_PATTERNS = ["_shadow", "_blur", "_dmg", "_glow", "_lodc", "_lodb", "_lodd"]

def _should_include(fname: str) -> bool:
    fn = fname.lower()
    if any(p in fn for p in _SKIP_PATTERNS):
        return False
    return "_loda" in fn


# ─────────────────────────────────────────────────────────────────────────────
# Part placement (VHF node transforms)
# ─────────────────────────────────────────────────────────────────────────────

# (translation xyz, rotation quaternion xyzw, uniform scale), pCARS2 raw space
Transform = Tuple[List[float], List[float], float]


def _quat_mul(a: List[float], b: List[float]) -> List[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def _quat_rotate(q: List[float], v: List[float]) -> List[float]:
    r = _quat_mul(_quat_mul(q, [v[0], v[1], v[2], 0.0]), [-q[0], -q[1], -q[2], q[3]])
    return r[:3]


def parse_vhf_transforms(vhf_path: Path) -> Dict[str, Transform]:
    """
    Map each mesh resource (lowercase .meb stem) to its world transform.

    The .vhf is the car's scene hierarchy: every OBJECT node names its .meb
    RESOURCE and a MatrixNumber; each MATRIX has Offset / Orientation (xyzw
    quaternion) / Scale and an optional parent. Body panels sit on the
    identity matrix, but wheels, tyres, discs, calipers, the steering wheel
    etc. are modelled around their own pivot and placed only by this
    transform — without it they all end up stacked at the origin.
    """
    text = vhf_path.read_bytes().decode("utf-8", errors="replace")
    text = text[text.find("<"):]

    mats: Dict[int, Tuple[List[float], List[float], float, Optional[int]]] = {}
    for m in re.finditer(r"<MATRIX\b([^>]*)/?>", text):
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
        try:
            mid = int(attrs["id"])
            off = [float(v) for v in attrs.get("Offset", "0 0 0").split()[:3]]
            rot = [float(v) for v in attrs.get("Orientation", "0 0 0 1").split()[:4]]
            scl = float(attrs.get("Scale", "1"))
        except (KeyError, ValueError):
            continue
        parent = int(attrs["parent"]) if attrs.get("parent", "").isdigit() else None
        mats[mid] = (off, rot, scl, parent)

    world_cache: Dict[int, Transform] = {}

    def world(mid: int, depth: int = 0) -> Transform:
        if mid in world_cache:
            return world_cache[mid]
        off, rot, scl, parent = mats.get(mid, ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0, None))
        if parent is not None and parent != mid and parent in mats and depth < 32:
            p_off, p_rot, p_scl = world(parent, depth + 1)
            moved = _quat_rotate(p_rot, [c * p_scl for c in off])
            result: Transform = ([p_off[i] + moved[i] for i in range(3)],
                                 _quat_mul(p_rot, rot), p_scl * scl)
        else:
            result = (off, rot, scl)
        world_cache[mid] = result
        return result

    out: Dict[str, Transform] = {}
    # Each OBJECT node opens with its MatrixNumber; its RESOURCE follows inside
    for m in re.finditer(r'<NODE type="OBJECT"[^>]*MatrixNumber="(\d+)"[^>]*>\s*'
                         r'<RESOURCE Filename="([^"]+)"', text):
        stem = Path(m.group(2).replace("\\", "/")).stem.lower()
        out.setdefault(stem, world(int(m.group(1))))
    return out


_CORNER = {"lf": "fl", "rf": "fr", "lr": "rl", "rr": "rr"}   # game -> SVJ station
_CORNER_PART = re.compile(r"_(wheel|tire|tyre|disc|caliper)_(lf|rf|lr|rr)(?:_|$)")


def svj_node_for(part_name: str) -> str:
    """
    SVJ glTF node (spec §22.5) a game mesh part belongs to.

    Wheels and tyres group under the wheel node, discs and calipers get brake
    nodes, the steering wheel its steering node; everything else (panels,
    interior, lights, the combined suspension mesh...) is part of the body.
    """
    n = part_name.lower()
    m = _CORNER_PART.search(n)
    if m:
        kind, corner = m.group(1), _CORNER[m.group(2)]
        if kind in ("wheel", "tire", "tyre"):
            return f"SVJ::wheel::wheel_{corner}"
        return f"SVJ::brake::{kind}_{corner}"
    if "steeringwheel" in n or "steering_wheel" in n:
        return "SVJ::steering::wheel"
    return "SVJ::body::chassis"


def _find_vhf(meb_dir: str) -> Optional[Path]:
    d = Path(meb_dir)
    for cand in (d, d.parent, d.parent.parent):
        hits = sorted(cand.glob("*.vhf"))
        if hits:
            return hits[0]
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Texture / material helpers
# ─────────────────────────────────────────────────────────────────────────────

# Default PBR colors for materials without available textures
# Keyed by lowercase substrings in the material/part name
_DEFAULT_COLORS: Dict[str, Tuple[float, float, float, float]] = {
    "paint":    (0.45, 0.45, 0.50, 1.0),   # neutral silver-gray (no livery)
    "plastic":  (0.20, 0.20, 0.20, 1.0),   # dark gray plastic
    "tyre":     (0.08, 0.08, 0.08, 1.0),   # near-black rubber
    "tire":     (0.08, 0.08, 0.08, 1.0),
    "tread":    (0.08, 0.08, 0.08, 1.0),
    "disc":     (0.30, 0.30, 0.32, 1.0),   # brake disc gray
    "caliper":  (1.00, 0.10, 0.10, 1.0),   # red calipers
    "glass":    (0.80, 0.90, 1.00, 0.35),  # semi-transparent glass
    "window":   (0.80, 0.90, 1.00, 0.35),
    "light":    (0.95, 0.95, 0.95, 1.0),   # light cluster off-white
    "glow":     (1.00, 1.00, 0.80, 1.0),   # warm glow
    "chrome":   (0.80, 0.80, 0.85, 1.0),
    "wheel":    (0.55, 0.55, 0.60, 1.0),   # alloy wheel
    "rim":      (0.55, 0.55, 0.60, 1.0),
    "interior": (0.18, 0.15, 0.13, 1.0),   # dark interior
    "badge":    (0.85, 0.85, 0.85, 1.0),
    "misc":     (0.50, 0.50, 0.52, 1.0),
}
_DEFAULT_COLOR_FALLBACK = (0.50, 0.50, 0.50, 1.0)

_MAX_TEXTURE_SIZE  = 512   # max pixel dimension for embedded textures
_MIN_TEXTURE_SIZE  = 32    # textures smaller than this are replaced with PBR baseColorFactor
_PAINT_METALLIC    = 0.7   # car paint metallic factor for solid-color materials
_PAINT_ROUGHNESS   = 0.2   # car paint roughness for solid-color materials


def _default_color_for(name: str) -> Tuple[float, float, float, float]:
    n = name.lower()
    for key, col in _DEFAULT_COLORS.items():
        if key in n:
            return col
    return _DEFAULT_COLOR_FALLBACK


# Texture slots read from .bmt materials and used in the GLB
_SLOTS = ("diffuseTexture", "normalTexture", "specularTexture")


def _parse_bmt_textures(bmt_path: Path) -> Dict[str, str]:
    """
    Read the texture slots of a pCARS2 .bmt binary material.

    In the material's string table each texture path is followed directly by
    its slot name, e.g. 'vehicles\\textures\\foo_nmp8888.dds', 'normalTexture'.
    Returns {slot: relative_dds_path} for the slots in _SLOTS.
    """
    strings = [s.decode("ascii", errors="replace")
               for s in re.findall(rb"[ -~]{4,}", bmt_path.read_bytes())]
    slots: Dict[str, str] = {}
    for i, st in enumerate(strings[:-1]):
        if st.lower().endswith(".dds") and strings[i + 1] in _SLOTS:
            slots.setdefault(strings[i + 1], st)
    if "diffuseTexture" not in slots:
        first = next((s for s in strings if s.lower().endswith(".dds")), None)
        if first:
            slots["diffuseTexture"] = first
    return slots


def _build_texture_map(search_dirs: List[Path]) -> Dict[str, Dict[str, str]]:
    """
    Scan search_dirs for .bmt files and resolve their texture slots to files.
    Returns {bmt_stem_lower: {slot: absolute_dds_path}}; slots whose DDS isn't
    present (e.g. shared textures in CommonVehicleTextures.bff) are omitted.
    """
    all_dds: Dict[str, str] = {}   # filename_lower → absolute_path
    for d in search_dirs:
        for p in d.rglob("*.dds"):
            all_dds[p.name.lower()] = str(p)

    result: Dict[str, Dict[str, str]] = {}
    for d in search_dirs:
        for bmt in d.rglob("*.bmt"):
            resolved = {}
            for slot, rel in _parse_bmt_textures(bmt).items():
                hit = all_dds.get(Path(rel.replace("\\", "/")).name.lower())
                if hit:
                    resolved[slot] = hit
            result[bmt.stem.lower()] = resolved
    return result


def _open_dds(dds_path: str, mode: str):
    """Open a DDS with Pillow as `mode`, downscaled to <= _MAX_TEXTURE_SIZE."""
    img = _PILImage.open(dds_path).convert(mode)
    w, h = img.size
    if max(w, h) > _MAX_TEXTURE_SIZE:
        f = _MAX_TEXTURE_SIZE / max(w, h)
        img = img.resize((max(1, int(w * f)), max(1, int(h * f))), _PILImage.LANCZOS)
    return img


def _png(img) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return buf.getvalue()


def _load_dds_png(dds_path: str) -> Optional[bytes]:
    """Load a DDS texture, resize to ≤ MAX_TEXTURE_SIZE, return PNG bytes."""
    if not HAS_PIL:
        return None
    try:
        return _png(_open_dds(dds_path, "RGBA"))
    except Exception:
        return None


def _load_normal_png(dds_path: str) -> Optional[bytes]:
    """
    Convert a pCARS2 '_nmp8888' normal map to a glTF normal texture.

    The game stores two channels (X in red, Y in green, blue empty) with
    DirectX's +Y-down convention — measured on the raised bolt heads in
    ford_gt40_badges_nmp8888: red rises left->right, green falls top->bottom.
    glTF wants +X right, +Y toward the image top and an explicit Z, so green
    is inverted and Z is rebuilt as sqrt(1 - x^2 - y^2).
    """
    if not HAS_PIL:
        return None
    try:
        a = np.asarray(_open_dds(dds_path, "RGB"), dtype=np.float32)
    except Exception:
        return None
    x = a[..., 0] / 127.5 - 1.0
    y = -(a[..., 1] / 127.5 - 1.0)
    z = np.sqrt(np.clip(1.0 - x * x - y * y, 0.0, 1.0))
    n = np.stack([x, y, z], axis=-1)
    out = np.clip((n * 0.5 + 0.5) * 255.0 + 0.5, 0, 255).astype(np.uint8)
    return _png(_PILImage.fromarray(out, "RGB"))


def _load_spec_pngs(dds_path: str) -> Optional[Tuple[bytes, bytes]]:
    """
    Split a pCARS2 specular map into glTF inputs: (specular colour, roughness).

    RGB is the specular colour; alpha is gloss (it selects between the
    material's min/max specular power — low on tyres, ~34/255, high on light
    lenses, up to 255). Roughness is approximated as 1 - gloss and written to
    the green channel of a metallicRoughness texture (blue = metallic = 0).
    """
    if not HAS_PIL:
        return None
    try:
        img = _open_dds(dds_path, "RGBA")
    except Exception:
        return None
    a = np.asarray(img)
    rough = np.zeros(a.shape[:2] + (3,), np.uint8)
    rough[..., 1] = 255 - a[..., 3]
    return _png(_PILImage.fromarray(np.ascontiguousarray(a[..., :3]), "RGB")), \
        _png(_PILImage.fromarray(rough, "RGB"))


# ─────────────────────────────────────────────────────────────────────────────
# Core GLB builder
# ─────────────────────────────────────────────────────────────────────────────

def _make_glb_from_mebs(
    meb_paths: List[str],
    car_name: str,
    texture_search_dirs: Optional[List[str]] = None,
    node_transforms: Optional[Dict[str, Transform]] = None,
) -> Optional[gltf.GLTF2]:
    """
    Parse a list of .meb files and return a pygltflib GLTF2 object.

    If texture_search_dirs is provided and Pillow is available, DDS diffuse
    textures are embedded as PNGs and applied via PBR materials.

    node_transforms (from parse_vhf_transforms) places each part; parts with
    no entry keep the identity transform.

    Each .meb becomes one Mesh with one Primitive per material submesh.
    Returns None if no valid geometry was parsed.
    """
    node_transforms = node_transforms or {}
    bin_data = bytearray()

    accessors:    List[gltf.Accessor]   = []
    buffer_views: List[gltf.BufferView] = []
    meshes:       List[gltf.Mesh]       = []
    nodes:        List[gltf.Node]       = []
    materials:    List[gltf.Material]   = []
    images:       List[gltf.Image]      = []
    textures_:    List[gltf.Texture]    = []

    # ── Texture map ───────────────────────────────────────────────────────────
    tex_map: Dict[str, Dict[str, str]] = {}   # bmt_stem → {slot: dds_path}
    if texture_search_dirs and HAS_PIL:
        dirs = [Path(d) for d in texture_search_dirs if os.path.isdir(d)]
        if dirs:
            tex_map = _build_texture_map(dirs)

    # ── Image embedding (each source image embedded once per GLB) ────────────
    tex_cache: Dict[Tuple[str, str], Optional[int]] = {}

    def _embed(png_bytes: bytes) -> int:
        """Store a PNG in the binary buffer; return its glTF texture index."""
        png_offset = len(bin_data)
        bin_data.extend(png_bytes)
        while len(bin_data) % 4:
            bin_data.append(0)
        buffer_views.append(gltf.BufferView(buffer=0, byteOffset=png_offset,
                                            byteLength=len(png_bytes)))
        images.append(gltf.Image(mimeType="image/png",
                                 bufferView=len(buffer_views) - 1))
        textures_.append(gltf.Texture(source=len(images) - 1))
        return len(textures_) - 1

    def _cached(path: str, kind: str, loader) -> Optional[int]:
        key = (path.lower(), kind)
        if key not in tex_cache:
            data = loader(path)
            tex_cache[key] = _embed(data) if data else None
        return tex_cache[key]

    def _spec_textures(path: str) -> Tuple[Optional[int], Optional[int]]:
        key = (path.lower(), "spec")
        if key not in tex_cache:
            pair = _load_spec_pngs(path)
            tex_cache[key] = (_embed(pair[0]), _embed(pair[1])) if pair else None
        return tex_cache[key] or (None, None)

    used_specular_ext = [False]

    # ── Cache: material name → glTF material index ───────────────────────────
    mat_cache: Dict[str, int] = {}

    def _get_or_create_material(mat_name: str) -> int:
        """Return (or create) a glTF material index for a .mtx material name."""
        key = mat_name.lower()
        if key in mat_cache:
            return mat_cache[key]

        # Derive BMT stem from .mtx path: "vehicles\GT86\Toy_GT86_PAINT.mtx" → "toy_gt86_paint"
        mtx_stem = Path(mat_name).stem.lower()
        slots = tex_map.get(mtx_stem, {})

        # Diffuse → base colour (tiny swatches become a flat colour instead)
        tex_idx: Optional[int] = None
        embedded_color = None   # sampled RGBA if texture is too small to use
        dds_path = slots.get("diffuseTexture")
        if dds_path:
            png_bytes = _load_dds_png(dds_path)
            if png_bytes:
                _probe = _PILImage.open(io.BytesIO(png_bytes))
                if max(_probe.size) < _MIN_TEXTURE_SIZE:
                    # Too small (pCARS2 solid-color paint swatch etc.) — sample the
                    # dominant colour and use it as a PBR baseColorFactor instead.
                    px = np.asarray(_probe.convert("RGBA"), dtype=np.float32)
                    embedded_color = tuple(float(c) for c in px.reshape(-1, 4).mean(0) / 255.0)
                else:
                    tex_idx = _cached(dds_path, "diffuse", lambda p: png_bytes)

        # Normal map and specular map (roughness + specular colour)
        normal_idx = (_cached(slots["normalTexture"], "normal", _load_normal_png)
                      if "normalTexture" in slots else None)
        spec_color_idx = rough_idx = None
        if "specularTexture" in slots:
            spec_color_idx, rough_idx = _spec_textures(slots["specularTexture"])

        def _finish(mat: "gltf.Material") -> None:
            if normal_idx is not None:
                mat.normalTexture = gltf.NormalMaterialTexture(index=normal_idx)
            if rough_idx is not None:
                mat.pbrMetallicRoughness.metallicRoughnessTexture = gltf.TextureInfo(index=rough_idx)
                mat.pbrMetallicRoughness.roughnessFactor = 1.0
            if spec_color_idx is not None:
                # Specular colour as a reflectance mask over the default
                # dielectric F0 (0.04): black = no reflection, white = default.
                # The game's values are artistic intensities, not physical F0.
                mat.extensions = {"KHR_materials_specular": {
                    "specularColorTexture": {"index": spec_color_idx},
                }}
                used_specular_ext[0] = True

        # Build the PBR material
        if tex_idx is not None:
            pbr = gltf.PbrMetallicRoughness(
                baseColorTexture=gltf.TextureInfo(index=tex_idx),
                metallicFactor=0.0,
                roughnessFactor=0.7,
            )
            mat = gltf.Material(
                name=mtx_stem,
                pbrMetallicRoughness=pbr,
                doubleSided=True,
            )
            _finish(mat)
        else:
            # Solid-color fallback: use sampled colour from tiny texture OR defaults
            if embedded_color is not None:
                r, g, b, a = embedded_color
            else:
                r, g, b, a = _default_color_for(mtx_stem)
            is_glass = ("glass" in mtx_stem or "window" in mtx_stem)
            is_paint = ("paint" in mtx_stem or "body" in mtx_stem
                        or "livery" in mtx_stem)
            if is_glass:
                metallic, roughness, a = 0.0, 0.05, 0.3
            elif is_paint or embedded_color is not None:
                metallic, roughness = _PAINT_METALLIC, _PAINT_ROUGHNESS
            else:
                metallic, roughness = 0.05, 0.6
            pbr = gltf.PbrMetallicRoughness(
                baseColorFactor=[r, g, b, a],
                metallicFactor=metallic,
                roughnessFactor=roughness,
            )
            mat = gltf.Material(
                name=mtx_stem,
                pbrMetallicRoughness=pbr,
                doubleSided=True,
                alphaMode="BLEND" if is_glass else "OPAQUE",
            )
            if not is_glass:
                _finish(mat)

        materials.append(mat)
        idx = len(materials) - 1
        mat_cache[key] = idx
        return idx

    # ── Buffer helpers ────────────────────────────────────────────────────────
    def _add_bv(data: bytes, target: int) -> int:
        offset = len(bin_data)
        bin_data.extend(data)
        while len(bin_data) % 4:
            bin_data.append(0)
        bv = gltf.BufferView(buffer=0, byteOffset=offset,
                              byteLength=len(data), target=target)
        buffer_views.append(bv)
        return len(buffer_views) - 1

    def _add_acc(bv_idx: int, comp_type: int, count: int,
                 acc_type: str, mn=None, mx=None) -> int:
        acc = gltf.Accessor(bufferView=bv_idx, byteOffset=0,
                             componentType=comp_type, count=count, type=acc_type)
        if mn is not None:
            acc.min = [float(x) for x in mn]
        if mx is not None:
            acc.max = [float(x) for x in mx]
        accessors.append(acc)
        return len(accessors) - 1

    ok_count = 0
    for meb_path in meb_paths:
        part_name = Path(meb_path).stem
        try:
            m = MebFile(meb_path)
        except Exception as exc:
            print(f"    [WARN] {part_name}: parse error — {exc}")
            continue

        pos  = m.positions()
        nrm  = m.normals()
        uvs  = m.uvs()

        if not pos or not m.index_buffers:
            print(f"    [SKIP] {part_name}: no geometry")
            continue

        # ── Vertex data (shared across all submesh primitives) ────────────────
        # pCARS2 mesh space is Y-up but faces the opposite way from glTF's
        # convention, so a 180 deg yaw (negate X and Z, keep Y) is needed.
        # This must be a proper rotation (det=+1), not a single-axis mirror:
        # a mirror (e.g. negating Z alone, as this used to do) flips triangle
        # winding relative to the transformed normals, which is why meshes
        # came out flat/inconsistently shaded — doubleSided materials hid the
        # backface-culling symptom but not the resulting lighting artifacts.
        pos_np = np.array(pos, dtype=np.float32)
        pos_gl = np.column_stack([-pos_np[:, 0],
                                    pos_np[:, 1],
                                   -pos_np[:, 2]]).astype(np.float32)
        bv_pos = _add_bv(pos_gl.tobytes(), gltf.ARRAY_BUFFER)
        acc_pos = _add_acc(bv_pos, gltf.FLOAT, len(pos), "VEC3",
                           pos_gl.min(axis=0).tolist(),
                           pos_gl.max(axis=0).tolist())

        acc_nrm: Optional[int] = None
        if nrm:
            nrm_np = np.array(nrm, dtype=np.float32)
            nrm_gl = np.column_stack([-nrm_np[:, 0],
                                        nrm_np[:, 1],
                                       -nrm_np[:, 2]]).astype(np.float32)
            bv_nrm = _add_bv(nrm_gl.tobytes(), gltf.ARRAY_BUFFER)
            acc_nrm = _add_acc(bv_nrm, gltf.FLOAT, len(nrm), "VEC3")

        acc_uv: Optional[int] = None
        if uvs:
            uv_np = np.array(uvs, dtype=np.float32)
            # Flip V for glTF convention (V=0 at top)
            uv_np[:, 1] = 1.0 - uv_np[:, 1]
            bv_uv = _add_bv(uv_np.tobytes(), gltf.ARRAY_BUFFER)
            acc_uv = _add_acc(bv_uv, gltf.FLOAT, len(uvs), "VEC2")

        # ── One Primitive per material submesh ────────────────────────────────
        primitives: List[gltf.Primitive] = []
        total_tris = 0
        for mat_name, face_list in m.index_buffers:
            if len(face_list) < 3:
                continue
            # Ensure triangle-aligned length
            n = (len(face_list) // 3) * 3
            idx_np = np.array(face_list[:n], dtype=np.uint16)
            bv_idx = _add_bv(idx_np.tobytes(), gltf.ELEMENT_ARRAY_BUFFER)
            acc_idx = _add_acc(bv_idx, gltf.UNSIGNED_SHORT, n, "SCALAR")

            mat_idx = _get_or_create_material(mat_name)

            attrs = gltf.Attributes(POSITION=acc_pos)
            if acc_nrm is not None:
                attrs.NORMAL = acc_nrm
            if acc_uv is not None:
                attrs.TEXCOORD_0 = acc_uv
            prim = gltf.Primitive(attributes=attrs, indices=acc_idx,
                                   mode=gltf.TRIANGLES, material=mat_idx)
            primitives.append(prim)
            total_tris += n // 3

        if not primitives:
            print(f"    [SKIP] {part_name}: no valid primitives")
            continue

        mesh = gltf.Mesh(name=part_name, primitives=primitives)
        meshes.append(mesh)
        node = gltf.Node(name=part_name, mesh=len(meshes) - 1)
        xf = node_transforms.get(part_name.lower())
        if xf:
            (tx, ty, tz), (qx, qy, qz, qw), s = xf
            # Same 180 deg yaw as the vertex data (negate X and Z): conjugating
            # by it maps translation (x,y,z)->(-x,y,-z), quaternion axis likewise.
            if any(abs(v) > 1e-9 for v in (tx, ty, tz)):
                node.translation = [-tx, ty, -tz]
            if any(abs(v) > 1e-9 for v in (qx, qy, qz)):
                node.rotation = [-qx, qy, -qz, qw]
            if abs(s - 1.0) > 1e-9:
                node.scale = [s, s, s]
        nodes.append(node)
        ok_count += 1
        tex_info = f"  ({len([p for p in primitives if p.material is not None])} textured)" if tex_map else ""
        print(f"    {part_name}: {len(pos)} verts, {total_tris} tris{tex_info}")

    if ok_count == 0:
        return None

    # ── Group parts under SVJ-named nodes (spec §22.5) ───────────────────────
    # SVJ visual bindings refer to glTF nodes by name (SVJ::body::chassis,
    # SVJ::wheel::wheel_fl, ...). Group nodes carry no transform; each part
    # keeps its own placement from the .vhf.
    groups: Dict[str, List[int]] = {}
    for i, n in enumerate(nodes):
        groups.setdefault(svj_node_for(n.name), []).append(i)
    group_idx: List[int] = []
    for name in sorted(groups):
        nodes.append(gltf.Node(name=name, children=groups[name]))
        group_idx.append(len(nodes) - 1)

    root_idx = len(nodes)
    nodes.append(gltf.Node(name=car_name, children=group_idx))

    buf = gltf.Buffer(byteLength=len(bin_data))
    scene = gltf.Scene(name=car_name, nodes=[root_idx])

    g = gltf.GLTF2(
        scene=0,
        scenes=[scene],
        nodes=nodes,
        meshes=meshes,
        accessors=accessors,
        bufferViews=buffer_views,
        buffers=[buf],
        materials=materials if materials else None,
        images=images if images else None,
        textures=textures_ if textures_ else None,
        asset=gltf.Asset(
            version="2.0",
            generator="pCARS2->GLB Converter v3 (meb_to_glb.py)",
            extras={"source_game": "Project CARS 2", "car": car_name},
        ),
    )
    if used_specular_ext[0]:
        # Optional (not "required"): viewers without it just ignore the
        # specular colour and still render base colour, normals, roughness.
        g.extensionsUsed = ["KHR_materials_specular"]
    g.set_binary_blob(bytes(bin_data))
    g.extras = {"svj_nodes": sorted(groups)}
    return g


def convert_car_to_glb(
    car_name: str,
    meb_dir: str,
    out_path: str,
    texture_dirs: Optional[List[str]] = None,
) -> List[str]:
    """
    Find all suitable .meb files under meb_dir for car_name and convert to GLB.
    If texture_dirs is None, auto-discover DDS files under meb_dir.

    Returns the SVJ node names created in the GLB (e.g. 'SVJ::body::chassis',
    'SVJ::wheel::wheel_fl'), so callers bind only nodes that exist. An empty
    list means the conversion failed.
    """
    meb_files = sorted(
        str(p) for p in Path(meb_dir).rglob("*.meb")
        if _should_include(p.name)
    )
    if not meb_files:
        print(f"  [WARN] No LOD-A .meb files found under {meb_dir}")
        return []

    # Auto-discover texture search dirs (the extraction root)
    if texture_dirs is None:
        # The BFF usually extracts to a subfolder; search the parent for textures
        tex_dirs = [meb_dir, str(Path(meb_dir).parent),
                    str(Path(meb_dir).parent.parent)]
        texture_dirs = [d for d in tex_dirs if os.path.isdir(d)]

    print(f"  {car_name}: {len(meb_files)} mesh parts")
    if HAS_PIL and texture_dirs:
        tex_map = _build_texture_map([Path(d) for d in texture_dirs
                                      if os.path.isdir(d)])
        count = lambda slot: sum(1 for v in tex_map.values() if slot in v)
        print(f"  Textures: {count('diffuseTexture')} diffuse, {count('normalTexture')} normal, "
              f"{count('specularTexture')} specular ({len(tex_map)} materials)")
    else:
        texture_dirs = None   # disable if no PIL

    vhf = _find_vhf(meb_dir)
    transforms = parse_vhf_transforms(vhf) if vhf else {}
    if not transforms:
        print(f"  [WARN] {car_name}: no .vhf hierarchy found — parts not positioned")

    g = _make_glb_from_mebs(meb_files, car_name, texture_dirs, transforms)
    if g is None:
        print(f"  [FAIL] {car_name}: no valid geometry")
        return []

    g.save_binary(out_path)
    size_kb = Path(out_path).stat().st_size / 1024
    print(f"  OK {car_name} -> {Path(out_path).name}  ({size_kb:.0f} KB)")
    return g.extras["svj_nodes"]


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    sys.stdout.reconfigure(encoding="utf-8")

    ap = argparse.ArgumentParser(
        description="Convert pCARS2 .meb mesh files to GLB (with optional textures)")
    ap.add_argument("mebs", nargs="*", help="Direct .meb file(s) to convert")
    ap.add_argument("--dir", default=None,
                    help="Directory containing .meb files for one car")
    ap.add_argument("--car", default=None,
                    help="Car name (GLB root node name)")
    ap.add_argument("--out", default=None, help="Output .glb path")
    ap.add_argument("--no-textures", action="store_true",
                    help="Skip texture embedding (geometry only)")
    args = ap.parse_args()

    tex_dirs = None if args.no_textures else None   # auto-detected in convert_car_to_glb

    if args.dir:
        car  = args.car or Path(args.dir).name
        out  = args.out or f"{car}.glb"
        convert_car_to_glb(car, args.dir, out, tex_dirs)
        return

    if args.mebs:
        car = args.car or Path(args.mebs[0]).stem
        out = args.out or f"{car}.glb"
        g   = _make_glb_from_mebs(args.mebs, car,
                                   None if args.no_textures else None)
        if g:
            g.save_binary(out)
            sz = Path(out).stat().st_size / 1024
            print(f"Saved {out}  ({sz:.0f} KB)")
        return

    ap.print_help()


if __name__ == "__main__":
    main()
