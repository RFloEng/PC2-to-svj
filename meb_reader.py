#!/usr/bin/env python3
"""
pCARS2 MEB (Mesh Binary) Format Parser
=======================================
Parses decrypted .meb files extracted from pCARS2 vehicle BFF archives.

Format documented via PCarsTools BinaryTemplates/Meb.bt by Nenkai.

Key points:
- BVersion bitfield at offset 0: Major(4), Minor(6), Interim(11), Auto(11)
  Stored as little-endian u32. Minor is bits 22-27.
- For Minor >= 4: 4 bytes of flags follow BVersion
- Vertex streams identified by SemanticName (POSITION=0, NORMAL=2, TEXCOORD=3, ...)
- Index buffer follows all streams: name, int a, int b, b × u16 triangle indices

Usage:
    mesh = MebFile("toy_gt86_13_chassis_loda.meb")
    mesh.save_obj("chassis.obj")
"""

from __future__ import annotations
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# DXGI_FORMAT enum values (from template)
DXGI_FORMAT = {
    0:  "UNKNOWN",
    1:  "R32G32B32A32_TYPELESS",
    2:  "R32G32B32A32_FLOAT",    # → 3 × f32 (RGBA_FLOAT in template, but only R,G,B used)
    4:  "R32G32B32A32_SINT",     # → RGBA[N/4] + skip 8
    5:  "R32G32B32_TYPELESS",
    6:  "R32G32B32_FLOAT",       # standard 3D float vector
    20: "D32_FLOAT_S8X24_UINT",  # encrypted position marker → BVec3F[N]
    16: "R32G32_FLOAT",          # → 2 × f32 (UV)
    17: "R32G32_UINT",
    41: "R32_FLOAT",             # → 1 × f32
    56: "R8G8_UNORM",
    28: "R8G8B8A8_UNORM",
    31: "R8G8B8A8_SNORM",        # → 4 × u8 (normals/tangents packed)
    29: "R8G8B8A8_UNORM_SRGB",
}

# SemanticName enum values
SEMANTIC = {
    0: "POSITION",
    1: "BLENDWEIGHT",
    2: "NORMAL",
    3: "TEXCOORD",
    4: "TANGENT",
    5: "BINORMAL",
    6: "COLOR",
    7: "DEPTH",
    8: "BLENDINDICES",
}

# Bytes per vertex for each DXGI_FORMAT used in pCARS2 MEB
_FORMAT_STRIDE = {
    2:  12,   # R32G32B32A32_FLOAT  → template reads RGBA_FLOAT (3×f32=12B)
    4:  16,   # R32G32B32A32_SINT   → RGBA (4×i32=16B), N/4 entries + skip 8
    6:  12,   # R32G32B32_FLOAT     → 3×f32
    16: 8,    # R32G32_FLOAT        → 2×f32
    17: 8,    # R32G32_UINT         → 2×u32
    20: 12,   # D32_FLOAT_S8X24_UINT→ BVec3F (3×f32) — encrypted
    28: 4,    # R8G8B8A8_UNORM      → 4 bytes
    29: 4,    # R8G8B8A8_UNORM_SRGB → 4 bytes
    31: 4,    # R8G8B8A8_SNORM      → 4 bytes
    41: 4,    # R32_FLOAT           → 1×f32
    0:  4,    # UNKNOWN             → int[N] = 4 bytes
    56: 2,    # R8G8_UNORM          → 2 bytes
}


class MebStream:
    """A single vertex data stream within a MEB file."""
    def __init__(self, fmt: int, semantic: int, semantic_idx: int, raw: bytes):
        self.format   = fmt
        self.semantic = semantic
        self.semantic_idx = semantic_idx
        self.raw      = raw

    @property
    def format_name(self) -> str:
        return DXGI_FORMAT.get(self.format, f"fmt_{self.format}")

    @property
    def semantic_name(self) -> str:
        return SEMANTIC.get(self.semantic, f"sem_{self.semantic}")

    def __repr__(self):
        return (f"<MebStream {self.semantic_name}[{self.semantic_idx}] "
                f"{self.format_name} {len(self.raw)}B>")


class MebFile:
    """
    Parsed pCARS2 .meb mesh file.

    After construction:
        .name          — mesh name string
        .num_verts     — vertex count per stream
        .streams       — list of MebStream objects
        .index_buffers — list of (name, list-of-u16) tuples

    Convenience methods:
        .positions()   — list of (x, y, z) float tuples
        .normals()     — list of (x, y, z) float tuples (if NORMAL stream present)
        .uvs()         — list of (u, v) float tuples (if TEXCOORD stream present)
        .indices()     — flattened list of u16 triangle indices (first index buffer)
        .triangles()   — list of (i0, i1, i2) tuples
        .save_obj()    — write OBJ file
    """

    def __init__(self, path: str):
        self.path = path
        with open(path, "rb") as f:
            self._buf = f.read()
        self._parse()

    # ─── Internal parser ──────────────────────────────────────────────────────

    def _u8(self, off):  return self._buf[off]
    def _u16(self, off): return struct.unpack_from("<H", self._buf, off)[0]
    def _u32(self, off): return struct.unpack_from("<I", self._buf, off)[0]
    def _s32(self, off): return struct.unpack_from("<i", self._buf, off)[0]
    def _f32(self, off): return struct.unpack_from("<f", self._buf, off)[0]

    @staticmethod
    def _align4(off): return (off + 3) & ~3

    def _parse(self):
        off = 0

        # ── BVersion (4 bytes) ──────────────────────────────────────────────
        ver_raw = self._u32(off); off += 4
        # Bitfield: Auto[0:11], Interim[11:22], Minor[22:28], Major[28:32]
        self.minor = (ver_raw >> 22) & 0x3F
        self.major = (ver_raw >> 28) & 0x0F

        # ── Version-dependent flags ──────────────────────────────────────────
        self.has_bones = False
        if self.minor == 2:
            self.has_bones = bool(self._u8(off)); off += 1
        elif self.minor in (4, 5):
            self.has_bones            = bool(self._u8(off));     off += 1
            self.dynamic_env_req      = bool(self._u8(off));     off += 1
            off += 2   # unk short
        elif self.minor >= 6:
            self.has_bones            = bool(self._u8(off));     off += 1
            self.dynamic_env_req      = bool(self._u8(off));     off += 1
            self.flags                = self._u8(off);           off += 1
            self.cpu_reason           = self._u8(off);           off += 1
            off += 4   # empty + unkBool + unkFlags[2]

        # ── Name (null-terminated) ──────────────────────────────────────────
        end = self._buf.index(b"\x00", off)
        self.name = self._buf[off:end].decode("ascii", errors="replace")
        off = end + 1
        if self.minor >= 4:
            off = self._align4(off)

        # ── Mesh counts ─────────────────────────────────────────────────────
        self.num_verts   = self._u32(off); off += 4
        self.num_streams = self._u32(off); off += 4
        self.num_idxbufs = self._u32(off); off += 4

        # ── Bounding volumes ─────────────────────────────────────────────────
        self.bounds_sphere = tuple(self._f32(off + i*4) for i in range(4)); off += 16
        self.aabb_min = tuple(self._f32(off + i*4) for i in range(3)); off += 12
        self.aabb_max = tuple(self._f32(off + i*4) for i in range(3)); off += 12

        # ── Bones (optional) ─────────────────────────────────────────────────
        self.bones = []
        if self.has_bones and self.minor >= 2:
            num_bones        = self._u32(off); off += 4
            bone_name_size   = self._u32(off); off += 4
            for _ in range(num_bones):
                bend = self._buf.index(b"\x00", off)
                self.bones.append(self._buf[off:bend].decode("ascii", errors="replace"))
                off = bend + 1
            off = self._align4(off)
            # Skip reference transforms + inverse transforms + unk (3 × numBones × BVec4F)
            off += num_bones * 48  # 3 × 4 × 4 bytes

        # ── Vertex streams ───────────────────────────────────────────────────
        self.streams: List[MebStream] = []
        for _ in range(self.num_streams):
            fmt   = self._u32(off); off += 4
            sem   = self._u32(off); off += 4
            semidx= self._u32(off); off += 4

            stride = _FORMAT_STRIDE.get(fmt, 4)
            # Special cases from template:
            # R32G32B32A32_SINT (4): empirically 4 bytes/vertex (packed SINT8x4
            # color), NOT the naive "RGBA[N/4] + FSkip(8)" reading of the .bt
            # template — that formula only coincidentally matches num_verts*4
            # when num_verts % 4 == 2, which silently desynced every stream
            # after COLOR (including NORMAL) for every other vertex count.
            # Confirmed against 5 real .meb files of varying vertex counts by
            # locating the next stream's header via forward byte-scan.
            # R32G32B32A32_TYPELESS (1): packs 2 vertices per 16-byte entry;
            # needs ceiling division so an odd num_verts still gets a full
            # entry for its last (unpaired) vertex — flooring left the last
            # entry's second slot missing entirely, one 16-byte block short,
            # desyncing every subsequent stream (confirmed via unpack_from
            # running off the end of the buffer while reading the final UV).
            if fmt == 4:
                byte_count = self.num_verts * 4
            elif fmt == 1:
                byte_count = ((self.num_verts + 1) // 2) * 16
            else:
                byte_count = self.num_verts * stride

            raw = self._buf[off:off + byte_count]
            self.streams.append(MebStream(fmt, sem, semidx, raw))
            off += byte_count

        # ── Index buffer(s) ──────────────────────────────────────────────────
        # Template: string name + align + int a + int b + short Faces[b]
        # Empirically: `b` is the TRIANGLE count, so actual u16 data = b * 3 shorts.
        # Multiple index buffers exist (one per material submesh).
        #
        # FALLBACK: if stream parsing lands at wrong position (misaligned strides),
        # scan the rest of the file for pCARS2 index buffer patterns:
        #   - null-terminated name string ending in ".mtx"
        #   - followed by 4-byte aligned a=0, b=triangle_count
        #   - followed by b*3 valid u16 indices all < num_verts
        self.index_buffers: List[Tuple[str, List[int]]] = []

        def _try_parse_ibuf(scan_off: int) -> Optional[Tuple[str, List[int], int]]:
            """Try to parse an IndexBuffer at scan_off. Returns (name, faces, end_off) or None."""
            try:
                end = self._buf.index(b"\x00", scan_off)
            except ValueError:
                return None
            name = self._buf[scan_off:end].decode("ascii", errors="replace")
            aoff = end + 1
            if self.minor >= 4:
                aoff = self._align4(aoff)
            if aoff + 8 > len(self._buf):
                return None
            a = self._u32(aoff)
            b = self._u32(aoff + 4)
            if b == 0 or b > 200_000:
                return None
            # Try b = triangle count (b*3 indices)
            idx_count = b * 3
            if aoff + 8 + idx_count * 2 > len(self._buf):
                idx_count = b   # fallback: b = raw index count
            if aoff + 8 + idx_count * 2 > len(self._buf):
                return None
            raw = struct.unpack_from(f"<{idx_count}H", self._buf, aoff + 8)
            if any(i >= self.num_verts for i in raw):
                # Try with b as raw index count
                idx_count = b
                if aoff + 8 + idx_count * 2 > len(self._buf):
                    return None
                raw = struct.unpack_from(f"<{idx_count}H", self._buf, aoff + 8)
                if any(i >= self.num_verts for i in raw):
                    return None
            return (name, list(raw), aoff + 8 + idx_count * 2)

        def _find_all_ibufs_by_scan() -> List[Tuple[str, List[int]]]:
            """
            Scan the entire file for '.mtx\0' patterns and parse each as an IndexBuffer.
            Used when sequential parsing after streams fails (misaligned strides) or
            when index buffers are non-contiguous (material data between them).
            """
            results: List[Tuple[str, List[int]]] = []
            # Collect all .mtx\0 offsets in the file
            import re as _re
            for m in _re.finditer(b"\\.mtx\x00", self._buf):
                mtx_off = m.start()
                # Walk backward to find the start of the path string
                ns = mtx_off
                while ns > 0 and 32 <= self._buf[ns - 1] <= 126:
                    ns -= 1
                # Skip leading garbage (non-alpha/non-underscore chars before path)
                while ns < mtx_off and not (chr(self._buf[ns]).isalpha()
                                             or self._buf[ns] == ord('_')):
                    ns += 1
                r = _try_parse_ibuf(ns)
                if r:
                    results.append((r[0], r[1]))
            return results

        # First attempt: parse sequentially immediately after vertex streams
        result = _try_parse_ibuf(off)
        if result:
            # Sequential parse works (streams were aligned correctly)
            while result:
                name, faces, off = result
                self.index_buffers.append((name, faces))
                result = _try_parse_ibuf(off)
            # If we got fewer index buffers than expected, supplement with full scan
            # (some index buffers may be non-contiguous, separated by per-submesh bounds)
            if len(self.index_buffers) < self.num_idxbufs:
                extra = _find_all_ibufs_by_scan()
                # Dedup by basename, not the full raw name: the sequential
                # parser can land a few bytes into the path string (e.g.
                # "\Foo\Bar.mtx" missing a leading "vehicles\"), which still
                # names the same submesh as the scan-found full path. Exact
                # string comparison missed that and let the same submesh's
                # triangles get appended twice, doubling its geometry.
                seen_basenames = {n.rsplit("\\", 1)[-1].rsplit("/", 1)[-1].lower()
                                   for n, _ in self.index_buffers}
                for n, f in extra:
                    base = n.rsplit("\\", 1)[-1].rsplit("/", 1)[-1].lower()
                    if base not in seen_basenames:
                        self.index_buffers.append((n, f))
                        seen_basenames.add(base)
        else:
            # Full fallback: scan whole file for all .mtx submeshes
            self.index_buffers = _find_all_ibufs_by_scan()

    # ─── Public accessors ────────────────────────────────────────────────────

    def _stream_for(self, semantic: int) -> Optional[MebStream]:
        for s in self.streams:
            if s.semantic == semantic:
                return s
        return None

    def positions(self) -> List[Tuple[float, float, float]]:
        """Return list of (x, y, z) vertex positions."""
        s = self._stream_for(0)   # POSITION
        if s is None:
            return []
        # Format 2 (R32G32B32A32_FLOAT) → RGBA_FLOAT = 3×f32 in template
        # Format 20 (D32_FLOAT_S8X24_UINT, encrypted) → BVec3F = 3×f32
        # Format 6  (R32G32B32_FLOAT) → 3×f32
        # All supported formats for POSITION in pCARS2 use 3×f32 = 12 bytes per vert
        out = []
        raw = s.raw
        for i in range(self.num_verts):
            x, y, z = struct.unpack_from("<fff", raw, i * 12)
            out.append((x, y, z))
        return out

    def normals(self) -> List[Tuple[float, float, float]]:
        """Return list of (x, y, z) normals (decoded from packed format if needed)."""
        s = self._stream_for(2)   # NORMAL
        if s is None:
            return []
        out = []
        raw = s.raw
        if s.format == 31:   # R8G8B8A8_SNORM → 4 bytes, each -128..127 → -1..1
            for i in range(self.num_verts):
                nx = struct.unpack_from("<b", raw, i * 4)[0] / 127.0
                ny = struct.unpack_from("<b", raw, i * 4 + 1)[0] / 127.0
                nz = struct.unpack_from("<b", raw, i * 4 + 2)[0] / 127.0
                out.append((nx, ny, nz))
        elif s.format in (2, 6, 20):   # 3×f32
            for i in range(self.num_verts):
                x, y, z = struct.unpack_from("<fff", raw, i * 12)
                out.append((x, y, z))
        return out

    def uvs(self) -> List[Tuple[float, float]]:
        """Return list of (u, v) UV coords from the primary TEXCOORD[0] stream."""
        # Find first TEXCOORD stream (semantic_idx 0)
        s = None
        for st in self.streams:
            if st.semantic == 3 and st.semantic_idx == 0:
                s = st
                break
        if s is None:
            return []
        out = []
        raw = s.raw
        if s.format == 16:   # R32G32_FLOAT → 2×f32 per vert, 8 bytes each
            for i in range(self.num_verts):
                u, v = struct.unpack_from("<ff", raw, i * 8)
                out.append((u, v))
        elif s.format == 1:  # R32G32B32A32_TYPELESS → RGBA_FLOAT_TYPELESS[N/2]
            # Each 16-byte entry stores 2 verts: (u0, v0, u1, v1)
            for i in range(self.num_verts):
                entry = i // 2
                slot  = i % 2
                base  = entry * 16 + slot * 8
                u, v = struct.unpack_from("<ff", raw, base)
                out.append((u, v))
        elif s.format == 35:  # R16G16_FLOAT (half-float) → 4 bytes per vert
            import struct as _st
            import ctypes
            for i in range(self.num_verts):
                # Read two half-floats (f16) and convert to f32
                h0, h1 = _st.unpack_from("<HH", raw, i * 4)
                # Simple f16 → f32 conversion
                def h2f(h):
                    s = (h >> 15) & 1
                    e = (h >> 10) & 0x1F
                    m = h & 0x3FF
                    if e == 0:
                        return float((-1)**s * m * 2**-24)
                    elif e == 31:
                        return float('inf') if m == 0 else float('nan')
                    return (-1)**s * (1 + m/1024.0) * 2**(e-15)
                out.append((h2f(h0), h2f(h1)))
        return out

    def indices(self) -> List[int]:
        """Return flat list of u16 triangle indices from ALL index buffers combined."""
        if not self.index_buffers:
            return []
        # Concatenate all material submeshes into one index list
        combined: List[int] = []
        for _, faces in self.index_buffers:
            combined.extend(faces)
        return combined

    def triangles(self) -> List[Tuple[int, int, int]]:
        """Return list of (i0, i1, i2) triangle tuples."""
        idx = self.indices()
        return [(idx[i], idx[i+1], idx[i+2]) for i in range(0, len(idx) - 2, 3)]

    # ─── Export ──────────────────────────────────────────────────────────────

    def save_obj(self, out_path: str, y_up_to_z_up: bool = True) -> int:
        """
        Write a Wavefront OBJ file.  Returns triangle count.

        y_up_to_z_up: if True, swaps pCARS2 Y-up to GLB/Blender Z-up
          (swaps Y and Z components, negates new Y = old Z)
        """
        pos  = self.positions()
        nrm  = self.normals()
        uvs  = self.uvs()
        tris = self.triangles()

        if not pos or not tris:
            print(f"  [WARN] {self.name}: no positions or triangles to write")
            return 0

        with open(out_path, "w", encoding="utf-8") as f:
            f.write(f"# pCARS2 MEB → OBJ  mesh={self.name}  verts={len(pos)}  tris={len(tris)}\n")
            f.write(f"# Source: {Path(self.path).name}\n")
            f.write(f"o {self.name}\n\n")

            for x, y, z in pos:
                if y_up_to_z_up:
                    f.write(f"v {x:.6f} {z:.6f} {-y:.6f}\n")   # Y-up → Z-up
                else:
                    f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")

            if nrm:
                for nx, ny, nz in nrm:
                    if y_up_to_z_up:
                        f.write(f"vn {nx:.6f} {nz:.6f} {-ny:.6f}\n")
                    else:
                        f.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")

            if uvs:
                for u, v in uvs:
                    f.write(f"vt {u:.6f} {1.0 - v:.6f}\n")   # flip V for OBJ convention

            f.write("\n")
            for i0, i1, i2 in tris:
                # OBJ is 1-indexed
                a, b, c = i0 + 1, i1 + 1, i2 + 1
                if nrm and uvs:
                    f.write(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n")
                elif nrm:
                    f.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")
                elif uvs:
                    f.write(f"f {a}/{a} {b}/{b} {c}/{c}\n")
                else:
                    f.write(f"f {a} {b} {c}\n")

        return len(tris)

    def __repr__(self):
        streams_s = ", ".join(s.semantic_name for s in self.streams)
        idxbufs_s = str([f"{n}:{len(f)//3}tris" for n, f in self.index_buffers])
        return (f"<MebFile '{self.name}' v{self.major}.{self.minor} "
                f"verts={self.num_verts} streams=[{streams_s}] "
                f"idxbufs={idxbufs_s}>")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    sys.stdout.reconfigure(encoding="utf-8")
    import argparse

    ap = argparse.ArgumentParser(
        description="Parse pCARS2 .meb mesh files and export to OBJ")
    ap.add_argument("meb", help="Path to a .meb file")
    ap.add_argument("--out", default=None, help="Output .obj path (default: <name>.obj)")
    ap.add_argument("--info", action="store_true", help="Print info only, no export")
    args = ap.parse_args()

    m = MebFile(args.meb)
    print(m)
    print(f"  Bounding sphere: center={m.bounds_sphere[:3]}  radius={m.bounds_sphere[3]:.3f}")
    print(f"  AABB: min={m.aabb_min}  max={m.aabb_max}")
    for s in m.streams:
        print(f"  Stream: {s}")
    for name, faces in m.index_buffers:
        print(f"  IndexBuffer '{name}': {len(faces)} indices ({len(faces)//3} triangles)")

    if args.info:
        return

    out = args.out or (Path(args.meb).stem + ".obj")
    n_tris = m.save_obj(out)
    print(f"\n  Exported {n_tris} triangles → {out}")


if __name__ == "__main__":
    main()
