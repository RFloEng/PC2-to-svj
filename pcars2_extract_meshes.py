#!/usr/bin/env python3
"""
pCARS2 Mesh Extraction Pipeline
================================
For each car in the SVJ output, this script:
  1. Locates the vehicle's .bff archive in the game's Pakfiles/Vehicles/ directory
  2. Extracts it with PCarsTools (Oodle-decrypted + decompressed)
  3. Decrypts all LOD-A .meb files (some use encrypted vertex positions)
  4. Combines the LOD-A mesh parts into a single .glb file
  5. Updates the corresponding .svj.json's assets.meshes to reference the GLB

Prerequisites
-------------
  - PCarsTools.exe v1.1.4 in tools_dir (from _pcars2/tools/win-x64/)
  - oo2core_7_win64.dll alongside PCarsTools.exe (the game's oo2core_4 renamed)
  - DOTNET_ROLL_FORWARD=Major set automatically (requires .NET 7+ installed)
  - meb_reader.py and meb_to_glb.py in the same directory as this script

Usage
-----
  # Convert a single car
  python pcars2_extract_meshes.py --game "E:/SteamLibrary/.../Project CARS 2" \\
    --car toyota_gt86 --svj-dir _pcars2/svj_output --glb-dir _pcars2/meshes

  # Batch: all cars whose SVJ file exists
  python pcars2_extract_meshes.py --game "E:/SteamLibrary/.../Project CARS 2" \\
    --all --svj-dir _pcars2/svj_output --glb-dir _pcars2/meshes \\
    --work-dir _pcars2/mesh_work

Output
------
  <glb-dir>/<car_name>.glb          — GLB mesh file
  <svj-dir>/<car_name>.svj.json     — updated with assets.meshes reference

Notes
-----
  Mass-extracting all 212 cars is disk-intensive (~2–4 GB temporary workspace).
  Use --cleanup to delete the temporary .meb/.dds extraction after each car.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ── Load meb_to_glb sibling ───────────────────────────────────────────────────
_HERE = Path(__file__).parent
_MEB2GLB_PATH = _HERE / "meb_to_glb.py"
if not _MEB2GLB_PATH.exists():
    sys.exit("[ERROR] meb_to_glb.py not found next to this script")
_spec = importlib.util.spec_from_file_location("meb_to_glb", str(_MEB2GLB_PATH))
_meb2glb_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_meb2glb_mod)
convert_car_to_glb = _meb2glb_mod.convert_car_to_glb

_spec_n = importlib.util.spec_from_file_location("pcars2_names", str(_HERE / "pcars2_names.py"))
_names_mod = importlib.util.module_from_spec(_spec_n)
_spec_n.loader.exec_module(_names_mod)
rank_candidates = _names_mod.rank_candidates


# ─────────────────────────────────────────────────────────────────────────────
# BFF name mapper
# ─────────────────────────────────────────────────────────────────────────────

# Physics car name -> BFF stem, for cases where a symbol was spelled out
# differently in each naming scheme (e.g. "2&4" -> "24" vs "2and4") and no
# amount of token matching can bridge it without risking false positives
# elsewhere.
_BFF_NAME_ALIASES: Dict[str, str] = {
    "honda_24_concept": "Honda_2and4",
}


def _bff_candidates(car_name: str, vehicles_dir: Path) -> List[Path]:
    """
    Return .bff files in vehicles_dir that likely match the physics car name,
    best match first.

    pCARS2 physics names use snake_case (e.g. 'toyota_gt86', 'mclaren_p1').
    BFF files use PascalCase (e.g. 'Toyota_GT86.bff', 'McLaren_P1.bff').
    Brand/model naming doesn't always agree word-for-word (e.g. the physics
    car 'mercedes-sauber_c9' ships its mesh as 'Sauber_C9.bff').

    Uses token scoring from pcars2_names (exact normalized match first, then
    best token overlap), so a shared brand prefix alone never wins against a
    candidate that also matches the model tokens — the old first-6-chars
    heuristic let e.g. 'lamborghini_huracan_lp610-4' silently pick
    'Lamborghini_Aventador.bff'. Cockpit/livery archives are excluded.
    candidates[0] is the best guess; the rest are lower-confidence fallbacks.
    """
    alias = _BFF_NAME_ALIASES.get(car_name.lower())
    if alias:
        hit = vehicles_dir / f"{alias}.bff"
        if hit.is_file():
            return [hit]

    bffs = [b for b in vehicles_dir.glob("*.bff")
            if "cockpit" not in b.stem.lower() and "livery" not in b.stem.lower()]
    return rank_candidates(car_name, bffs)


# ─────────────────────────────────────────────────────────────────────────────
# Extraction helpers
# ─────────────────────────────────────────────────────────────────────────────

def _run_pcarstools(pcarstools: str, bff: str, game_dir: str, out_dir: str) -> bool:
    """Extract a .bff with PCarsTools (DOTNET_ROLL_FORWARD=Major)."""
    env = dict(os.environ, DOTNET_ROLL_FORWARD="Major")
    cmd = [pcarstools, "pak", "-i", bff, "-g", game_dir, "-o", out_dir]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=120)
    if r.returncode != 0:
        print(f"    [ERROR] PCarsTools pak: {r.stderr.strip()}")
        return False
    extracted = r.stdout.count("Unpacked:")
    print(f"    Extracted {extracted} files")
    return True


def _decrypt_mebs(pcarstools: str, meb_dir: str) -> int:
    """Run decryptmodel on all .meb files under meb_dir. Returns count of files processed."""
    env = dict(os.environ, DOTNET_ROLL_FORWARD="Major")
    meb_files = list(Path(meb_dir).rglob("*.meb"))
    count = 0
    for meb in meb_files:
        # Only decrypt LOD-A (skip LOD-B/C/D, shadows, blurs)
        n = meb.name.lower()
        if ("_loda" in n or "_loda_" in n) and "_dmg" not in n:
            r = subprocess.run(
                [pcarstools, "decryptmodel", "-i", str(meb)],
                capture_output=True, text=True, env=env, timeout=30
            )
            count += 1
    return count


def _cleanup_dir(d: str):
    """Remove a directory tree (temporary extraction workspace)."""
    import shutil
    shutil.rmtree(d, ignore_errors=True)


# ─────────────────────────────────────────────────────────────────────────────
# VHF (Vehicle Hierarchy File) parser — wheel positions → wheelbase / track
# ─────────────────────────────────────────────────────────────────────────────

def _parse_vhf(car_work_dir: str) -> Optional[dict]:
    """
    Parse the .vhf XML in the extracted vehicle directory to extract:
      - wheelbase:   distance front-axle to rear-axle (metres)
      - track_front: left-to-right wheel centre distance, front (metres)
      - track_rear:  left-to-right wheel centre distance, rear  (metres)
      - wheel_radius: loaded radius from wheel centre height (metres)

    The VHF stores wheel-centre positions as MATRIX Offset="X Y Z" values
    referenced by WHEEL_LF/RF/LR/RR LOD nodes.

    pCARS2 Y-up coordinate system:
      X = lateral  (positive = left)
      Y = vertical (positive = up)
      Z = longitudinal (positive = rear)

    Returns a dict with the above keys, or None if parsing fails.
    """
    import re
    import xml.etree.ElementTree as ET

    # Find the .vhf file (it's a disguised XML, not always parseable by ET directly)
    vhf_files = list(Path(car_work_dir).rglob("*.vhf"))
    if not vhf_files:
        return None
    vhf_path = vhf_files[0]

    try:
        # The VHF has a binary header + embedded XML; read as text and extract XML
        raw = vhf_path.read_bytes()
        # XML starts at first '<'
        xml_start = raw.find(b"<?xml")
        if xml_start == -1:
            xml_start = raw.find(b"<CAR")
        if xml_start == -1:
            return None
        xml_text = raw[xml_start:].decode("utf-8", errors="replace")

        # Build matrix id → offset mapping
        matrices: dict = {}
        for m in re.finditer(
                r'<MATRIX\s+id="(\d+)"\s+Offset="([^"]+)"', xml_text):
            mid = int(m.group(1))
            parts = m.group(2).split()
            if len(parts) >= 3:
                matrices[mid] = [float(p) for p in parts[:3]]  # [X, Y, Z]

        # Find wheel LOD node → MatrixNumber
        wheel_pos: dict = {}   # 'LF'/'RF'/'LR'/'RR' → [X, Y, Z]
        for m in re.finditer(
                r'Name="([^"]*WHEEL_([LR][FR])_LOD0[^"]*)"[^>]*MatrixNumber="(\d+)"',
                xml_text, re.IGNORECASE):
            corner = m.group(2).upper()   # LF, RF, LR, RR
            mat_id = int(m.group(3))
            if mat_id in matrices and corner not in wheel_pos:
                wheel_pos[corner] = matrices[mat_id]

        if len(wheel_pos) < 4:
            return None

        lf, rf, lr, rr = (wheel_pos.get(c) for c in ("LF", "RF", "LR", "RR"))
        if None in (lf, rf, lr, rr):
            return None

        # Wheelbase: |Z_front - Z_rear|  (Z positive = rear)
        wheelbase = abs(((lf[2] + rf[2]) / 2) - ((lr[2] + rr[2]) / 2))
        # Track: |X_left - X_right| (X positive = left in pCARS2)
        track_front = abs(lf[0] - rf[0])
        track_rear  = abs(lr[0] - rr[0])
        # Wheel centre height (= Y coordinate = loaded radius approximation)
        wheel_radius = (lf[1] + rf[1] + lr[1] + rr[1]) / 4

        return {
            "wheelbase":    round(wheelbase, 4),
            "track_front":  round(track_front, 4),
            "track_rear":   round(track_rear, 4),
            "wheel_radius": round(wheel_radius, 4),
        }

    except Exception as exc:
        print(f"    [WARN] VHF parse failed: {exc}")
        return None


# ─────────────────────────────────────────────────────────────────────────────
# SVJ asset update
# ─────────────────────────────────────────────────────────────────────────────

def _apply_vhf(svj: dict, vhf_data: Optional[dict]) -> None:
    """
    Fill chassis wheelbase/track from the VHF only where the physics converter
    left them unset. The converter derives them (and per-axle loaded radius)
    from the vdfm wheel centres, which is per-axle and preferred; the VHF
    wheel_radius is a four-wheel average, so it never overwrites loaded_radius.
    """
    if not vhf_data:
        return
    ch = svj.get("chassis")
    if ch is None:
        return
    for key in ("wheelbase", "track_front", "track_rear"):
        if ch.get(key) is None and vhf_data.get(key):
            ch[key] = vhf_data[key]


_STATION = {"fl": "FL", "fr": "FR", "rl": "RL", "rr": "RR"}


def _apply_bindings(svj: dict, mesh_id: str, svj_nodes: List[str]) -> None:
    """
    Write SVJ visual bindings (spec §22.3) for the glTF nodes the GLB really
    contains, at the binding site each node belongs to. Nodes the GLB lacks
    get no binding: pCARS2 has no separate upright mesh, for instance, so
    uprights stay unbound rather than pointing at a node that doesn't exist.
    """
    def bind(node: str) -> dict:
        return {"mesh_ref": mesh_id, "node": node}

    # Start clean: earlier versions bound nodes the GLB never had
    # (e.g. SVJ::body::upright_fl at corner level).
    def strip(o):
        if isinstance(o, dict):
            o.pop("visual", None)
            for v in o.values():
                strip(v)
        elif isinstance(o, list):
            for v in o:
                strip(v)
    strip(svj)

    susp = svj.get("suspension", {})
    for node in svj_nodes:
        _, category, ident = node.split("::")
        if node == "SVJ::body::chassis" and "chassis" in svj:
            svj["chassis"]["visual"] = bind(node)
        elif node == "SVJ::steering::wheel" and "steering" in svj:
            svj["steering"].setdefault("steering_wheel", {})["visual"] = bind(node)
        elif category in ("wheel", "brake"):
            part, corner = ident.rsplit("_", 1)
            station = susp.get(_STATION.get(corner, ""))
            if station is None:
                continue
            if part == "wheel" and "wheel" in station:
                station["wheel"]["visual"] = bind(node)
            elif part == "disc" and "disc" in station.get("brake", {}):
                station["brake"]["disc"]["visual"] = bind(node)
            elif part == "caliper" and "brake" in station:
                station["brake"].setdefault("caliper", {})["visual"] = bind(node)


def _update_svj_assets(svj_path: str, car_name: str, glb_uri: str,
                        vhf_data: Optional[dict] = None,
                        svj_nodes: Optional[List[str]] = None):
    """
    Update the assets.meshes section in a .svj.json file to reference the GLB,
    bind the GLB's SVJ nodes, and backfill chassis geometry from the VHF
    where it's still missing.
    """
    with open(svj_path, encoding="utf-8") as f:
        svj = json.load(f)

    # Schema requires ^[a-z0-9_]+$; car names like 'porsche_935-78' have hyphens
    mesh_id = re.sub(r"[^a-z0-9_]", "_", car_name.lower())
    svj["assets"] = {
        "meshes": [
            {
                "id":          mesh_id,
                "uri":         glb_uri,
                "description": "Project CARS 2 vehicle mesh (auto-extracted, LOD-A)",
            }
        ]
    }
    _apply_bindings(svj, mesh_id, svj_nodes or [])
    _apply_vhf(svj, vhf_data)

    with open(svj_path, "w", encoding="utf-8") as f:
        json.dump(svj, f, indent=2, ensure_ascii=False)


# ─────────────────────────────────────────────────────────────────────────────
# Per-car pipeline
# ─────────────────────────────────────────────────────────────────────────────

def process_car(
    car_name:    str,
    game_dir:    str,
    pcarstools:  str,
    work_dir:    str,
    glb_dir:     str,
    svj_dir:     Optional[str],
    cleanup:     bool = True,
) -> bool:
    """
    Full pipeline for one car. Returns True on success.
    """
    print(f"\n  [{car_name}]")

    vehicles_bff = Path(game_dir) / "Pakfiles" / "Vehicles"
    if not vehicles_bff.is_dir():
        print(f"    [ERROR] Vehicles directory not found: {vehicles_bff}")
        return False

    # ── 1. Find BFF ──────────────────────────────────────────────────────────
    candidates = _bff_candidates(car_name, vehicles_bff)
    if not candidates:
        print(f"    [SKIP] No matching .bff found for {car_name}")
        return False
    bff = candidates[0]
    print(f"    BFF: {bff.name}")

    # ── 2. Extract ───────────────────────────────────────────────────────────
    car_work = Path(work_dir) / car_name
    car_work.mkdir(parents=True, exist_ok=True)
    if not _run_pcarstools(pcarstools, str(bff), game_dir, str(car_work)):
        return False

    # ── 3. Decrypt MEB files ─────────────────────────────────────────────────
    n_dec = _decrypt_mebs(pcarstools, str(car_work))
    print(f"    Decrypted {n_dec} LOD-A .meb files")

    # ── 4. Convert to GLB ────────────────────────────────────────────────────
    # Find the per-car mesh subdirectory (the extracted BFF unpacks into a named folder)
    # Look for the subfolder containing .meb files
    meb_dirs = list({p.parent for p in car_work.rglob("*.meb")})
    if not meb_dirs:
        print(f"    [FAIL] No .meb files found after extraction")
        return False
    # Use the shallowest directory containing .meb files
    meb_dir = min(meb_dirs, key=lambda p: len(p.parts))

    glb_out = Path(glb_dir) / f"{car_name}.glb"
    Path(glb_dir).mkdir(parents=True, exist_ok=True)

    # Texture search: car-specific dirs + common textures (if extracted)
    tex_dirs = [str(meb_dir), str(meb_dir.parent), str(car_work)]
    common_tex = Path(glb_dir).parent / "common_textures"
    if common_tex.is_dir():
        tex_dirs.append(str(common_tex))

    svj_nodes = convert_car_to_glb(car_name, str(meb_dir), str(glb_out), tex_dirs)
    if not svj_nodes:
        return False

    # ── 5. Parse VHF for wheelbase / track ──────────────────────────────────
    vhf_data = _parse_vhf(str(car_work))
    if vhf_data:
        print(f"    VHF: wheelbase={vhf_data['wheelbase']}m  "
              f"track_f={vhf_data['track_front']}m  "
              f"track_r={vhf_data['track_rear']}m  "
              f"wheel_r={vhf_data['wheel_radius']}m")
    else:
        print(f"    VHF: not found or parse failed")

    # ── 6. Update SVJ ────────────────────────────────────────────────────────
    if svj_dir:
        svj_path = Path(svj_dir) / f"{car_name}.svj.json"
        if svj_path.is_file():
            # Use relative URI from the SVJ output folder to the GLB.
            # Falls back to an absolute path if GLB/SVJ live on different drives
            # (os.path.relpath raises ValueError in that case on Windows).
            try:
                glb_rel = os.path.relpath(str(glb_out), svj_dir).replace("\\", "/")
            except ValueError:
                glb_rel = str(glb_out.resolve()).replace("\\", "/")
            _update_svj_assets(str(svj_path), car_name, glb_rel, vhf_data, svj_nodes)
            print(f"    SVJ updated: assets.meshes → {glb_rel}")

    # ── 7. Cleanup ───────────────────────────────────────────────────────────
    if cleanup:
        _cleanup_dir(str(car_work))
        print(f"    Temp workspace removed")

    return True


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    sys.stdout.reconfigure(encoding="utf-8")

    ap = argparse.ArgumentParser(
        description="Extract pCARS2 vehicle meshes and update SVJ files with GLB references",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples
--------
  # Single car
  python pcars2_extract_meshes.py \\
    --game "E:/SteamLibrary/steamapps/common/Project CARS 2" \\
    --car toyota_gt86 --svj-dir _pcars2/svj_output --glb-dir _pcars2/meshes

  # All cars (will take 15-40 minutes)
  python pcars2_extract_meshes.py \\
    --game "E:/SteamLibrary/steamapps/common/Project CARS 2" \\
    --all --svj-dir _pcars2/svj_output --glb-dir _pcars2/meshes
        """
    )
    ap.add_argument("--game", required=True,
                    help="Project CARS 2 installation directory")
    ap.add_argument("--car", default=None,
                    help="Single car name to process (e.g. toyota_gt86)")
    ap.add_argument("--all", action="store_true",
                    help="Process all cars with existing SVJ files")
    ap.add_argument("--svj-dir", default=None,
                    help="Directory containing .svj.json files (also updated with GLB URIs)")
    ap.add_argument("--glb-dir", default=None,
                    help="Output directory for .glb files (default: <svj-dir>/../meshes)")
    ap.add_argument("--work-dir", default=None,
                    help="Temporary workspace for BFF extraction (default: <glb-dir>/../mesh_work)")
    ap.add_argument("--tools-dir", default=None,
                    help="Directory containing PCarsTools.exe (auto-detected if omitted)")
    ap.add_argument("--no-cleanup", action="store_true",
                    help="Keep temporary BFF extraction files after each car")
    ap.add_argument("--vhf-only", action="store_true",
                    help=(
                        "Quick pass: extract BFF, parse VHF for wheelbase/track, "
                        "update SVJ chassis section ONLY — skip MEB decryption and GLB. "
                        "Useful to backfill chassis geometry after a full batch run."
                    ))

    args = ap.parse_args()

    # ── Locate PCarsTools ─────────────────────────────────────────────────────
    if args.tools_dir:
        pcarstools = str(Path(args.tools_dir) / "PCarsTools.exe")
    else:
        # Try common locations relative to this script
        for candidate in [
            _HERE / "_pcars2" / "tools" / "win-x64" / "PCarsTools.exe",
            _HERE / "tools" / "PCarsTools.exe",
            _HERE / "PCarsTools.exe",
        ]:
            if candidate.is_file():
                pcarstools = str(candidate)
                break
        else:
            ap.error(
                "PCarsTools.exe not found. Specify --tools-dir or place it in "
                "_pcars2/tools/win-x64/"
            )

    print(f"PCarsTools: {pcarstools}")

    # ── Resolve directories ───────────────────────────────────────────────────
    svj_dir = str(Path(args.svj_dir)) if args.svj_dir else None
    glb_dir = str(Path(args.glb_dir)) if args.glb_dir else (
        str(Path(svj_dir).parent / "meshes") if svj_dir else "meshes"
    )
    work_dir = str(Path(args.work_dir)) if args.work_dir else (
        str(Path(glb_dir).parent / "mesh_work")
    )
    game_dir = str(Path(args.game))

    print(f"Game     : {game_dir}")
    print(f"SVJ dir  : {svj_dir or '(none — SVJ not updated)'}")
    print(f"GLB dir  : {glb_dir}")
    print(f"Work dir : {work_dir}")

    cleanup = not args.no_cleanup

    # ── Build car list ────────────────────────────────────────────────────────
    if args.all:
        if not svj_dir or not Path(svj_dir).is_dir():
            ap.error("--all requires --svj-dir pointing at an existing directory")
        cars = sorted(Path(svj_dir).glob("*.svj.json"))
        car_names = [p.stem.replace(".svj", "") for p in cars]
        print(f"\nBatch mode: {len(car_names)} cars")
    elif args.car:
        car_names = [args.car]
    else:
        ap.error("Specify --car <name> or --all")

    # ── Process ───────────────────────────────────────────────────────────────
    ok = err = 0

    if getattr(args, "vhf_only", False):
        # Fast VHF-only pass: extract BFF, parse VHF, update SVJ chassis
        print(f"\nVHF-only mode: {len(car_names)} cars")
        vehicles_bff = Path(game_dir) / "Pakfiles" / "Vehicles"
        for car in car_names:
            print(f"\n  [{car}] VHF", end=" ")
            candidates = _bff_candidates(car, vehicles_bff)
            if not candidates:
                print("-- no BFF"); err += 1; continue
            bff = candidates[0]
            car_work = Path(work_dir) / (car + "_vhf")
            car_work.mkdir(parents=True, exist_ok=True)
            if not _run_pcarstools(pcarstools, str(bff), game_dir, str(car_work)):
                err += 1
                if cleanup: _cleanup_dir(str(car_work))
                continue
            vhf_data = _parse_vhf(str(car_work))
            if vhf_data and svj_dir:
                svj_path = Path(svj_dir) / f"{car}.svj.json"
                if svj_path.is_file():
                    # Keep existing assets.meshes; only update chassis geometry
                    with open(svj_path, encoding="utf-8") as f:
                        svj = json.load(f)
                    _apply_vhf(svj, vhf_data)
                    with open(svj_path, "w", encoding="utf-8") as f:
                        json.dump(svj, f, indent=2, ensure_ascii=False)
                    print(f"wb={vhf_data['wheelbase']}m  trk_f={vhf_data['track_front']}m")
                    ok += 1
            else:
                print("-- VHF parse failed"); err += 1
            if cleanup:
                _cleanup_dir(str(car_work))
    else:
        for car in car_names:
            success = process_car(
                car_name=car,
                game_dir=game_dir,
                pcarstools=pcarstools,
                work_dir=work_dir,
                glb_dir=glb_dir,
                svj_dir=svj_dir,
                cleanup=cleanup,
            )
            if success:
                ok += 1
            else:
                err += 1

    print(f"\nDone: {ok} OK, {err} errors / skipped")


if __name__ == "__main__":
    main()
