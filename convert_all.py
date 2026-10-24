#!/usr/bin/env python3
"""
convert_all.py - Project CARS 2 -> SVJ, one command
====================================================

Runs the whole pipeline against your own Project CARS 2 install:

  1. finds the game (or use --game)
  2. unpacks the physics archives with PCarsTools (skipped when already unpacked)
  3. converts every car's physics to an SVJ file
  4. extracts each car's 3D model (GLB with textures), its collision hulls and
     hinges, and links them into the SVJ file
  5. validates every SVJ file against the SVJ schema and prints a summary

Output: <out>/<car>/<car>.svj.json, <car>.glb and <car>.collision.json

Examples
--------
  python convert_all.py                         # all cars, game auto-detected
  python convert_all.py --game "D:/Steam/steamapps/common/Project CARS 2"
  python convert_all.py --car toyota_gt86       # one car (repeat, or use * patterns)
  python convert_all.py --car "porsche_*" --no-meshes
  python convert_all.py --resume                # skip cars that already have a GLB

Requirements: see README.md (PCarsTools in _pcars2/tools/win-x64/, .NET 7+,
numpy, pygltflib, pillow, jsonschema). The SVJ specification is downloaded into
svj_spec/ when it is missing.
"""

from __future__ import annotations

import argparse
import fnmatch
import importlib.util
import io
import os
import shutil
import subprocess
import sys
import time
import urllib.request
import zipfile
from pathlib import Path
from typing import List, Optional

HERE = Path(__file__).resolve().parent
PCARS = HERE / "_pcars2"
TOOLS_EXE = PCARS / "tools" / "win-x64" / "PCarsTools.exe"
EXTRACTED = PCARS / "out"
SPEC_DIR = HERE / "svj_spec" / "SVJ-standard-vehicle-json-main"
SPEC_ZIP = "https://github.com/RFloEng/SVJ-standard-vehicle-json/archive/refs/heads/main.zip"

STEAM_PATHS = [
    r"C:\Program Files (x86)\Steam\steamapps\common\Project CARS 2",
    r"C:\Program Files\Steam\steamapps\common\Project CARS 2",
    *[rf"{d}:\{p}\steamapps\common\Project CARS 2"
      for d in "CDEFGH" for p in ("Steam", "SteamLibrary", "Games\\Steam", "Games\\SteamLibrary")],
]


def load(name: str):
    """Import a sibling script by file name (they are scripts, not a package)."""
    path = HERE / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def step(msg: str) -> None:
    print(f"\n=== {msg}", flush=True)


def die(msg: str) -> None:
    sys.exit(f"\n[ERROR] {msg}")


# ── 1. Game ───────────────────────────────────────────────────────────────────

def find_game(arg: Optional[str]) -> Path:
    candidates = [arg] if arg else [os.environ.get("PCARS2_DIR", "")] + STEAM_PATHS
    for c in candidates:
        if c and (Path(c) / "Pakfiles" / "PHYSICSMENU.bff").is_file():
            return Path(c)
    if arg:
        die(f"--game does not look like a Project CARS 2 folder (no Pakfiles/PHYSICSMENU.bff): {arg}")
    die("Project CARS 2 not found in the usual Steam locations. "
        "Pass --game \"<path>\" or set PCARS2_DIR.")


# ── 2. Spec + physics archives ───────────────────────────────────────────────

def ensure_spec() -> None:
    if (SPEC_DIR / "schema" / "svj.schema.json").is_file() and \
       (SPEC_DIR / "tools" / "migrate_suspension.py").is_file():
        return
    step("Downloading the SVJ specification (svj_spec/)")
    with urllib.request.urlopen(SPEC_ZIP, timeout=120) as r:
        data = r.read()
    SPEC_DIR.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        z.extractall(SPEC_DIR.parent)
    if not (SPEC_DIR / "schema" / "svj.schema.json").is_file():
        die(f"The downloaded specification has an unexpected layout under {SPEC_DIR.parent}")
    print(f"  SVJ spec unpacked to {SPEC_DIR}")


def ensure_tools(game: Path) -> None:
    if not TOOLS_EXE.is_file():
        die(f"PCarsTools.exe not found at {TOOLS_EXE}\n"
            "Download PCarsTools 1.1.4 from https://github.com/Nenkai/PCarsTools/releases/tag/1.1.4 "
            "and place PCarsTools.exe there (see README.md).")
    dll = TOOLS_EXE.parent / "oo2core_7_win64.dll"
    src = game / "oo2core_4_win64.dll"
    if not dll.is_file() and src.is_file():
        shutil.copy2(src, dll)             # PCarsTools wants v7; the game's v4 decodes these archives
        print(f"  Copied {src.name} -> {dll.name}")


def run_tools(args: List[str], timeout: int = 1800) -> subprocess.CompletedProcess:
    env = dict(os.environ, DOTNET_ROLL_FORWARD="LatestMajor")
    return subprocess.run([str(TOOLS_EXE), *args], capture_output=True, text=True,
                          timeout=timeout, env=env)


def ensure_physics(game: Path, force: bool, pc2) -> None:
    have = len(pc2.list_cars(str(EXTRACTED))) if EXTRACTED.is_dir() else 0
    if have and not force:
        print(f"  Physics already unpacked ({have} cars in {EXTRACTED}); use --reextract to redo")
        return
    step("Unpacking the physics archives (a few minutes)")
    ensure_tools(game)
    for bff, out in (("PHYSICSMENU.bff", "PHYSICSMENU"), ("PHYSICSPERSISTENT.bff", "PHYSICSPERSISTENT")):
        print(f"  {bff} ...", flush=True)
        r = run_tools(["pak", "-i", str(game / "Pakfiles" / bff), "-g", str(game),
                       "-o", str(EXTRACTED / out)])
        if r.returncode != 0:
            die(f"PCarsTools failed on {bff}:\n{(r.stderr or r.stdout)[:600]}")


# ── 3-5. Convert ─────────────────────────────────────────────────────────────

def select(cars: List[str], patterns: Optional[List[str]]) -> List[str]:
    if not patterns:
        return cars
    out = [c for c in cars if any(fnmatch.fnmatch(c, p) for p in patterns)]
    missing = [p for p in patterns if not any(fnmatch.fnmatch(c, p) for c in cars)]
    if missing:
        print(f"  [WARN] no car matches: {', '.join(missing)}")
    return out


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="Convert Project CARS 2 cars to SVJ (physics + 3D models).",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="Examples" + __doc__.split("Examples", 1)[1])
    ap.add_argument("--game", help="Project CARS 2 folder (default: auto-detect, or $PCARS2_DIR)")
    ap.add_argument("--out", default=str(PCARS / "cars"), help="output folder (default: _pcars2/cars)")
    ap.add_argument("--car", action="append", help="car name or * pattern; repeatable (default: all)")
    ap.add_argument("--no-meshes", action="store_true", help="physics only: no GLB, collision or hinges")
    ap.add_argument("--resume", action="store_true", help="skip the model step for cars that already have a GLB")
    ap.add_argument("--reextract", action="store_true", help="unpack the physics archives again")
    ap.add_argument("--keep-work", action="store_true", help="keep the temporary model extraction folders")
    args = ap.parse_args()

    t0 = time.time()
    step("Locating the game")
    game = find_game(args.game)
    print(f"  {game}")
    ensure_spec()

    pc2 = load("pcars2_to_svj")
    ensure_physics(game, args.reextract, pc2)

    cars = select(pc2.list_cars(str(EXTRACTED)), args.car)
    if not cars:
        die("No cars to convert.")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    step(f"Converting physics: {len(cars)} cars -> {out}")
    ok, failed, invalid = [], [], []
    for car in cars:
        try:
            svj = pc2.convert_car(car, str(EXTRACTED))
            pc2.write_svj(svj, str(out / car / f"{car}.svj.json"))
            ok.append(car)
        except Exception as exc:
            failed.append((car, f"{type(exc).__name__}: {exc}"))
            print(f"  [ERROR] {car}: {exc}")

    mesh_ok, mesh_failed = [], []
    if not args.no_meshes:
        step(f"Extracting 3D models, collision hulls and hinges: {len(ok)} cars (about 7 s each)")
        ensure_tools(game)
        meshes = load("pcars2_extract_meshes")
        work = out.parent / "mesh_work"
        for i, car in enumerate(ok, 1):
            if args.resume and (out / car / f"{car}.glb").is_file():
                mesh_ok.append(car)
                continue
            print(f"\n[{i}/{len(ok)}]", end="", flush=True)
            try:
                good = meshes.process_car(car_name=car, game_dir=str(game), pcarstools=str(TOOLS_EXE),
                                          work_dir=str(work), cars_dir=str(out),
                                          cleanup=not args.keep_work)
            except Exception as exc:
                print(f"    [ERROR] {car}: {type(exc).__name__}: {exc}")
                good = False
            (mesh_ok if good else mesh_failed).append(car)

    step("Validating against the SVJ schema")
    for car in ok:
        path = out / car / f"{car}.svj.json"
        try:
            import json
            problems = pc2.validate_svj(json.loads(path.read_text(encoding="utf-8")))
        except Exception as exc:
            problems = [f"unreadable: {exc}"]
        if problems:
            invalid.append((car, problems[0]))
    print("  schema validation unavailable (pip install jsonschema)" if pc2.validate_svj({}) is None
          else f"  {len(ok) - len(invalid)}/{len(ok)} valid")

    # One row per car, like the other converters' batch_summary.csv
    import csv, json as _json
    rows = []
    bad = {c for c, _ in invalid} | {c for c, _ in failed}
    for car in cars:
        path = out / car / f"{car}.svj.json"
        hinged = 0
        if path.is_file():
            try:
                d = _json.loads(path.read_text(encoding="utf-8"))
                hinged = sum(1 for b in d.get("chassis", {}).get("mass_bodies", []) if b.get("x_source"))
            except (OSError, ValueError):
                pass
        rows.append({"car": car, "svj": car in ok, "schema_valid": car in ok and car not in bad,
                     "glb": (out / car / f"{car}.glb").is_file(), "hinged_parts": hinged})
    with open(out / "batch_summary.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    mins = (time.time() - t0) / 60
    step(f"Done in {mins:.1f} min")
    print(f"  physics -> SVJ : {len(ok)}/{len(cars)}")
    if not args.no_meshes:
        print(f"  models  -> GLB : {len(mesh_ok)}/{len(ok)}")
    for car, why in failed:
        print(f"  FAILED  {car}: {why}")
    for car in mesh_failed:
        print(f"  NO MODEL {car}")
    for car, why in invalid:
        print(f"  INVALID {car}: {why}")
    print(f"  output  : {out}  (batch_summary.csv lists every car)")
    sys.exit(1 if failed or invalid else 0)


if __name__ == "__main__":
    main()
