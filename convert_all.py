#!/usr/bin/env python3
"""
convert_all.py - Project CARS 2 -> SVJ, one command
====================================================

Runs the whole pipeline against your own Project CARS 2 install:

  1. finds the game (or use --game)
  2. unpacks the physics archives with PCarsTools (skipped when already unpacked)
     and keeps the physics files each car's own archive loads
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
  python convert_all.py --edition pc1           # Project CARS (needs the XNA Framework 4.0)

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
TOOLS_DIR = HERE / "_pcars2" / "tools"
SPEC_DIR = HERE / "svj_spec" / "SVJ-standard-vehicle-json-main"
SPEC_ZIP = "https://github.com/RFloEng/SVJ-standard-vehicle-json/archive/refs/heads/main.zip"

# One profile per game. Project CARS keeps all its physics in PHYSICSPERSISTENT, packs its
# archives with Xbox compression (the 32-bit PCarsTools, which needs the XNA Framework 4.0)
# and needs --game-type PC1.
PROFILES = {
    "pc2": {
        "label": "Project CARS 2", "work": "_pcars2", "steam": ["Project CARS 2"],
        "exe": "pCARS2.exe", "env": "PCARS2_DIR", "tools": "win-x64", "game_type": None,
        "archives": [("PHYSICSMENU.bff", "PHYSICSMENU"), ("PHYSICSPERSISTENT.bff", "PHYSICSPERSISTENT")],
    },
    "pc1": {
        "label": "Project CARS", "work": "_pcars1", "steam": ["pCars", "Project CARS"],
        "exe": "pCARS.exe", "env": "PCARS1_DIR", "tools": "win-x86", "game_type": "PC1",
        "archives": [("PHYSICSPERSISTENT.bff", "PHYSICSPERSISTENT"), ("HRDFPERSISTENT.bff", "HRDFPERSISTENT"),
                     ("PHYSICSBOOTFLOW.bff", "PHYSICSBOOTFLOW")],
    },
}
PROFILE = PROFILES["pc2"]
PCARS = HERE / PROFILE["work"]
TOOLS_EXE = TOOLS_DIR / PROFILE["tools"] / "PCarsTools.exe"
EXTRACTED = PCARS / "out"


def configure(edition: str) -> None:
    """Select the game profile: paths, tool, archives."""
    global PROFILE, PCARS, TOOLS_EXE, EXTRACTED
    PROFILE = PROFILES[edition]
    PCARS = HERE / PROFILE["work"]
    TOOLS_EXE = TOOLS_DIR / PROFILE["tools"] / "PCarsTools.exe"
    EXTRACTED = PCARS / "out"


def steam_paths() -> List[str]:
    out = []
    for name in PROFILE["steam"]:
        out += [rf"C:\Program Files (x86)\Steam\steamapps\common\{name}",
                rf"C:\Program Files\Steam\steamapps\common\{name}"]
        out += [rf"{d}:\{p}\steamapps\common\{name}"
                for d in "CDEFGH" for p in ("Steam", "SteamLibrary", "Games\\Steam", "Games\\SteamLibrary")]
    return out


def edition_of(folder: Path) -> Optional[str]:
    """pc2 if the folder holds pCARS2.exe, pc1 if it holds pCARS.exe or pCARS64.exe."""
    if (folder / "pCARS2.exe").is_file():
        return "pc2"
    if (folder / "pCARS.exe").is_file() or (folder / "pCARS64.exe").is_file():
        return "pc1"
    return None


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
    candidates = [arg] if arg else [os.environ.get(PROFILE["env"], "")] + steam_paths()
    for c in candidates:
        if c and (Path(c) / "Pakfiles" / "PHYSICSPERSISTENT.bff").is_file() and edition_of(Path(c)) in (None, _EDITION):
            return Path(c)
    if arg:
        die(f"--game does not look like a {PROFILE['label']} folder "
            f"(no Pakfiles/PHYSICSPERSISTENT.bff, or it is the other game): {arg}")
    die(f"{PROFILE['label']} not found in the usual Steam locations. "
        f"Pass --game \"<path>\" or set {PROFILE['env']}.")


_EDITION = "pc2"


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
            f"and place the {PROFILE['tools']} PCarsTools.exe there (see README.md).")
    if PROFILE["game_type"] == "PC1":
        if not _xna_installed():
            die("Project CARS archives need the Microsoft XNA Framework 4.0 Redistributable "
                "(it supplies the Xbox decompression PCarsTools calls). Install xnafx40_redist.msi "
                "from Microsoft, then run this again.")
        return
    dll = TOOLS_EXE.parent / "oo2core_7_win64.dll"
    src = game / "oo2core_4_win64.dll"
    if not dll.is_file() and src.is_file():
        shutil.copy2(src, dll)             # PCarsTools wants v7; the game's v4 decodes these archives
        print(f"  Copied {src.name} -> {dll.name}")


def _xna_installed() -> bool:
    """True when the XNA Framework 4.0 registered its native library path."""
    try:
        import winreg
        for view in (winreg.KEY_WOW64_32KEY, winreg.KEY_WOW64_64KEY):
            try:
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Microsoft\XNA\Framework\v4.0",
                                    0, winreg.KEY_READ | view):
                    return True
            except OSError:
                continue
    except ImportError:
        pass
    return False


def run_tools(args: List[str], timeout: int = 1800) -> subprocess.CompletedProcess:
    env = dict(os.environ, DOTNET_ROLL_FORWARD="LatestMajor")
    extra = ["--game-type", PROFILE["game_type"]] if PROFILE["game_type"] else []
    return subprocess.run([str(TOOLS_EXE), *args, *extra], capture_output=True, text=True,
                          timeout=timeout, env=env)


def ensure_physics(game: Path, force: bool, pc2) -> None:
    have = len(pc2.list_cars(str(EXTRACTED))) if EXTRACTED.is_dir() else 0
    todo = [(bff, out) for bff, out in PROFILE["archives"] if force or not (EXTRACTED / out).is_dir()]
    if have and not todo:
        print(f"  Physics already unpacked ({have} cars in {EXTRACTED}); use --reextract to redo")
        return
    step("Unpacking the physics archives (a few minutes)")
    ensure_tools(game)
    for bff, out in todo:
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
    ap = argparse.ArgumentParser(description="Convert Project CARS and Project CARS 2 cars to SVJ (physics + 3D models).",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="Examples" + __doc__.split("Examples", 1)[1])
    ap.add_argument("--game", help="game folder (default: auto-detect, or $PCARS2_DIR / $PCARS1_DIR)")
    ap.add_argument("--edition", choices=sorted(PROFILES), default=None,
                    help="pc2 (Project CARS 2, the default) or pc1 (Project CARS); inferred from --game when it holds the exe")
    ap.add_argument("--out", default=None, help="output folder (default: _pcars2/cars or _pcars1/cars)")
    ap.add_argument("--car", action="append", help="car name or * pattern; repeatable (default: all)")
    ap.add_argument("--no-meshes", action="store_true", help="physics only: no GLB, collision or hinges")
    ap.add_argument("--resume", action="store_true", help="skip the model step for cars that already have a GLB")
    ap.add_argument("--reextract", action="store_true", help="unpack the physics archives again")
    ap.add_argument("--keep-work", action="store_true", help="keep the temporary model extraction folders")
    args = ap.parse_args()

    global _EDITION
    edition = args.edition or (edition_of(Path(args.game)) if args.game else None) or "pc2"
    _EDITION = edition
    configure(edition)
    if args.out is None:
        args.out = str(PCARS / "cars")
    t0 = time.time()
    step(f"Locating {PROFILE['label']}")
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

    # Each car's own archive holds the physics files it really loads (some are shared with
    # another car and are not in the common physics archives): keep them, once.
    own = PCARS / "car_physics"
    todo = [c for c in cars if args.reextract or not (own / c / "chassis").is_dir()]
    if todo:
        step(f"Extracting the physics files of each car's own archive: {len(todo)} cars (about 1 s each)")
        ensure_tools(game)
        meshes = load("pcars2_extract_meshes")
        meshes.set_game_type(PROFILE["game_type"])
        failed_own = []
        for car in todo:
            try:
                good = meshes.extract_car_physics(car, str(game), str(TOOLS_EXE), str(PCARS / "mesh_work"), str(own))
            except Exception as exc:
                print(f"    [ERROR] {car}: {type(exc).__name__}: {exc}")
                good = False
            if not good:
                failed_own.append(car)
        if failed_own and PROFILE["game_type"] == "PC1":
            # Project CARS keeps a few engine definitions that no car archive uses: no wheels, no car
            print(f"  [WARN] not cars (no archive of their own, skipped): {', '.join(failed_own)}")
            cars = [c for c in cars if c not in failed_own]
            if not cars:
                die("No cars to convert.")
        elif failed_own:
            print(f"  [WARN] no own archive for: {', '.join(failed_own)} (looked up by name instead)")

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
        meshes.set_game_type(PROFILE["game_type"])
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
