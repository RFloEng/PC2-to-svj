#!/usr/bin/env python3
"""
svj_usd.py - OpenUSD copies of the extracted car models (SVJ spec 22.10, v0.105 / v0.106)
==========================================================================================

Next to each <car>.glb this writes <car>.usdz with the spec's own tools/gltf2usd.py (metres,
the SVJ binding names as svj:node, PBR materials with the textures baked in), then points the
car's assets.meshes entry at the USD file and keeps the GLB as its `preview`:

    {"id": ..., "uri": "<car>.usdz", "format": "usd", "preview": "<car>.glb"}

Only shapes and appearance go into the USD; every physical datum stays in the SVJ JSON.

Needs the usd-core library (pip install usd-core). Without it nothing is written and the
GLB stays the asset.

    python svj_usd.py <library folder> [--jobs 4] [--force]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Optional, Tuple

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))        # worker processes import this module by name
_TOOL = _HERE / "svj_spec" / "SVJ-standard-vehicle-json-main" / "tools" / "gltf2usd.py"


def available() -> bool:
    """True when usd-core and the spec's converter are both there."""
    if not _TOOL.is_file():
        return False
    try:
        import pxr  # noqa: F401
    except ImportError:
        return False
    return True


def _load_converter():
    spec = importlib.util.spec_from_file_location("gltf2usd", _TOOL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["gltf2usd"] = mod
    spec.loader.exec_module(mod)
    return mod


def convert_car(car_dir: str, force: bool = False) -> Tuple[str, str]:
    """Write <car>.usdz for one car folder and update its SVJ. Returns (car, status)."""
    folder = Path(car_dir)
    car = folder.name
    glb, usdz, svj_path = folder / f"{car}.glb", folder / f"{car}.usdz", folder / f"{car}.svj.json"
    if not glb.is_file() or not svj_path.is_file():
        return car, "no model"
    try:
        if force or not usdz.is_file() or usdz.stat().st_mtime < glb.stat().st_mtime:
            conv = _load_converter()
            tmp = usdz.with_suffix(".tmp.usdz")
            conv.convert(str(glb), str(tmp), "Car")
            tmp.replace(usdz)
        svj = json.loads(svj_path.read_text(encoding="utf-8"))
        meshes = svj.get("assets", {}).get("meshes") or []
        if not meshes:
            return car, "no mesh entry"
        m = meshes[0]
        m["uri"], m["format"], m["preview"] = usdz.name, "usd", glb.name
        m["description"] = str(m.get("description", "")).replace("vehicle mesh", "vehicle model", 1)
        svj_path.write_text(json.dumps(svj, indent=2, ensure_ascii=False), encoding="utf-8")
        return car, "ok"
    except Exception as exc:                       # one broken car must not stop the batch
        return car, f"error: {type(exc).__name__}: {exc}"


def convert_all(folder: Path, jobs: int = 4, force: bool = False, cars: Optional[list] = None):
    dirs = sorted(d for d in folder.iterdir() if d.is_dir() and (d / f"{d.name}.glb").is_file())
    if cars:
        dirs = [d for d in dirs if d.name in set(cars)]
    results = []
    with ProcessPoolExecutor(max_workers=max(1, jobs)) as ex:
        futs = [ex.submit(convert_car, str(d), force) for d in dirs]
        for i, f in enumerate(futs, 1):
            r = f.result()
            results.append(r)
            if r[1] != "ok":
                print(f"  [{i}/{len(futs)}] {r[0]}: {r[1]}")
            elif i % 25 == 0:
                print(f"  [{i}/{len(futs)}] ...", flush=True)
    return results


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", help="a game's library folder: one sub-folder per car")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--force", action="store_true", help="rewrite the USD even if it is newer than the GLB")
    ap.add_argument("--car", action="append")
    args = ap.parse_args()
    if not available():
        sys.exit("usd-core is not installed (pip install usd-core) or svj_spec/ is missing")
    res = convert_all(Path(args.folder), args.jobs, args.force, args.car)
    ok = sum(1 for _, s in res if s == "ok")
    print(f"{ok}/{len(res)} cars now use a USD asset")


if __name__ == "__main__":
    main()
