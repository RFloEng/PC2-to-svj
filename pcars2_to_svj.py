#!/usr/bin/env python3
"""
Project CARS 2 → SVJ Converter
================================
Converts extracted Project CARS 2 physics files to Standard Vehicle JSON (SVJ).

Prerequisites
-------------
The pCARS2 physics archives must first be extracted with PCarsTools 1.1.4:
  https://github.com/Nenkai/PCarsTools/releases/download/1.1.4/PCarsTools.exe.zip

  1. Copy the game's oo2core_4_win64.dll next to PCarsTools.exe and rename it
     oo2core_7_win64.dll  (the game ships v4; the tool expects v7 — both work).
  2. Run:
       PCarsTools.exe pak -i <game>/Pakfiles/PHYSICSPERSISTENT.bff \
                         -g <game_dir> -o <out>/PHYSICSPERSISTENT
       PCarsTools.exe pak -i <game>/Pakfiles/PHYSICSMENU.bff \
                         -g <game_dir> -o <out>/PHYSICSMENU
  3. Point this script at the <out> directory with --dir.

Data coverage
-------------
  SECTION                    SOURCE           STATUS
  chassis.mass_total         mrdf             ✅ spec-sheet weight
  chassis.inertia            cdfbin           ✅ Inertia; axis order inferred (roll < pitch < yaw)
  chassis.center_of_gravity  mrdf+cdfbin      ⚠ X from the rear weight fraction (cdfbin CGRearRange agrees); Z = CGHeight + ride height (assumed plane)
  powertrain.layout          —                ❌ defaults to FR
  powertrain.engine          edfbin           ✅ full RPM + torque curve
  powertrain.gearbox.ratios  gdfbin           ✅ validated on GT86, FR3.5, NSX
  powertrain.differentials   gdfbin+driveline ⚠ final_drive; AWD diff layout from driveline.rg; type defaults open
  powertrain.gearbox.type    driveline.rg     ✅ manual / sequential / amt / dct from the setup
  powertrain.clutch          driveline.rg     ✅ type + max torque
  powertrain.engine.inertia  driveline.rg     ⚠ crankshaft 1-D mass, unit assumed kg*m^2
  suspension.*.spring.rate   cdfbin           ✅ SpringRange + setting, times SpringMult (wheel rate)
  suspension.*.damper        cdfbin           ✅ slow/fast bump and rebound, times DamperMult; knee speed assumed
  suspension.*.alignment     cdfbin           ✅ default camber, toe, caster
  suspension.*.wheel.mass    sdfbin           ✅ Mass and Inertia of the wheel and tyre
  suspension.*.upright.mass  sdfbin           ✅ Mass of the non-rotating unsprung assembly
  suspension.*.wheel_center  vdfm             ✅ exact wheel-centre XYZ from physics
  suspension.*.topology      sdfbin+vdfm      ✅ link records, placed on the vdfm wheel centres (estimate when undecodable)
  suspension.*.wheel         hdtbin+vdfm      ✅ D from b31155cd; C from b1033b86; E from c0eaf58d
  suspension.*.tire_dims     vdfm             ✅ section_width + free_radius per axle
  suspension.*.brake         cdfbin           ⚠ torque and bias only (kept as _ keys); disc size is a placeholder
  steering                   cdfbin           ✅ ratio and lock; rack geometry is a placeholder
  aerodynamics               cdfbin           ⚠ the game's aero model kept by name in x_pcars2; Cd/Cl are placeholders

Field names: the chassis file keys its fields by hash; pcars2_hashnames.json maps
them to names (from the Project CARS / AMS2 translation by JDougNY and GvsE).

Usage
-----
  python pcars2_to_svj.py --dir ./extracted --list
  python pcars2_to_svj.py --dir ./extracted --car acura_nsx_2017 --out nsx.svj.json
  python pcars2_to_svj.py --dir ./extracted --all --outdir ./cars

Requirements
------------
  shcb_reader.py must be in the same directory as this script.
  scipy / numpy optional: enables Pacejka fitting (falls back to estimation).

SVJ schema version targeted: 0.102 (unified suspension, spec section 25)
Coordinate convention: SAE J670 (Z-down).  CG Z is set to zero pending extraction.
Units: SI — kg, m, N, Pa, N·m, rad.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import re
import struct
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ── Load shcb_reader sibling ──────────────────────────────────────────────────
_HERE = Path(__file__).parent
_SHCB_PATH = _HERE / "shcb_reader.py"
if not _SHCB_PATH.exists():
    sys.exit(f"[ERROR] shcb_reader.py not found next to this script: {_SHCB_PATH}")
_spec = importlib.util.spec_from_file_location("shcb_reader", str(_SHCB_PATH))
_shcb_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_shcb_mod)
ShCBFile = _shcb_mod.ShCBFile

_spec_n = importlib.util.spec_from_file_location("pcars2_names", str(_HERE / "pcars2_names.py"))
_names_mod = importlib.util.module_from_spec(_spec_n)
_spec_n.loader.exec_module(_names_mod)
rank_candidates = _names_mod.rank_candidates

_spec_ch = importlib.util.spec_from_file_location("pcars2_chassis", str(_HERE / "pcars2_chassis.py"))
_chassis_mod = importlib.util.module_from_spec(_spec_ch)
_spec_ch.loader.exec_module(_chassis_mod)

_spec_c = importlib.util.spec_from_file_location("pcars2_collision", str(_HERE / "pcars2_collision.py"))
_collision_mod = importlib.util.module_from_spec(_spec_c)
_spec_c.loader.exec_module(_collision_mod)

# ── vdfm name aliases (SVJ car name → vdfm stem when they differ) ─────────────
_VDFM_ALIASES: Dict[str, str] = {
    "audi_r8_lms":                              "audi_r8_lms15",
    "bmw_320turbo_group5":                      "bmw_320_group5",
    "dallara_dw12c_oval":                       "dallara_dw12c_2016_oval",
    "dallara_dw12c_road":                       "dallara_dw12c_2016_road",
    "ford_sierra_rs500_cosworth":               "ford_sierra_rs500a",
    "ginetta_lmp3":                             "ginetta_g57_lmp3",
    "honda_24_concept":                         "honda_2and4",
    "honda_civic_type-r":                       "honda_civic_typer",
    "lamborghini_aventador_lp700-4":            "lamborghini_aventador",
    "lamborghini_huracan_lp610-4":              "lamborghini_huracan",
    "lamborghini_huracan_super-trofeo_lp620-2": "lamborghini_huracan_supertrofeo",
    "lamborghini_veneno_lp750-4":               "lamborghini_veneno",
    "ligier_jsp2-honda":                        "ligier_jsp2_honda",
    "ligier_jsp2-judd":                         "ligier_jsp2_judd",
    "lotus_51":                                 "lotus_51_f3",
    "mercedes-sauber_c9":                       "sauber_c9",
    "mercedes_300sel_68amg":                    "mercedes_300sel_amg",
    "mitsubishi_sva_evo6rs":                    "mitsubishi_lancer_sva",
    "oreca_03_nissan":                          "oreca_03",
    "porsche_935-78":                           "porsche_935_78md",
    "porsche_935-80":                           "porsche_935_80",
}

# Optional Pacejka fitting
try:
    import numpy as np
    from scipy.optimize import curve_fit
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ─────────────────────────────────────────────────────────────────────────────
# Directory / file helpers
# ─────────────────────────────────────────────────────────────────────────────

SVJ_VERSION = "0.102"
_migrator = None


def _load_migrator():
    """The SVJ standard's suspension migration (legacy corner suspension -> spec section 25),
    vendored in svj_migrate/ (refresh it with vendor_svj_migrate.py)."""
    global _migrator
    if _migrator is None:
        if str(_HERE) not in sys.path:
            sys.path.insert(0, str(_HERE))
        import svj_migrate
        _migrator = svj_migrate
    return _migrator


def _unify_suspension(svj: Dict) -> Dict:
    """
    Convert the corner-based suspension this module builds into the unified model
    (bodies, joints, elements and wheel_stations[], spec section 25) with the
    spec's own migration tool, and declare the current spec version.
    """
    unified, notes = _load_migrator().migrate(svj)
    unified["_metadata"]["version"] = SVJ_VERSION
    # The migration keeps a spring's or damper's numbers but not where they came
    # from; carry the provenance notes over so estimates stay flagged.
    by_id = {c.get("id"): c for c in unified.get("constraints", []) if isinstance(c, dict)}
    for corner, data in svj.get("suspension", {}).items():
        for kind in ("spring", "damper"):
            src, dst = data.get(kind), by_id.get(f"{kind}_{corner.lower()}")
            if isinstance(src, dict) and dst is not None:
                for key in ("_source", "_note", "_est", "x_pcars2"):
                    if key in src:
                        dst[key] = src[key]
    for n in notes:
        print(f"  [migrate] {n}")
    return unified


def _physics_dirs(extracted_root: str) -> Dict[str, Optional[str]]:
    """
    Locate the per-type physics subdirectories under the extracted root.
    Returns a dict of {type_name: absolute_path_or_None}, plus "game": "pc2" or "pc1".

    Project CARS 2 splits the physics over two archives (PHYSICSMENU holds the
    engine, chassis, gearbox and suspension files; PHYSICSPERSISTENT the descriptors,
    spec sheets and tyres). Project CARS keeps its chassis, engine, gearbox and
    suspension files in PHYSICSPERSISTENT, the spec sheets (.mrdf) in HRDFPERSISTENT and
    the tyres (.hdtbin) in PHYSICSBOOTFLOW and the descriptors (.vdfm) in HRDFPERSISTENT. Its
    its descriptors (.vdfm, in HRDFPERSISTENT) have a shorter data section, so the wheel block is
    found by pattern (_wheel_block_offset); a car's .vhf is the fallback (_parse_geometry_json). A root
    with no PHYSICSMENU engines is read as Project CARS.
    """
    root = Path(extracted_root)
    menu = root / "PHYSICSMENU" / "vehicles" / "physics"
    pers = root / "PHYSICSPERSISTENT" / "vehicles" / "physics"
    pc1 = not (menu / "engines").is_dir() and (pers / "engines").is_dir()
    base = pers if pc1 else menu
    hrdf = root / "HRDFPERSISTENT" / "vehicles" / "physics"      # Project CARS spec sheets
    boot = root / "PHYSICSBOOTFLOW" / "vehicles" / "physics"     # Project CARS tyres

    def d(p): return str(p) if p.is_dir() else None

    return {
        "game":       "pc1" if pc1 else "pc2",
        "engines":    d(base / "engines"),
        "chassis":    d(base / "chassis"),
        "gearbox":    d(base / "gearbox"),
        "suspension": d(base / "suspension"),
        "tyres":      d(boot / "tyres") if pc1 else d(pers / "tyres"),
        "vehicles":   d(hrdf / "vehicles") if pc1 else d(pers / "vehicles"),   # .vdfm master descriptors
        "statistics": d(hrdf / "statistics") if pc1 else d(pers / "statistics"),  # .mrdf spec sheets
        "driveline":  None if pc1 else d(pers / "driveline"),  # driveline.rg (text, per-car setups)
        # The files each car's own archive loads (extract_car_physics); preferred when present.
        "car_physics": d(root.parent / "car_physics"),
    }


GAME_LABEL = {"pc2": "Project CARS 2", "pc1": "Project CARS"}


_DRIVELINE_CACHE: Dict[str, Dict] = {}


def _load_driveline(dirs: Dict[str, Optional[str]]) -> Dict:
    """
    Parse driveline.rg (plain text) into {"records", "groups", "cars"}.

      RECORD <name> <layout>      then tab-indented "key value" lines
      RECORDGROUP <name>          then tab-indented member record names
      l_driveline records bind a car (name "CAR_NAME") to a group (contents).
    """
    folder = dirs.get("driveline")
    path = Path(folder) / "driveline.rg" if folder else None
    if not path or not path.is_file():
        return {"records": {}, "groups": {}, "cars": {}}
    key = str(path)
    if key in _DRIVELINE_CACHE:
        return _DRIVELINE_CACHE[key]
    records: Dict[str, Dict] = {}
    groups: Dict[str, List[str]] = {}
    cur: Optional[Dict] = None
    cur_group: Optional[List[str]] = None
    for raw in path.read_text(encoding="latin-1").splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        if not raw[0].isspace():
            cur = cur_group = None
            parts = line.split()
            if parts[0] == "RECORD" and len(parts) >= 3:
                cur = {"layout": parts[2], "attrs": {}}
                records[parts[1]] = cur
            elif parts[0] == "RECORDGROUP" and len(parts) >= 2:
                cur_group = groups.setdefault(parts[1], [])
            continue
        body = line.strip()
        if cur is not None:
            k, _, v = body.partition(" ")
            cur["attrs"][k] = v.strip().strip('"')
        elif cur_group is not None:
            cur_group.append(body.split()[0])
    cars = {}
    for rec in records.values():
        if rec["layout"] == "l_driveline" and rec["attrs"].get("name"):
            cars[rec["attrs"]["name"].upper()] = rec["attrs"].get("contents")
    out = {"records": records, "groups": groups, "cars": cars}
    _DRIVELINE_CACHE[key] = out
    return out


def _num(v: Optional[str]) -> Optional[float]:
    try:
        return float(v) if v is not None else None
    except ValueError:
        return None


def _parse_driveline(dirs: Dict[str, Optional[str]], car: str) -> Optional[Dict]:
    """
    Per-car driveline facts from driveline.rg, or None when the car has no entry.

    Returns group, gearbox_type, clutch (type/max_torque), crank_inertia and
    diffs (locations present). Differential type and preload are mostly named
    per-car variables (R_preload, R_gear_lsd ...) that aren't stored anywhere in
    the extracted files, so they are not reported.
    """
    dl = _load_driveline(dirs)
    group = dl["cars"].get(car.upper())
    if not group or group not in dl["groups"]:
        return None
    members = [(n, dl["records"].get(n)) for n in dl["groups"][group]]
    info: Dict[str, Any] = {"group": group}

    # Gearbox type from the group's own gearbox record: the quick-shift dog box
    # (gearbox_GT3) and Hewland boxes are sequential, the synchro AMT is a dual
    # clutch where the group says so and an automated manual otherwise.
    gb_name = next((n.lower() for n, r in members
                    if r and r["layout"] == "l_gearbox"), "")
    g = group.upper()
    if "amt_synchro" in gb_name:
        info["gearbox_type"] = "dct" if ("DUAL" in g and "CLUTCH" in g) else "amt"
    elif "gt3" in gb_name or "hewland" in gb_name:
        info["gearbox_type"] = "sequential"
    else:
        info["gearbox_type"] = "manual"

    diffs = []
    for name, rec in members:
        if rec is None:
            continue
        a = rec["attrs"]
        if rec["layout"] in ("l_clutch", "l_centrifugal_clutch") and "clutch" not in info:
            t = _num(a.get("clutch:torque"))
            if t:
                info["clutch"] = {
                    "type": "dual_clutch" if info["gearbox_type"] == "dct" else "single_plate",
                    "max_torque": t}
        elif rec["layout"] == "shaft" and a.get("name", "").strip('"').startswith("crankshaft"):
            m = _num(a.get("sim:mass"))
            if m:
                info["crank_inertia"] = m
        elif rec["layout"] == "amalgam_diff":
            low = name.lower()
            loc = "center" if "center" in low else "front" if "front" in low else "rear"
            if loc not in diffs:
                diffs.append(loc)
    info["diffs"] = diffs
    return info


def _extra_diffs(dl_info: Optional[Dict], drive: Optional[str], driven: str) -> List[Dict]:
    """Front/centre diffs of an AWD car, as listed by its driveline.rg setup."""
    if not dl_info or drive != "AWD":
        return []
    out = []
    for loc in ("front", "center", "rear"):
        if loc in dl_info["diffs"] and loc != driven:
            out.append({
                "id": f"diff_{loc}",
                "location": loc,
                "type": "open",
                "final_drive": 1.0,
                "_note": ("Present in the car's driveline.rg setup; type defaults to 'open' "
                          "and final_drive 1.0 is a placeholder (the game's single final-drive "
                          "ratio is on the driven-axle entry)."),
            })
    return out


def _apply_chassis_diffs(diffs: List[Dict], cd: Optional[Dict]) -> None:
    """
    Set each differential's type and lock from the chassis file (pcars2_chassis.parse_diffs).

    A type is on when its Option setting is 1; some cars have several on, and
    then clutch LSD wins over geared (Torsen-like) over viscous, a priority
    that is a guess (flagged). A geared bias ratio r is a lock of (r-1)/(r+1).
    """
    for d in diffs:
        info = (cd or {}).get(d.get("location"))
        act = (info or {}).get("active")
        if not act:
            continue
        x = {"active_options": list(act)}
        if "ClutchLsd" in act:
            d["type"] = "lsd_clutch"
            for key, src in (("preload", "preload_Nm"), ("ramp_angle_power", "ramp_power_deg"),
                             ("ramp_angle_coast", "ramp_coast_deg")):
                if info.get(src) is not None:
                    d[key] = float(info[src])
        elif "GearedLsd" in act:
            d["type"] = "lsd_torsen"
        elif "ViscousLsd" in act:
            d["type"] = "lsd_viscous"
        elif "Spool" in act:
            d["type"] = "spool"
        if "GearedLsd" in act and info.get("bias_power") and info.get("bias_coast"):
            bp, bc = info["bias_power"], info["bias_coast"]
            d["lock_power"] = round((bp - 1.0) / (bp + 1.0), 4)
            d["lock_coast"] = round((bc - 1.0) / (bc + 1.0), 4)
            x["bias_ratio_power"], x["bias_ratio_coast"] = round(bp, 3), round(bc, 3)
        if "ClutchLsd" in act and info.get("clutches"):
            x["clutch_plates"] = info["clutches"]
        if "ViscousLsd" in act and info.get("viscous_lock") is not None:
            x["viscous_lock_setting"] = info["viscous_lock"]
        d["x_pcars2"] = x
        note = "Type and lock from the chassis file (Diff*Option settings and ranges)."
        if len(act) > 1 and act != ["Ratcheting"]:
            d["_est"] = True
            note += (f" Several types are on ({', '.join(act)}); the type chosen follows an assumed "
                     "priority of clutch over geared over viscous.")
        if "Ratcheting" in act and len(act) == 1:
            note += " Only a ratcheting differential is on; type left unchanged."
        d["_note"] = note


def mesh_id(car: str) -> str:
    """
    The assets.meshes id for a car. The schema requires ^[a-z0-9_]+$, and car
    names like 'porsche_935-78' contain hyphens. Every visual.mesh_ref must
    use this same id or the binding doesn't resolve.
    """
    return re.sub(r"[^a-z0-9_]", "_", car.lower())


def list_cars(extracted_root: str) -> List[str]:
    """Return sorted list of car names for which an edfbin exists."""
    dirs = _physics_dirs(extracted_root)
    eng = dirs.get("engines")
    if not eng:
        return []
    return sorted(
        Path(f).stem for f in Path(eng).glob("*.edfbin")
    )


def _car_file(dirs: Dict[str, Optional[str]], subdir: str,
              car: str, ext: str) -> Optional[str]:
    """
    Locate <subdir>/<car>.<ext> from the physics dirs dict.

    When the exact file is missing, falls back only to a strict base variant
    of the same car (see pcars2_names.rank_candidates(strict=True)), e.g.
    'ford_fusion_nascar13_daytona' -> 'ford_fusion_nascar13'. The previous
    prefix match took the alphabetically first sibling, which paired cars
    like 'aston_martin_db11' with the 1959 DBR1's suspension or 'ford_gtlm'
    with the Bronco's tyres; a missing file now falls back to defaults.
    """
    own = dirs.get("car_physics")
    if own:
        od = Path(own) / car / subdir
        files = sorted(od.glob(f"*.{ext}")) if od.is_dir() else []
        if files:
            return str(next((f for f in files if f.stem.lower() == car.lower()), files[0]))
    d = dirs.get(subdir)
    if not d:
        return None
    p = Path(d) / f"{car}.{ext}"
    if p.is_file():
        return str(p)
    ranked = rank_candidates(car, Path(d).glob(f"*.{ext}"), strict=True)
    if ranked:
        chosen = str(ranked[0])
        print(f"  [INFO] {subdir}/{car}.{ext} not found; "
              f"using base variant {ranked[0].name}")
        return chosen
    return None


def _wheel_block_offset(data: bytes) -> Optional[int]:
    """
    Offset of the four wheel-centre triples (FL, FR, RL, RR; game frame: X left, Y up,
    Z rear) in a .vdfm. Project CARS 2 puts them at 136; Project CARS has a shorter data
    section, so they sit elsewhere (92 on the Ariel Atom 3) with the same layout after
    them. Found by pattern: left and right mirror each other in X (within 15 cm), the front pair is
    ahead (Z < 0) and the rear pair behind (Z > 0), and the centres are 5-100 cm high.
    """
    def ok(o: int) -> bool:
        try:
            fl, fr, rl, rr = (struct.unpack_from("<3f", data, o + 12 * i) for i in range(4))
        except struct.error:
            return False
        return (all(math.isfinite(v) for t in (fl, fr, rl, rr) for v in t)
                and 0.3 < fl[0] < 1.5 and 0.3 < rl[0] < 1.5
                and abs(fl[0] + fr[0]) < 0.15 and abs(rl[0] + rr[0]) < 0.15     # the Lotus 38 is 7 cm off-centre
                and abs(fl[2] - fr[2]) < 0.02 and abs(rl[2] - rr[2]) < 0.02
                and fl[2] < -0.3 and rl[2] > 0.3
                and 0.05 < fl[1] < 1.0 and 0.05 < rl[1] < 1.0)
    if ok(136):
        return 136
    return next((o for o in range(0, len(data) - 72, 4) if ok(o)), None)


def _parse_vdfm(dirs: Dict[str, Optional[str]], car: str) -> Optional[Dict]:
    """
    Parse the vehicle's .vdfm file and return wheel-centre positions (SAE J670)
    plus tire section_width and free_radius per axle.

    vdfm offsets (little-endian f32):
      136-183 : 4 × XYZ triples in VHF coords (X=left, Y=up, Z=rear,
                origin on the ground plane), order: FL, FR, RL, RR
      184-215 : 8 floats → (front_section_width, front_OD, ×2, rear_section_width, rear_OD, ×2)

    VHF → SVJ SAE J670 (X=fwd, Y=right, Z=down, origin at the front-axle
    midpoint on the ground — SVJ_Spec.md "Origin"):
      SAE_X = -VHF_Z - front_axle_x,  SAE_Y = -VHF_X,  SAE_Z = -VHF_Y
    so front wheels sit at X=0, rear wheels at X=-wheelbase, and wheel
    centres have negative Z (above ground).
    """
    veh_dir = dirs.get("vehicles")
    if not veh_dir:
        return _parse_geometry_json(dirs, car) if dirs.get("game") == "pc1" else None
    # Resolve car name → vdfm stem (may differ from car name)
    stem = _VDFM_ALIASES.get(car, car)
    p = Path(veh_dir) / f"{stem}.vdfm"
    if not p.is_file():
        ranked = rank_candidates(stem, Path(veh_dir).glob("*.vdfm"), strict=True)
        if not ranked:
            return _parse_geometry_json(dirs, car) if dirs.get("game") == "pc1" else None
        p = ranked[0]

    try:
        data = p.read_bytes()
        w = _wheel_block_offset(data)
        if w is None or len(data) < w + 72:
            return _parse_geometry_json(dirs, car) if dirs.get("game") == "pc1" else None
        fl_v = struct.unpack_from("<3f", data, w)
        fr_v = struct.unpack_from("<3f", data, w + 12)
        rl_v = struct.unpack_from("<3f", data, w + 24)
        rr_v = struct.unpack_from("<3f", data, w + 36)
        twf  = struct.unpack_from("<f",  data, w + 48)[0]
        odf  = struct.unpack_from("<f",  data, w + 52)[0]
        twr  = struct.unpack_from("<f",  data, w + 64)[0]
        odr  = struct.unpack_from("<f",  data, w + 68)[0]

        front_axle_x = -(fl_v[2] + fr_v[2]) / 2

        def _v2sae(v: Tuple[float, float, float]) -> List[float]:
            return [round(-v[2] - front_axle_x, 4), round(-v[0], 4), round(-v[1], 4)]

        corners = {"FL": _v2sae(fl_v), "FR": _v2sae(fr_v),
                   "RL": _v2sae(rl_v), "RR": _v2sae(rr_v)}
        rear_axle_x = (corners["RL"][0] + corners["RR"][0]) / 2

        return {
            **corners,
            "wheelbase":   round(-rear_axle_x, 4),
            "track_front": round(abs(corners["FR"][1] - corners["FL"][1]), 4),
            "track_rear":  round(abs(corners["RR"][1] - corners["RL"][1]), 4),
            "_vdfm_file":  p.name,
            "_front_axle_x": front_axle_x,   # raw -> SVJ X offset, for other game-frame points
            "tire_front": {
                "section_width_m": round(twf, 4),
                "free_radius_m":   round(odf / 2, 4),
            },
            "tire_rear": {
                "section_width_m": round(twr, 4),
                "free_radius_m":   round(odr / 2, 4),
            },
        }
    except Exception as exc:
        print(f"  [WARN] vdfm parse error for {car}: {exc}")
        return None


def _parse_geometry_json(dirs: Dict[str, Optional[str]], car: str) -> Optional[Dict]:
    """
    Wheel centres and tyre size of a Project CARS car, which has no .vdfm.

    extract_car_physics writes <car_physics>/<car>/geometry.json from the car's
    .vhf (wheel node positions, game frame: X left, Y up, Z rear, origin on the
    ground) and its tyre meshes (width along X, diameter along Y). Same result
    as _parse_vdfm.
    """
    own = dirs.get("car_physics")
    p = Path(own) / car / "geometry.json" if own else None
    if not p or not p.is_file():
        return None
    try:
        g = json.loads(p.read_text(encoding="utf-8"))
        wc = g["wheel_centres"]
        fl_v, fr_v, rl_v, rr_v = (wc[k] for k in ("FL", "FR", "RL", "RR"))
        front_axle_x = -(fl_v[2] + fr_v[2]) / 2

        def _v2sae(v: List[float]) -> List[float]:
            return [round(-v[2] - front_axle_x, 4), round(-v[0], 4), round(-v[1], 4)]

        corners = {"FL": _v2sae(fl_v), "FR": _v2sae(fr_v), "RL": _v2sae(rl_v), "RR": _v2sae(rr_v)}
        rear_axle_x = (corners["RL"][0] + corners["RR"][0]) / 2
        tf, tr = g["tyre_front"], g["tyre_rear"]
        return {
            **corners,
            "wheelbase":   round(-rear_axle_x, 4),
            "track_front": round(abs(corners["FR"][1] - corners["FL"][1]), 4),
            "track_rear":  round(abs(corners["RR"][1] - corners["RL"][1]), 4),
            "_vdfm_file":  "geometry.json (car .vhf and tyre meshes)",
            "_front_axle_x": front_axle_x,
            "tire_front": {"section_width_m": round(tf["width"], 4), "free_radius_m": round(tf["diameter"] / 2, 4)},
            "tire_rear":  {"section_width_m": round(tr["width"], 4), "free_radius_m": round(tr["diameter"] / 2, 4)},
        }
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"  [WARN] geometry.json not usable for {car}: {exc}")
        return None


def _mrdf_from_chassis(chassis: Dict[str, Any], diffs: Dict[str, Any]) -> Dict[str, Any]:
    """
    Stand-in for the spec sheet of a car that has none (a few Project CARS cars).

    mass: the chassis file's Mass, which is all mass except fuel with the driver.
    drivetrain: the differentials that are on (front only FWD, rear only RWD,
    front and rear or a centre one AWD; 203 of 212 right on Project CARS 2 cars,
    whose spec sheets give the true value).
    rear weight fraction: the chassis file's CGRearRange (agrees with the spec sheet
    to 1 % on 148 of 208 Project CARS 2 cars).
    """
    out: Dict[str, Any] = {"_mrdf_file": "cdfbin (no spec sheet)", "_from_chassis": True}
    if chassis.get("mass_with_driver_kg"):
        out["mass_kg"] = chassis["mass_with_driver_kg"]
    if chassis.get("cg_rear_fraction") is not None:
        out["rear_weight_fraction"] = chassis["cg_rear_fraction"]
    front = bool((diffs.get("front") or {}).get("active"))
    rear = bool((diffs.get("rear") or {}).get("active"))
    centre = bool((diffs.get("center") or {}).get("active"))
    if (front and rear) or centre:
        out["drive_type"] = "AWD"
    elif front:
        out["drive_type"] = "FWD"
    elif rear:
        out["drive_type"] = "RWD"
    return out


_MRDF_DRIVE = {0: "RWD", 1: "AWD", 2: "FWD"}
# Engine type of the spec sheet (offset 100), named as SVJ names a configuration
_MRDF_ENGINE = {1: "V6", 2: "V8", 3: "V10", 4: "V12", 5: "I4", 6: "I5", 7: "I6", 8: "rotary2",
                9: "rotary3", 10: "flat4", 11: "flat6", 12: "W16", 13: "W12", 14: "single", 15: "twin"}


def _parse_mrdf(dirs: Dict[str, Optional[str]], car: str) -> Optional[Dict]:
    """
    Parse the car's statistics .mrdf — the spec sheet the game shows on its
    car-select screen. Fixed 256-byte layout (little-endian):

      off  60  f32  mass, kg            (A45 1555, P1 1490, Aventador 1575 —
                                         exact published values)
      off  76  u32  drivetrain          0=RWD 1=AWD 2=FWD (checked on 19 cars)
      off 136  f32  rear weight fraction (FWD 0.34-0.41, mid-engine ~0.59,
                                         991 GT3 RS 0.62)
      off 160  f32  displacement, L     (GT86 1.998, P1 3.799, R18 4.0)

    Also present but unused here: 32 top speed m/s, 68 torque lb-ft,
    72 power hp, 132 wheelbase m. Returns None if the file is absent or
    malformed; individual fields are dropped when out of range.
    """
    sdir = dirs.get("statistics")
    if not sdir:
        return None
    stem = _VDFM_ALIASES.get(car, car)
    p = Path(sdir) / f"{stem}.mrdf"
    if not p.is_file():
        p = Path(sdir) / f"{car}.mrdf"
    if not p.is_file():
        ranked = rank_candidates(car, Path(sdir).glob("*.mrdf"), strict=True)
        if not ranked:
            return None
        p = ranked[0]
    data = p.read_bytes()
    if len(data) != 256:
        return None

    f32 = lambda o: struct.unpack_from("<f", data, o)[0]
    u32 = lambda o: struct.unpack_from("<I", data, o)[0]
    out: Dict[str, Any] = {"_mrdf_file": p.name}
    mass = f32(60)
    if 200.0 <= mass <= 4000.0:
        out["mass_kg"] = round(mass, 1)
    if u32(76) in _MRDF_DRIVE:
        out["drive_type"] = _MRDF_DRIVE[u32(76)]
    frac = f32(136)
    if 0.2 <= frac <= 0.8:
        out["rear_weight_fraction"] = round(frac, 4)
    disp = f32(160)
    if 0.3 <= disp <= 12.0:
        out["displacement_l"] = round(disp, 3)
    # Fields documented in the AMS2 docs' "statistics mrdf" translation (JDougNY, GvsE)
    top, acc = f32(32), f32(36)
    if 20.0 <= top <= 140.0:
        out["top_speed_ms"] = round(top, 2)                 # metres per second
    if 1.0 <= acc <= 30.0:
        out["accel_0_100kph_s"] = round(acc, 2)
    tq, hp = f32(68), f32(72)
    if 20.0 <= tq <= 2500.0:
        out["torque_Nm"] = round(tq * 1.355818, 1)          # the sheet gives lb-ft
    if 10.0 <= hp <= 2500.0:
        out["power_W"] = round(hp * 745.7, 0)               # SAE net horsepower
    if 1 <= u32(64) <= 10:
        out["gear_count"] = u32(64)
    if u32(80) in (0, 1, 2):
        out["boost_type"] = {0: "natural", 1: "supercharged", 2: "turbo"}[u32(80)]
    if u32(100) in _MRDF_ENGINE:
        out["engine_configuration"] = _MRDF_ENGINE[u32(100)]
    if u32(164) in (0, 1):
        out["shift_type"] = "manual" if u32(164) == 0 else "sequential"   # H pattern / sequential
    return out


# Byte markers inside the sdfbin suspension block (raw primary block; the ShCB
# record parser can't be used because it splits record bodies at any 0x24).
_LINK_HEAD = bytes.fromhex("8a1e5909")      # link element; next byte = corner index
_PT_IN     = bytes.fromhex("4209feeaa302")  # followed by inboard (chassis) xyz f32
_PT_OUT    = bytes.fromhex("5a44c7e0a302")  # followed by outboard (upright) xyz f32
_AXLE_REF  = bytes.fromhex("42444209feea2302")  # spring mount: <x f32> 00 <z f32>

Vec3 = Tuple[float, float, float]


def parse_sdf_links(sf_sdf: Optional[ShCBFile]) -> Dict[str, Any]:
    """
    Read the suspension link geometry stored in an sdfbin.

    Layout (reverse-engineered; consistent across all 145 sdfbin files):
      link   = 8a1e5909 <corner byte> ... 4209feea a302 <inboard xyz>
               5a44c7e0 a302 <outboard xyz>
      single = ... 5a44c7e0 a302 <xyz> with no inboard point; above the
               wheel centre these are strut top mounts (MacPherson).
      axle   = 42444209 feea2302 <x f32> 00 <z f32>, one per spring mount;
               z is that axle's fore/aft position in the link frame.
    Coordinates are the game's vehicle frame (X left, Y up, Z rear), with Y
    relative to the wheel centre. Fore/aft, the links are authored around
    the sdfbin's own axle positions, which can differ from the vdfm wheel
    centres by up to ~0.5 m (Caterham, Radical); the caller shifts each
    axle by (vdfm wheel z - axle z). Both points were checked against the
    game's suspension meshes.

    Returns {"links": [(corner_byte, inboard, outboard)], "tops": [xyz],
             "axle_z": {"F": z, "R": z}} (axle_z entries may be missing).
    """
    if sf_sdf is None:
        return {"links": [], "tops": [], "axle_z": {}}
    b = sf_sdf._buf[sf_sdf.header.hdr_size: sf_sdf.header.sec_offset]

    def xyz(o: int) -> Vec3:
        return struct.unpack_from("<3f", b, o)

    links, paired_outs, tops = [], set(), []
    o = b.find(_PT_IN)
    while o != -1:
        head = o - 5
        if (head >= 0 and b[head:head + 4] == _LINK_HEAD
                and b[o + 18:o + 24] == _PT_OUT and o + 36 <= len(b)):
            links.append((b[o - 1], xyz(o + 6), xyz(o + 24)))
            paired_outs.add(o + 18)
        o = b.find(_PT_IN, o + 1)

    o = b.find(_PT_OUT)
    while o != -1:
        if o not in paired_outs and o + 18 <= len(b):
            p = xyz(o + 6)
            if all(math.isfinite(v) and abs(v) < 4.0 for v in p):
                tops.append(p)
        o = b.find(_PT_OUT, o + 1)

    zs = []
    o = b.find(_AXLE_REF)
    while o != -1 and o + 17 <= len(b):
        z = struct.unpack_from("<f", b, o + 13)[0]
        if math.isfinite(z) and abs(z) < 4.0:
            zs.append(z)
        o = b.find(_AXLE_REF, o + 1)
    axle_z = {}
    if any(z < 0 for z in zs):
        axle_z["F"] = min(zs)
    if any(z > 0 for z in zs):
        axle_z["R"] = max(zs)
    return {"links": links, "tops": tops, "axle_z": axle_z}


_MASS_TAG = bytes.fromhex("670b57ab21")      # Mass (int, kg) of an unsprung body
_INERTIA_TAG = bytes.fromhex("bbb39f0ba302")  # Inertia: three f32, kg.m^2
_STATION_TAG = bytes.fromhex("feea2302")      # <x f32> 00 <z f32>: where the body sits


def _parse_sdf_masses(sf_sdf: Optional[ShCBFile]) -> Dict[str, Dict[str, Any]]:
    """
    Unsprung masses and inertias per axle from the sdfbin.

    Hash 670b57ab is `Mass` and bbb39f0b is `Inertia` in the game's field
    vocabulary (pcars2_hashnames.json). The file lists, for each wheel, two
    bodies: the wheel and tyre (the larger first inertia, spun about the
    first axis) and the rest of the non-rotating unsprung assembly (hub,
    upright, brake). Each is followed by its station (x, z); z < 0 is the front
    axle. Returns {"front"|"rear": {"wheel": {mass, inertia}, "upright": {...}}}
    or {} when the layout is not recognised (about 20 cars, whose files lack
    the station records).
    """
    if sf_sdf is None:
        return {}
    b = sf_sdf._buf[sf_sdf.header.hdr_size: sf_sdf.header.sec_offset]
    events = []
    o = b.find(_MASS_TAG)
    while o != -1:
        if o + 9 <= len(b):
            mass = struct.unpack_from("<f", b, o + 5)[0]
            t = b.find(_INERTIA_TAG, o, o + 40)
            if t >= 0 and t + 18 <= len(b):
                events.append((o, 0, mass, struct.unpack_from("<3f", b, t + 6)))
        o = b.find(_MASS_TAG, o + 1)
    o = b.find(_STATION_TAG)
    while o != -1:
        if o + 13 <= len(b) and b[o + 8] == 0:
            events.append((o, 1, struct.unpack_from("<f", b, o + 4)[0],
                           struct.unpack_from("<f", b, o + 9)[0]))
        o = b.find(_STATION_TAG, o + 1)
    events.sort(key=lambda e: e[0])

    axles: Dict[str, List[Tuple[float, Tuple[float, ...]]]] = {"front": [], "rear": []}
    pending = None
    for _, kind, a, c in events:
        if kind == 0:
            pending = (a, c)
        elif pending is not None:
            if math.isfinite(c) and abs(c) > 0.2:
                axles["front" if c < 0 else "rear"].append(pending)
            pending = None
    out: Dict[str, Dict[str, Any]] = {}
    for axle, els in axles.items():
        els = [e for e in els if all(math.isfinite(v) for v in (e[0], *e[1]))
               and 1.0 <= e[0] < 200.0 and e[1][0] > 0]
        if not els:
            return {}
        wheel = max(els, key=lambda e: e[1][0])
        rest = [e for e in els if e is not wheel]
        entry: Dict[str, Any] = {"wheel": {"mass": wheel[0], "inertia": [round(v, 4) for v in wheel[1]]}}
        if rest:
            up = rest[0]
            entry["upright"] = {"mass": up[0], "inertia": [round(v, 4) for v in up[1]]}
        out[axle] = entry
    return out


def _safe_shcb(path: Optional[str], label: str) -> Optional[ShCBFile]:
    """Open a ShCBFile, printing a warning and returning None on failure."""
    if not path:
        print(f"  [WARN] {label}: file not found, section will use defaults")
        return None
    try:
        return ShCBFile(path)
    except Exception as exc:
        print(f"  [WARN] {label}: parse error - {exc}")
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Pacejka helpers
# ─────────────────────────────────────────────────────────────────────────────

def _magic_formula(slip, B, C, D, E):
    Bx = B * slip
    return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))


def _estimate_pacejka(mu: float, slip_peak_deg: float,
                      longitudinal: bool = False) -> Dict[str, float]:
    """Estimate B,C,D,E from peak friction and peak slip angle/ratio."""
    C = 1.65 if longitudinal else 1.9
    D = mu
    if HAS_SCIPY:
        slip_rad = (np.deg2rad(slip_peak_deg)
                    if not longitudinal else slip_peak_deg)
        B = float(np.tan(np.pi / (2 * C)) / max(slip_rad, 1e-6))
    else:
        B = 10.0
    E = -1.0
    return {"B": round(B, 4), "C": round(C, 4),
            "D": round(D, 4), "E": round(E, 4)}


# ─────────────────────────────────────────────────────────────────────────────
# Section builders
# ─────────────────────────────────────────────────────────────────────────────

def build_engine(sf_edf: Optional[ShCBFile], car: str) -> Dict:
    """
    Build SVJ powertrain.engine from an edfbin ShCBFile.

    Torque curve:
      pCARS2 stores (rpm, internal_friction_f32, gross_torque_f32) per curve
      point.  'gross_torque' tracks real-world ICE net torque closely (validated
      against NSX 2017, McLaren P1, Aston Martin Vulcan, Ferrari LaFerrari).
      Boundary sentinel points (RPM < 500 or gross <= 0) are stripped.
    """
    if sf_edf is None:
        # displacement/configuration are omitted rather than null: the schema
        # types them as number/string, and pCARS2 doesn't encode them.
        return {
            "idle_rpm": 800.0,
            "max_rpm": 8000.0,
            "torque_curve": [],
            "_note": "edfbin not found — defaults only",
        }

    sc    = sf_edf.get_engine_scalars()
    curve = sf_edf.get_engine_torque_curve()

    idle_rpm = float(sc.get("idle_rpm_min", 800))
    rev_lim  = float(sc.get("rev_limiter_rpm") or 0)

    # Filter curve: discard sentinels (RPM < idle or gross <= 0)
    # and discard points beyond 110 % of rev_limit (if known)
    rpm_max = rev_lim * 1.1 if rev_lim > 0 else 12000
    pts: List[List[float]] = []
    for rpm, fric, gross in curve:
        if rpm < max(idle_rpm * 0.8, 400):
            continue
        if gross <= 0:
            continue
        if rpm > rpm_max:
            continue
        pts.append([float(rpm), round(gross, 2)])

    # If rev_limit not found from scalar, estimate from the curve:
    # use the last RPM at which torque >= 50% of peak (avoids the high-RPM
    # sentinel values that appear in some pCARS2 engine files).
    if rev_lim <= 0 and pts:
        peak_t = max(t for _, t in pts)
        half_t = peak_t * 0.50
        usable = [rpm for rpm, t in pts if t >= half_t]
        rev_lim = usable[-1] if usable else pts[-1][0]

    # max_rpm: prefer the rev-limiter scalar; fall back to heuristic
    max_rpm = round(rev_lim if rev_lim > 0 else (pts[-1][0] if pts else 8000), 0)

    return {
        "idle_rpm":    round(idle_rpm, 0),
        "max_rpm":     max_rpm,
        "torque_curve": pts,
        "_note": (
            "Torque values are gross ICE torque (N·m) — matches flywheel output "
            "for ICE-only components. Hybrid electric contribution not included. "
            "displacement/configuration not encoded in pCARS2 physics files."
        ),
    }


def build_gearbox(sf_gdf: Optional[ShCBFile], car: str) -> Dict:
    """
    Build SVJ powertrain.gearbox from gdfbin.

    Returns a dict with 'ratios', 'final_drive', and '_note'.
    The caller extracts 'final_drive' for the differentials list.

    Encoding (reverse-engineered from GT86 / FR3.5 / NSX ground truth):
      Each record body is 2 bytes: [A, B]
      Gear ratio  = B / A

    File structure:
      - The dominant hash (most records) holds all ratio entries.
      - Entries are NOT guaranteed to be in gear order in the file.
      - Entries where the same B value (denominator) repeats across 2+ records
        encode a parameter *range* (min/default/max), not individual gears.
        These are interpreted as the FINAL DRIVE range; the middle value is used.
      - All remaining entries are forward gear ratios, sorted descending.
      - The special [1, 1] entry (ratio = 1.000) is a 1:1 gear (often 5th or 6th).

    Validation:
      Toyota GT86 (Aisin 6-spd): all 6 gear ratios match to <0.05% error;
        final drive 4.100 matches exactly.
      Formula Renault 3.5 (Sadev 6-spd seq): all 6 gear ratios within 0.5%
        of published range.
    """
    if sf_gdf is None:
        return {
            "type": "manual",
            "ratios":      [3.583, 2.022, 1.384, 1.000, 0.861, 0.697],
            "final_drive": 3.600,
            "_note": "gdfbin not found — 6-speed defaults",
        }

    # ── Raw scan of primary block for type-0x02 gear records ────────────────
    # The v1 gdfbin has a 3-byte block preamble before the first 0x24 tag;
    # this fools the ShCBFile parser into creating a ghost record that swallows
    # the first real gear pair.  We scan raw bytes directly instead.
    #
    # Record format:  0x24 + 4-byte_hash + 0x02 (type) + A_byte + B_byte
    # Gear ratio = B / A  (validated: GT86 <0.05%, FR3.5 <0.5%).

    hdr = sf_gdf.header
    raw = sf_gdf._buf[hdr.hdr_size : hdr.sec_offset]

    pairs_by_hash: Dict[str, list] = {}
    off = 0
    while off < len(raw) - 7:
        if raw[off] == 0x24 and raw[off + 5] == 0x02:
            h4 = raw[off + 1 : off + 5].hex()
            a, b = raw[off + 6], raw[off + 7]
            pairs_by_hash.setdefault(h4, []).append((a, b))
            off += 8
        else:
            off += 1

    if not pairs_by_hash:
        return {
            "type": "manual",
            "ratios":      [3.583, 2.022, 1.384, 1.000, 0.861, 0.697],
            "final_drive": 3.600,
            "_note": "gdfbin no records found — 6-speed defaults",
        }

    # ── Use only the dominant hash (most records = main gear table) ──────────
    # The secondary hash 3c5bef62 always carries a single [1,1] metadata marker
    # and is excluded.  Pairs (A=1, B!=1) are type/count markers, not ratios.
    dom_hash  = max(pairs_by_hash, key=lambda h: len(pairs_by_hash[h]))
    dom_pairs = pairs_by_hash[dom_hash]
    valid     = [(a, b) for a, b in dom_pairs if not (a == 1 and b != 1)]
    if not valid:
        valid = dom_pairs

    # ── Step 1: Detect and separate final-drive from gear entries ────────────
    # Operate on the original (A, B) pairs so FD detection retains full context.
    # Three strategies (tried in order):
    #
    # 1. Repeated-B + consecutive-A: genuine FD ranges share the same B value
    #    AND their A values form a consecutive integer sequence (e.g. [9,10,11]).
    #    Coincidental B repeats (e.g. LaFerrari [45,31] + [24,31]) are excluded
    #    because A values are far apart.
    #
    # 2. Consecutive-A at top: the two highest-ratio pairs have |A₁−A₂|=1
    #    AND both B/A > 2.0 → 2-step FD range; use the lower-A entry as default.
    #
    # 3. Single top outlier: highest B/A >= 1.20× the second → lone FD entry.

    b_to_a: dict = defaultdict(list)
    for a, b in valid:
        if not (a == 1 and b == 1):
            b_to_a[b].append(a)

    def _is_consecutive(vals: list) -> bool:
        s = sorted(vals)
        return all(s[i+1] - s[i] == 1 for i in range(len(s) - 1))

    rep_b = {b for b, a_list in b_to_a.items()
             if len(a_list) >= 2 and _is_consecutive(a_list)}

    fd_pairs:   list = []
    gear_pairs: list = []

    if rep_b:                                            # Strategy 1
        fd_pairs   = [(a, b) for a, b in valid if b in rep_b]
        gear_pairs = [(a, b) for a, b in valid if b not in rep_b]
    else:
        by_ratio = sorted(valid, key=lambda x: x[1] / max(x[0], 1), reverse=True)
        if len(by_ratio) >= 2:
            (a0, b0), (a1, b1) = by_ratio[0], by_ratio[1]
            r0, r1 = b0 / max(a0, 1), b1 / max(a1, 1)
            if abs(a0 - a1) == 1 and r1 > 2.0:          # Strategy 2
                fd_pairs   = [by_ratio[0], by_ratio[1]]
                gear_pairs = by_ratio[2:]
            elif r0 >= r1 * 1.20:                        # Strategy 3
                fd_pairs   = [by_ratio[0]]
                gear_pairs = by_ratio[1:]
            else:
                gear_pairs = by_ratio
        else:
            gear_pairs = by_ratio

    fd_vals = sorted([b / max(a, 1) for a, b in fd_pairs])
    if fd_vals:
        final_drive = round(fd_vals[len(fd_vals) // 2], 5)
    else:
        final_drive = 3.600

    # ── Step 2: Cluster nearby gear ratios (selectable-ratio race cars) ───────
    # Many race cars store ALL available gear-ratio options (one entry per
    # tooth-pair combination, e.g. 30+ options for a single gear position).
    # Adjacent ratios that differ by < 13% belong to the same gear position;
    # we keep the median value of each cluster as the representative.
    # Simple road cars have large inter-gear gaps (30–70%) so each ratio forms
    # its own cluster of 1, and the behaviour is identical to before.

    gear_raw = sorted(
        [b / max(a, 1) for a, b in gear_pairs if 0.25 <= b / max(a, 1) <= 6.0],
        reverse=True
    )

    clusters: List[List[float]] = []
    for r in gear_raw:
        # Same gear position if within 8 % of the cluster's latest entry.
        # Selectable-ratio options are typically < 5 % apart; real consecutive
        # gears differ by >= 13 %, so the boundary at 8 % cleanly separates them.
        if clusters and r > clusters[-1][-1] * 0.92:
            clusters[-1].append(r)
        else:
            clusters.append([r])

    gear_ratios = sorted(
        [round(c[len(c) // 2], 5) for c in clusters],
        reverse=True
    )
    if not gear_ratios:
        gear_ratios = [3.583, 2.022, 1.384, 1.000, 0.861, 0.697]
        final_drive = 3.600

    n_gears = len(gear_ratios)
    return {
        "type":        "manual",
        "ratios":      gear_ratios,
        "final_drive": final_drive,    # extracted by convert_car() into differentials
        "_note": (
            f"{n_gears}-speed gearbox decoded from gdfbin "
            f"(ratio = body[1]/body[0], validated on GT86/FR3.5/NSX)."
        ),
    }


_DEFAULT_MASS = 1200.0


def build_chassis(car: str, vdfm: Optional[Dict] = None,
                  mrdf: Optional[Dict] = None,
                  cdf: Optional[Dict] = None) -> Dict:
    """
    Build SVJ chassis.

    mass_total: from the statistics .mrdf (the game's spec-sheet weight).
      The old approach — the most common plausible f32 in the cdfbin — mostly
      found shared constants (212 cars collapsed onto 24 values; F40 and
      Radical SR3 both came out at 2596 kg), so it was dropped. No mrdf means
      a flagged default.
    center_of_gravity: X from the mrdf rear weight fraction and the vdfm
      wheelbase (CG sits rear_fraction * wheelbase behind the front axle);
      Y on the centreline; Z (height) is not stored, so it stays 0.
    wheelbase / track: from the vdfm wheel centres.
    """
    mass = (mrdf or {}).get("mass_kg")
    mass_source = ((mrdf or {}).get("_from_chassis") and "cdfbin Mass (all mass except fuel, driver included)"
                   or "pcars2_mrdf") if mass else "default"
    if not mass:
        mass = _DEFAULT_MASS
        print(f"  [WARN] {car}: no mrdf mass; using default {_DEFAULT_MASS:.0f} kg")

    cg = [0.0, 0.0, 0.0]
    frac = (mrdf or {}).get("rear_weight_fraction") or (cdf or {}).get("cg_rear_fraction")
    if frac and vdfm:
        cg[0] = round(-frac * vdfm["wheelbase"], 4)

    cg_note = ""
    cgh = (cdf or {}).get("cg_height_m")
    rides = [c["ride_height_m"] for c in (cdf or {}).get("corners", []) if c.get("ride_height_m")]
    if cgh and rides:
        # CGHeight is the body mass height above a reference plane that the file does not name.
        # Compared with Assetto Corsa's CG height for 52 cars matched by name and mass, adding
        # the ride height (plane = underside of the body at ride height) leaves a -2 cm mean
        # error (5 cm scatter); CGHeight alone is 12 cm low.
        cg[2] = round(-(cgh + sum(rides) / len(rides)), 4)
        cg_note = (f" Z: CGHeight {cgh:.3f} m (body mass, excluding fuel) plus the mean ride height "
                   f"{sum(rides) / len(rides):.3f} m; the reference plane is assumed to be the body "
                   "underside at ride height. Over 52 cars matched by name and mass to Assetto Corsa "
                   "cars, this reading is 2 cm below their CG height on average (scatter 5 cm); "
                   "CGHeight alone is 12 cm below.")

    chassis: Dict[str, Any] = {
        "mass_total":        mass,
        "center_of_gravity": cg,
        # No visual binding here: it must name a node that exists in the GLB,
        # so pcars2_extract_meshes adds bindings when it builds the model.
        "_mass_source": mass_source,
        "_note": (
            "mass_total: game spec-sheet weight from statistics .mrdf (default if absent). "
            "center_of_gravity: X from mrdf rear weight fraction x vdfm wheelbase "
            "(0 if either is missing); Y centreline; Z height from the chassis file when "
            "present (0 otherwise). wheelbase/track: vdfm wheel centres." + cg_note
        ),
    }
    ig = (cdf or {}).get("inertia_game_axes")
    if ig:
        # The game's axes: X lateral, Y up, Z longitudinal. Inertia about X is
        # pitch, about Y yaw, about Z roll.
        chassis["inertia"] = {"Ixx": ig[2], "Iyy": ig[0], "Izz": ig[1],
                              "Ixy": 0.0, "Ixz": 0.0, "Iyz": 0.0}
        chassis["_inertia_source"] = ("pcars2_cdfbin Inertia (X, Y, Z of the game's axes: "
                                      f"{ig[0]:g}, {ig[1]:g}, {ig[2]:g}); mapped to roll, pitch, yaw. "
                                      "The axis order is inferred: roll comes out smallest and "
                                      "yaw largest in every car.")
    if vdfm:
        chassis["wheelbase"]   = vdfm["wheelbase"]
        chassis["track_front"] = vdfm["track_front"]
        chassis["track_rear"]  = vdfm["track_rear"]
    return chassis


def build_aerodynamics(sf_cdf: Optional[ShCBFile], car: str) -> Dict:
    """
    Aerodynamics: Cd and Cl are flagged placeholders; the game's own aero model is
    kept as decoded under x_pcars2.

    Earlier versions read a drag area from hash 6f70f3c7 and lift from e0a125de.
    Those are FuelTankMotion and DiffuserStall (6f70f3c7 holds 560.0 and 0.7 in
    every car), so every car came out with the same Cd. The game does not store
    a drag coefficient: it computes the forward drag at run time from
    BodyDragBase and the ride heights, with a formula that is not in the files
    (BodyFore z, where it would sit, is 0 in all 209 cars). Against Assetto
    Corsa's body drag area for 52 matched cars, BodyDragBase correlates only at
    r = 0.29, so no scale is claimed.
    """
    DEFAULT_FRONTAL_AREA = 2.1   # m², typical sports car
    out: Dict[str, Any] = {
        "reference":    {"frontal_area": DEFAULT_FRONTAL_AREA},
        "coefficients": {"Cd": 0.35, "Cl": 0.0},
        "_est": True,
        "_note": ("Placeholder Cd and Cl: the game computes drag at run time and stores no drag "
                  "coefficient. Its aero model (body, wings, diffuser, radiator, brake ducts) is "
                  "kept in x_pcars2.aero_model."),
    }
    if sf_cdf is not None:
        try:
            model = _chassis_mod.parse_aero(sf_cdf)
        except Exception:                      # a malformed file must not stop the car
            model = {}
        if model:
            out["x_pcars2"] = {
                "aero_model": model,
                "_note": ("Fields of the chassis file by their documented names. Force vectors are "
                          "(lateral, vertical, longitudinal) coefficients of speed squared in the "
                          "game's units; centres are offsets from the rear axle at the reference "
                          "plane. BodyFore and RWFore store 0 because forward drag is computed at "
                          "run time from BodyDragBase and the ride heights."),
            }
    return out


def _tire_data(sf_hdt: Optional[ShCBFile], car: str, axle: str) -> Dict:
    """
    Extract tire properties from an hdtbin ShCBFile for one axle.

    Key hashes in hdtbin:
      b1033b86 : [mu_lat f32 @ 0, mu_long f32 @ 4, ...]
      b31155cd : [scale f32 @ 0, scale f32 @ 4]  (may be D coeff directly)

    Returns a dict suitable for embedding in suspension corner data.
    """
    radius   = 0.330 if axle == "front" else 0.340
    width    = 0.240 if axle == "front" else 0.270
    rim_dia  = 0.457                    # 18" default (0.4572 m)
    rim_wid  = 0.240 if axle == "front" else 0.270
    pressure = 200_000.0                # 2 bar nominal

    # ── Pacejka D (peak friction) from hdtbin hash b31155cd ──────────────────
    # b31155cd = [mu_lat f32, mu_long f32] — confirmed as peak friction μ
    # (scales with grip: GT86=1.85, Ferrari=2.2, R18=2.3, Ariel Atom=1.7)
    D_lat  = 1.85
    D_long = 1.85
    if sf_hdt is not None:
        r_d = sf_hdt.get_one("b31155cd")
        if r_d and len(r_d.body) >= 8:
            v0 = struct.unpack_from("<f", r_d.body, 0)[0]
            v1 = struct.unpack_from("<f", r_d.body, 4)[0]
            if 0.5 <= v0 <= 5.0:
                D_lat = round(v0, 4)
            if 0.5 <= v1 <= 5.0:
                D_long = round(v1, 4)

    # ── Pacejka C (shape factor) from hdtbin hash b1033b86 ───────────────────
    # b1033b86 = [C_lat f32, C_long f32] — confirmed as shape factor
    # (Ariel Atom has C_lat=1.25 ≠ C_long=1.67, confirming lat≠long encoding)
    C_lat  = 1.35
    C_long = 1.65
    if sf_hdt is not None:
        r_c = sf_hdt.get_one("b1033b86")
        if r_c and len(r_c.body) >= 8:
            v0 = struct.unpack_from("<f", r_c.body, 0)[0]
            v1 = struct.unpack_from("<f", r_c.body, 4)[0]
            if 0.5 <= v0 <= 4.0:
                C_lat = round(v0, 4)
            if 0.5 <= v1 <= 4.0:
                C_long = round(v1, 4)

    # ── Pacejka E (curvature factor) from hdtbin hash c0eaf58d ───────────────
    # c0eaf58d[0] = E (0.65 road cars, 0.85 race/slick — confirmed car-specific)
    E_lat  = 0.65
    E_long = 0.65
    if sf_hdt is not None:
        r_e = sf_hdt.get_one("c0eaf58d")
        if r_e and len(r_e.body) >= 4:
            v = struct.unpack_from("<f", r_e.body, 0)[0]
            if -3.0 <= v <= 2.0:
                E_lat = E_long = round(v, 4)

    # ── Pacejka B (stiffness factor) — estimated ─────────────────────────────
    # B is not directly stored; estimated so that B×C×D ≈ 24 (lateral) / 30 (long)
    # This gives initial cornering stiffness typical of road/race tires.
    # Mark as estimated with _est flag.
    _BCD_lat  = 24.0
    _BCD_long = 30.0
    B_lat  = round(_BCD_lat  / max(C_lat  * D_lat,  0.1), 3)
    B_long = round(_BCD_long / max(C_long * D_long, 0.1), 3)
    B_lat  = max(4.0, min(25.0, B_lat))
    B_long = max(4.0, min(30.0, B_long))

    mu_lat  = D_lat
    mu_long = D_long

    # Coefficient blocks must hold numbers only (schema); the "B is estimated"
    # flag lives one level up in _tire_model / tires.sets.*.pacejka.
    pac_lat  = {"B": B_lat,  "C": C_lat,  "D": D_lat,  "E": E_lat}
    pac_long = {"B": B_long, "C": C_long, "D": D_long, "E": E_long}

    return {
        "loaded_radius":    round(radius, 4),
        "rim_diameter":     round(rim_dia, 4),
        "rim_width":        round(rim_wid, 4),
        "mass":             18.0,
        "rotational_inertia": 0.80,
        "pressure_nominal": round(pressure, 1),
        "_tire_model": {
            "type":            "pacejka",
            "lateral":         pac_lat,
            "longitudinal":    pac_long,
            "mu_lateral":      round(mu_lat, 4),
            "mu_longitudinal": round(mu_long, 4),
            "_B_estimated":    True,
            "_B_note":         "B not stored by pCARS2; estimated from B*C*D = 24 (lat) / 30 (long).",
        },
        "_note": (
            "Geometry (rim_diameter, rim_width, loaded_radius) uses axle-specific defaults "
            "(override from vdfm in build_suspension). "
            "Pacejka D from b31155cd, C from b1033b86, E from c0eaf58d; B estimated (BCD~24/30)."
        ),
    }


def _estimated_double_wishbone(wc: List[float], axle: str
                               ) -> Tuple[Dict[str, List[float]], List[Dict]]:
    """
    Representative double-wishbone hardpoints and links around a wheel centre.

    pCARS2 physics stores only wheel centres; kinematics come from K&C lookup
    tables (sdfbin), not link geometry. SVJ requires topology.links, so this
    follows the spec's own convention for missing data (see its skeleton
    examples): plausible engineering estimates, flagged _est by the caller.
    wheel_center itself stays the real vdfm value.

    Coordinates: SVJ SAE J670 (X fwd, Y right, Z down; origin front axle,
    ground). "Up" is -Z; "inboard" is toward Y=0.
    """
    x, y, z = wc
    side = 1.0 if y >= 0 else -1.0          # +1 right, -1 left
    inward = -side

    def inboard_y(depth: float) -> float:
        return round(side * max(abs(y) - depth, 0.10), 4)

    ubj = [round(x, 4), round(y + inward * 0.10, 4), round(z - 0.17, 4)]
    lbj = [round(x, 4), round(y + inward * 0.05, 4), round(min(z + 0.13, -0.05), 4)]
    tre = [round(x - 0.12, 4), round(y + inward * 0.08, 4), round(z + 0.02, 4)]

    hardpoints = {
        "wheel_center":     [round(v, 4) for v in wc],
        "upper_ball_joint": ubj,
        "lower_ball_joint": lbj,
        "tie_rod_end":      tre,
    }
    tie_name = "steering_tie_rod" if axle == "front" else "toe_link"
    links = [
        {"name": "upper_control_arm", "type": "arm",
         "inboard_points": [[round(x + 0.12, 4), inboard_y(0.36), round(ubj[2] - 0.01, 4)],
                            [round(x - 0.12, 4), inboard_y(0.36), round(ubj[2] - 0.01, 4)]],
         "outboard_ref": "hardpoints.upper_ball_joint",
         "joint_type": "spherical", "inboard_joint_type": "revolute"},
        {"name": "lower_control_arm", "type": "arm",
         "inboard_points": [[round(x + 0.16, 4), inboard_y(0.40), lbj[2]],
                            [round(x - 0.16, 4), inboard_y(0.40), lbj[2]]],
         "outboard_ref": "hardpoints.lower_ball_joint",
         "joint_type": "spherical", "inboard_joint_type": "revolute"},
        {"name": tie_name, "type": "rod",
         "inboard_points": [[tre[0], inboard_y(0.40), tre[2]]],
         "outboard_ref": "hardpoints.tie_rod_end",
         "joint_type": "spherical", "inboard_joint_type": "spherical"},
    ]
    return hardpoints, links


_ZETA_BUMP    = 0.30   # damping ratios for the estimated dampers
_ZETA_REBOUND = 0.55
_DAMPER_KNEE  = 0.1    # m/s: assumed speed where the slow damper slope turns into the fast one
_MAX_JOINT_REACH = 0.45   # m: an upright joint farther than this from its wheel centre is rejected


def _real_topologies(sdf_links: Dict[str, Any], vdfm: Optional[Dict]
                     ) -> Dict[str, Tuple[str, Dict[str, List[float]], List[Dict], Dict]]:
    """
    Build SVJ suspension topology per corner from the sdfbin link geometry.

    Links are assigned to a corner by geometry (side from X, axle from the
    nearest sdfbin axle position), moved fore/aft onto the vdfm wheel centre
    of that axle, converted to SVJ SAE J670 (heights are stored relative to
    the wheel centre), and grouped: links that share an upright joint form
    one A-arm. Checked against the game's own suspension meshes, where the
    joints then sit on mesh vertices.

    Layout per corner:
      2 A-arms (+ link)        -> double_wishbone
      1 A-arm + link + strut   -> macpherson (strut top from the standalone point)
      3 or more links          -> multi_link
      1 A-arm (+ link), pivot axis near lateral -> trailing_arm (< 10 deg)
                                                   or semi_trailing_arm (< 60 deg)
      anything else            -> not returned (caller keeps the estimate;
                                  e.g. live axles, which SVJ models as an
                                  axle body the link records don't describe)

    Returns {station: (system_type, hardpoints, links)} for corners that pass
    the sanity checks (every upright joint within _MAX_JOINT_REACH of the wheel
    centre, both corners of an axle present).
    """
    if not vdfm or not sdf_links.get("links"):
        return {}
    fax = vdfm["_front_axle_x"]
    # Fore/aft (raw z) of each axle: as authored in the sdfbin, and in the vdfm
    wheel_z = {a: -vdfm[a + "L"][0] - fax for a in ("F", "R")}
    link_z = {a: sdf_links.get("axle_z", {}).get(a, wheel_z[a]) for a in ("F", "R")}

    def station_of(p_raw: Vec3) -> str:
        side = "L" if p_raw[0] > 0 else "R"
        axle = min(("F", "R"), key=lambda a: abs(p_raw[2] - link_z[a]))
        return axle + side

    def to_sae(p: Vec3, st: str) -> List[float]:
        z = p[2] + wheel_z[st[0]] - link_z[st[0]]
        return [round(-z - fax, 4), round(-p[0], 4), round(-(p[1] - vdfm[st][2]), 4)]

    corners: Dict[str, List[Tuple[List[float], List[float]]]] = {}
    for _, inb, outb in sdf_links["links"]:
        st = station_of(outb)
        corners.setdefault(st, []).append((to_sae(inb, st), to_sae(outb, st)))
    tops: Dict[str, List[float]] = {}
    for p in sdf_links.get("tops", []):
        st = station_of(p)
        if p[1] > 0.1 and st not in tops:          # above the wheel centre
            tops[st] = to_sae(p, st)

    out: Dict[str, Tuple[str, Dict[str, List[float]], List[Dict], Dict]] = {}
    for st, pairs in corners.items():
        wc = vdfm[st]
        if any(math.hypot(o[0] - wc[0], o[1] - wc[1]) > _MAX_JOINT_REACH for _, o in pairs):
            continue
        front = st[0] == "F"
        groups: Dict[Tuple[float, ...], List[List[float]]] = {}
        for inb, outb in pairs:
            groups.setdefault(tuple(round(v, 3) for v in outb), []).append(inb)
        arms = [(list(k), v) for k, v in groups.items() if len(v) >= 2]
        singles = [(list(k), v[0]) for k, v in groups.items() if len(v) == 1]
        n_links = len(arms) + len(singles)

        if len(arms) == 2:
            system = "double_wishbone"
        elif len(arms) == 1 and singles and st in tops and n_links <= 3:
            system = "macpherson"
        elif n_links >= 3:
            system = "multi_link"
        elif len(arms) == 1 and n_links <= 2 and len(arms[0][1]) == 2:
            a, b = arms[0][1]
            axis = math.degrees(math.atan2(abs(a[0] - b[0]), abs(a[1] - b[1])))
            if axis < 10.0:
                if n_links == 1:
                    continue   # a lone lateral-axis arm is a live axle's link; keep the estimate
                system = "trailing_arm"
            elif axis < 60.0:
                system = "semi_trailing_arm"
            else:
                continue
        else:
            continue

        if system == "macpherson":
            # A strut top that coincides with an upright joint, or an arm that
            # spans the centreline, is not a real strut: it is a beam axle
            # whose links the sdfbin describes differently. Keep the estimate.
            top = tops[st]
            if (any(math.dist(top, o) < 0.05 for o, _ in singles)
                    or any(p[1] * wc[1] < 0 for _, v in arms for p in v)):
                continue

        hp: Dict[str, List[float]] = {"wheel_center": [round(v, 4) for v in wc]}
        links: List[Dict] = []
        used: set = set()

        def add(name: str, kind: str, inboard: List[List[float]], outb: List[float],
                joint_name: str) -> None:
            base, n = name, 2
            while name in used:
                name, n = f"{base}_{n}", n + 1
            used.add(name)
            hp[joint_name if joint_name not in hp else f"{name}_outer"] = outb
            ref = joint_name if hp.get(joint_name) is outb else f"{name}_outer"
            links.append({
                "name": name, "type": kind,
                "inboard_points": inboard,
                "outboard_ref": f"hardpoints.{ref}",
                "joint_type": "spherical",
                "inboard_joint_type": "revolute" if kind == "arm" else "spherical",
            })

        # A-arms: higher upright joint (more negative Z, Z is down) is the upper one
        arms.sort(key=lambda a: a[0][2])
        if len(arms) == 2:
            add("upper_wishbone", "arm", arms[0][1], arms[0][0], "upper_ball_joint")
            add("lower_wishbone", "arm", arms[1][1], arms[1][0], "lower_ball_joint")
        elif system in ("trailing_arm", "semi_trailing_arm"):
            # Spec 9.2.1: the arm carries the hub, so it ends on wheel_center.
            links.append({"name": system, "type": "arm",
                          "inboard_points": arms[0][1],
                          "outboard_ref": "hardpoints.wheel_center",
                          "joint_type": "spherical",
                          "inboard_joint_type": "revolute"})
            used.add(system)
        elif len(arms) == 1:
            upper = arms[0][0][2] < wc[2] and system != "macpherson"
            add("upper_wishbone" if upper else "lower_wishbone", "arm", arms[0][1], arms[0][0],
                "upper_ball_joint" if upper else "lower_ball_joint")

        # Single links: the one reaching farthest fore/aft of the wheel centre
        # is the steering tie rod (front) or toe link (rear).
        dropped: List[str] = []
        if system in ("trailing_arm", "semi_trailing_arm") and singles:
            # The arm already fixes the hub; a toe link would over-constrain it.
            dropped += [f"toe-link-like rod outboard {o} inboard {i}" for o, i in singles]
            singles = []
        if singles:
            singles.sort(key=lambda s: -abs(s[0][0] - wc[0]))
            toe = singles.pop(0)
            if front:
                add("tie_rod", "rod", [toe[1]], toe[0], "tie_rod_end")
            else:
                add("toe_link", "rod", [toe[1]], toe[0], "toe_link_outer")
        if system == "macpherson" and singles:
            # arm (2) + toe (1) + strut (2) already fixes the hub; a further
            # rod would over-constrain it.
            dropped += [f"extra rod outboard {o} inboard {i}" for o, i in singles]
            singles = []
        for outb, inb in singles:
            along = abs(inb[0] - outb[0]) > abs(inb[1] - outb[1])
            if along:
                add("trailing_arm", "rod", [inb], outb, "trailing_arm_outer")
            else:
                level = "upper" if outb[2] < wc[2] else "lower"
                pos = "front" if outb[0] >= wc[0] else "rear"
                add(f"{level}_link_{pos}", "rod", [inb], outb, f"{level}_link_{pos}_outer")

        for link in links:
            if (link["name"].startswith("upper")
                    and any(p[1] * wc[1] < 0 for p in link["inboard_points"])):
                link["_note"] = ("Inboard end lies past the vehicle centreline: this is how the "
                                 "game represents a strut, as a long virtual upper link.")
        if system == "macpherson":
            top = tops[st]
            hp["strut_top"] = top
            # Strut lower end on the upright: not in the game data. Put it on
            # the line from the lower ball joint to the top mount, just above
            # the wheel centre, so the strut axis stays plausible.
            lbj = hp.get("lower_ball_joint", wc)
            dz = top[2] - lbj[2]
            t = (wc[2] - 0.04 - lbj[2]) / dz if abs(dz) > 1e-6 else 0.4
            t = min(max(t, 0.25), 0.7)
            hp["strut_lower"] = [round(lbj[i] + t * (top[i] - lbj[i]), 4) for i in range(3)]
            links.append({"name": "strut", "type": "strut", "inboard_points": [top],
                          "outboard_ref": "hardpoints.strut_lower",
                          "joint_type": "translational", "inboard_joint_type": "fixed",
                          "_note": "strut_top decoded from the sdfbin; strut_lower is an "
                                   "estimate on the line from lower_ball_joint to strut_top."})

        # A corner needs 5 constraints (arm 2, rod 1, strut 2). Where the game
        # data leaves one out, add a flagged estimate.
        def n_cons(ls: List[Dict]) -> int:
            return sum({"arm": 2, "strut": 2}.get(l["type"], 1) for l in ls)
        if n_cons(links) == 4:
            names = [l["name"] for l in links]
            uppers = [n for n in names if n.startswith("upper_link_")]
            has_toe = any(n in ("tie_rod", "toe_link") for n in names)
            if len(uppers) == 1:
                src = next(l for l in links if l["name"] == uppers[0])
                ob = hp[src["outboard_ref"].split(".", 1)[1]]
                ib = src["inboard_points"][0]
                flip = lambda q: [round(2 * wc[0] - q[0], 4), q[1], q[2]]
                nm = "upper_link_front" if uppers[0].endswith("rear") else "upper_link_rear"
                hp[f"{nm}_outer"] = flip(ob)
                links.append({"name": nm, "type": "rod", "inboard_points": [flip(ib)],
                              "outboard_ref": f"hardpoints.{nm}_outer",
                              "joint_type": "spherical", "inboard_joint_type": "spherical",
                              "_est": True,
                              "_note": f"Missing from the game data; mirrored from {uppers[0]} "
                                       "about the wheel-centre X."})
            elif not has_toe:
                side = 1.0 if wc[1] >= 0 else -1.0
                key = "tie_rod_end" if front else "toe_link_outer"
                z = round(hp.get("lower_ball_joint", wc)[2] - 0.02, 4)
                hp[key] = [round(wc[0] - 0.10, 4), round(wc[1] - side * 0.08, 4), z]
                links.append({"name": "tie_rod" if front else "toe_link", "type": "rod",
                              "inboard_points": [[round(wc[0] - 0.10, 4),
                                                  round(side * max(abs(wc[1]) - 0.40, 0.10), 4), z]],
                              "outboard_ref": f"hardpoints.{key}",
                              "joint_type": "spherical", "inboard_joint_type": "spherical",
                              "_est": True,
                              "_note": "No toe/steering link in the game data for this corner; "
                                       "estimated behind the wheel centre so the wheel is located."})
        extra = {"_dropped_links": dropped} if dropped else {}
        out[st] = (system, hp, links, extra)

    # Keep an axle only when both of its corners decoded (avoids half-real axles)
    for axle in ("F", "R"):
        if (axle + "L" in out) != (axle + "R" in out):
            out.pop(axle + "L", None); out.pop(axle + "R", None)
    return out


# Wheel-centre fallback when no vdfm is found (SVJ SAE J670, front-axle origin)
_DEFAULT_WHEELBASE = 2.60
_DEFAULT_TRACK     = 1.55
_DEFAULT_WHEEL_R   = 0.33


def build_suspension(sf_sdf: Optional[ShCBFile],
                     sf_hdt: Optional[ShCBFile],
                     car: str,
                     vdfm: Optional[Dict] = None,
                     mass_kg: Optional[float] = None,
                     rear_fraction: Optional[float] = None,
                     chassis: Optional[Dict] = None) -> Dict:
    """
    Build SVJ suspension as FL/FR/RL/RR per-corner dicts.

    Springs, dampers, static camber/toe/caster and tyre pressure: the chassis
      file (`chassis`, from pcars2_chassis.parse_chassis). Each is a range
      (start, step, count) plus the default setting; wheel rates are the
      spring and damper rates times the file's SpringMult / DamperMult.
    Unsprung masses and inertias (wheel, upright): the sdfbin (_parse_sdf_masses).
    Topology: decoded from the sdfbin link records where the layout is
      recognised (_real_topologies); otherwise a flagged double-wishbone estimate.

    Where a value is missing it is estimated and flagged `_est`.
    Tire and brake data are embedded within each corner per the SVJ schema.
    """
    sdf_masses = _parse_sdf_masses(sf_sdf)
    cc = (chassis or {}).get("corners") or []
    cc_axle = {"front": cc[0] if len(cc) >= 4 and cc[0].get("spring_Nm") else None,
               "rear":  cc[2] if len(cc) >= 4 and cc[2].get("spring_Nm") else None}
    # Where the chassis file has no rate, derive one from the corner's sprung mass
    # and a typical ride frequency (flagged), not one number for every car.
    mass = mass_kg or _DEFAULT_MASS
    rf = rear_fraction if rear_fraction else 0.5
    def _ride_rate(axle_fraction: float, freq_hz: float) -> float:
        m_corner = mass * axle_fraction / 2 * 0.88      # ~12 % unsprung
        return round(m_corner * (2 * math.pi * freq_hz) ** 2, -2)
    spring_src = {}
    rates = {}
    for axle, frac, hz in (("front", 1 - rf, 1.5), ("rear", rf, 1.7)):
        c = cc_axle[axle]
        if c:
            rates[axle] = round(c["spring_Nm"] * (c.get("spring_mult") or 1.0))
            spring_src[axle] = "pcars2_cdfbin"
        else:
            rates[axle], spring_src[axle] = _ride_rate(frac, hz), "estimated_ride_frequency"
    front_spring, rear_spring = rates["front"], rates["rear"]

    # Dampers: from the chassis file when present, else estimated from a damping ratio.
    front_mc = mass * (1 - rf) / 2 * 0.88
    rear_mc  = mass * rf / 2 * 0.88

    real_topo = _real_topologies(parse_sdf_links(sf_sdf), vdfm)

    tire_f = _tire_data(sf_hdt, car, "front")
    tire_r = _tire_data(sf_hdt, car, "rear")

    def _corner(axle: str, side: str, spring: float, m_corner: float,
                tire: Dict) -> Dict:
        tag  = axle[0].upper() + side[0].upper()   # FL, FR, RL, RR
        node = f"upright_{tag.lower()}"
        ccx = cc_axle[axle]

        # ── Wheel-centre position from vdfm (SVJ SAE J670, Z down) ───────────
        if vdfm and tag in vdfm:
            wc_sae = vdfm[tag]
            wc_source = "pcars2_vdfm"
            tire_vd = vdfm[f"tire_{axle}"]
            loaded_radius  = round(-wc_sae[2], 4)   # Z is down: centre height = -Z
            free_radius    = tire_vd["free_radius_m"]
            section_width  = tire_vd["section_width_m"]
            tire_src_note  = (
                "loaded_radius = wheel-centre height from vdfm; _free_radius and "
                "_section_width from vdfm. Pacejka D from hdtbin b31155cd, "
                "C from b1033b86, E from c0eaf58d; B estimated."
            )
        else:
            x = 0.0 if axle == "front" else -_DEFAULT_WHEELBASE
            y = (-1.0 if side == "left" else 1.0) * _DEFAULT_TRACK / 2
            wc_sae = [x, y, -_DEFAULT_WHEEL_R]
            wc_source = "estimated"
            loaded_radius = tire["loaded_radius"]
            free_radius   = None
            section_width = None
            tire_src_note = tire["_note"]

        if tag in real_topo:
            system_type, hardpoints, links, topo_notes = real_topo[tag]
            topo_extra = {
                **topo_notes,
                "_wheel_center_source": wc_source,
                "_topology_source": "pcars2_sdfbin",
                "_note": (
                    "Hardpoints and links decoded from the sdfbin link records, "
                    "placed on the vdfm wheel centre. "
                    "system_type and link names are inferred from the layout; "
                    "upright mass and joint types are estimates."
                ),
            }
        else:
            system_type = "double_wishbone"
            hardpoints, links = _estimated_double_wishbone(wc_sae, axle)
            topo_extra = {
                "_est": True,
                "_wheel_center_source": wc_source,
                "_note": (
                    "Only wheel_center comes from game data (when _wheel_center_source "
                    "is pcars2_vdfm). No link geometry was decoded for this axle "
                    "(no sdfbin of its own, or a layout the converter doesn't "
                    "recognise, such as a live axle), so system_type, the other "
                    "hardpoints and the links are representative double-wishbone "
                    "estimates."
                ),
            }

        wheel_dict: Dict[str, Any] = {
            "rim_diameter":       tire["rim_diameter"],
            "rim_width":          tire["rim_width"],
            "loaded_radius":      loaded_radius,
            "mass":               tire["mass"],
            "rotational_inertia": tire["rotational_inertia"],
            "_tire_model":        tire["_tire_model"],
            "_pressure_nominal":  tire["pressure_nominal"],
            "_note":              tire_src_note,
        }
        sm = sdf_masses.get(axle, {})
        if sm.get("wheel"):
            wheel_dict["mass"] = round(sm["wheel"]["mass"], 3)
            wheel_dict["rotational_inertia"] = sm["wheel"]["inertia"][0]
            wheel_dict["_mass_source"] = ("pcars2_sdfbin (Mass, Inertia of the wheel and tyre; "
                                          "the first inertia is the spin axis)")
        if ccx and ccx.get("pressure_kpa"):
            wheel_dict["_pressure_nominal"] = round(ccx["pressure_kpa"] * 1000.0)
        if free_radius is not None:
            wheel_dict["_free_radius"]   = free_radius
            wheel_dict["_section_width"] = section_width
            wheel_dict["_tire_source"]   = "pcars2_vdfm"

        alignment = {}
        if ccx and chassis:
            toe = chassis.get("toe_front_deg" if axle == "front" else "toe_rear_deg")
            alignment = {k: round(math.radians(v), 6) for k, v in (
                ("camber", ccx.get("camber_deg")), ("toe", toe),
                ("caster", chassis.get("caster_deg") if axle == "front" else None)) if v is not None}
            if alignment:
                alignment["_note"] = ("Default setup values from the chassis file (range start + "
                                      "setting x step), degrees converted to radians.")
        arb = {}
        arb_rate = (chassis or {}).get("arb_front_Nm" if axle == "front" else "arb_rear_Nm")
        if arb_rate:
            track = (vdfm or {}).get("track_front" if axle == "front" else "track_rear") or _DEFAULT_TRACK
            arb = {"bar_id": f"arb_{axle}",
                   # roll stiffness a wheel rate k adds across a track t is k t^2 / 2
                   "bar_rate": round(0.5 * arb_rate * track ** 2, 1),
                   "wheel_rate_equiv": float(arb_rate),
                   "motion_ratio": 1.0,
                   "_est": True,
                   "_note": ("wheel_rate_equiv is the bar rate of the chassis file (the setup screen "
                             "shows it in N/mm; confirmed on the GT86: 55 front, 20 rear). bar_rate is "
                             "the roll stiffness that wheel rate gives across the track, with the arm "
                             "geometry unknown (motion ratio 1 assumed).")}
        strut = next((l for l in links if l["type"] == "strut"), None)
        spring_d = {
            "type":    "coil",
            "rate":    spring,
            "_source": spring_src[axle],
        }
        c_crit = 2.0 * math.sqrt(spring * m_corner)            # N·s/m at the wheel
        if ccx and ccx.get("damper") and len(ccx["damper"]) == 4:
            dm = ccx.get("damper_mult") or 1.0
            sb, fb, sr, fr = (ccx["damper"][k] * dm for k in
                              ("SlowBump", "FastBump", "SlowRebound", "FastRebound"))
            tr = ccx.get("transition") or {}
            mr = math.sqrt(dm)               # DamperMult taken as the motion ratio squared

            def _curve(slow: float, fast: float, transition: Optional[float]) -> List[List[float]]:
                """Force against wheel speed: the slow slope up to the transition force, then the fast one."""
                if not transition or slow <= 0:
                    vk, fk = _DAMPER_KNEE, slow * dm * _DAMPER_KNEE
                else:
                    # damper level: knee at force F, speed F / slow; wheel level: speed / MR, force x MR
                    vk, fk = (abs(transition) / slow) / mr, abs(transition) * mr
                vk = min(vk, 0.9)
                return [[0.0, 0.0], [round(vk, 4), round(fk, 1)],
                        [1.0, round(fk + fast * dm * (1.0 - vk), 1)]]
            note = ("Slow and fast bump and rebound coefficients (N.s/m) times DamperMult, as force "
                    "against wheel speed (reference 'wheel'). The slope changes from slow to fast at "
                    "the bump/rebound transition force (N) of the chassis file, confirmed against the "
                    "game's setup screen for the GT86. DamperMult is taken as the motion ratio squared.")
            slow_bump, slow_reb = sb / dm, sr / dm
            src = "pcars2_cdfbin"
            x_p = {"damping_source": src,
                   "slow_bump": round(sb, 1), "fast_bump": round(fb, 1),
                   "slow_rebound": round(sr, 1), "fast_rebound": round(fr, 1),
                   "damper_mult": dm, "transition_raw": ccx.get("transition"),
                   "damping_ratio_bump": round(sb / c_crit, 3),
                   "damping_ratio_rebound": round(sr / c_crit, 3)}
            bump_curve = _curve(slow_bump, fb / dm, tr.get("BumpTransition"))
            reb_curve = _curve(slow_reb, fr / dm, tr.get("ReboundTransition"))
        else:
            c_bump, c_reb = round(_ZETA_BUMP * c_crit, 1), round(_ZETA_REBOUND * c_crit, 1)
            note = ("Not from the game: no damper data in the chassis file. Curves are linear, at "
                    "the wheel (reference 'wheel'): c = zeta x 2 x sqrt(spring.rate x corner sprung "
                    f"mass), zeta {_ZETA_BUMP} bump / {_ZETA_REBOUND} rebound (typical values).")
            src = "estimated_damping_ratio"
            x_p = {"damping_source": src, "damping_ratio_bump": _ZETA_BUMP,
                   "damping_ratio_rebound": _ZETA_REBOUND}
            bump_curve = [[0.0, 0.0], [0.5, round(c_bump * 0.5, 1)], [1.0, c_bump]]
            reb_curve = [[0.0, 0.0], [0.5, round(c_reb * 0.5, 1)], [1.0, c_reb]]
        damper_d = {
            "type": "twin_tube",
            # Force at the wheel against wheel speed; no motion_ratio needed.
            "reference": "wheel",
            # [velocity_m_s, force_N] pairs
            "bump_curve":    bump_curve,
            "rebound_curve": reb_curve,
            "_est": True,
            "_source": src,
            "_note": note,
            "x_pcars2": x_p,
        }
        if ccx and ccx.get("spring_mult"):
            spring_d["x_pcars2"] = {"rate_at_spring": ccx["spring_Nm"], "spring_mult": ccx["spring_mult"]}
        if spring_src[axle] != "pcars2_cdfbin":
            spring_d["_est"] = True
            spring_d["_note"] = ("No spring rate in the chassis file: derived from the car mass, "
                                 "weight split and a typical ride frequency "
                                 "(1.5 Hz front, 1.7 Hz rear).")
        if strut is not None:
            # On a strut the spring and damper sit on the strut axis.
            mounts = {"inboard_mount": strut["inboard_points"][0],
                      "outboard_mount": hardpoints["strut_lower"]}
            spring_d.update(mounts)
            damper_d.update(mounts)   # geometry only: the curves stay wheel-referenced

        return {
            "topology": {
                "system_type": system_type,
                "upright": {
                    "id":       node,
                    "mass":     round(sm["upright"]["mass"], 3) if sm.get("upright") else 12.0,
                    **({"_mass_source": "pcars2_sdfbin (non-rotating unsprung assembly)"}
                       if sm.get("upright") else {"_est": True}),
                    "hardpoints": hardpoints,
                },
                "links": links,
                **topo_extra,
            },
            "wheel": wheel_dict,
            **({"alignment": alignment} if alignment else {}),
            **({"arb": arb} if arb else {}),
            "spring": spring_d,
            "damper": damper_d,
            "brake": {
                "disc": {
                    "type":           "vented" if axle == "front" else "solid",
                    "outer_diameter": 0.330 if axle == "front" else 0.280,
                    "mass":           5.5   if axle == "front" else 4.0,
                    "material":       "cast_iron",
                    "_note":          "Placeholder — brake data not located in pCARS2 files.",
                },
            },
            "_est": True,
        }

    return {
        "FL": _corner("front", "left",  front_spring, front_mc, tire_f),
        "FR": _corner("front", "right", front_spring, front_mc, tire_f),
        "RL": _corner("rear",  "left",  rear_spring,  rear_mc,  tire_r),
        "RR": _corner("rear",  "right", rear_spring,  rear_mc,  tire_r),
        # No top-level _note: the schema forbids extra keys here (FL/FR/RL/RR
        # only). Provenance lives in _metadata._section_notes.suspension.
    }


def build_steering(car: str, cdf: Optional[Dict] = None) -> Dict:
    """Steering ratio and lock from the chassis file; the rack geometry stays a placeholder."""
    ratio, lock = (cdf or {}).get("steering_ratio"), (cdf or {}).get("steering_lock_deg")
    out: Dict[str, Any] = {
        "type":              "rack_and_pinion",
        "rack_position":     [0.0, 0.0, 0.0],
        "rack_travel":       0.08,
        "overall_ratio":     15.0,
        "lock_to_lock":      22.0,
        "lock_to_lock_turns": 3.5,
        "_est": True,
        "_note": "Placeholder — steering data not yet located in pCARS2 files.",
    }
    if ratio and lock:
        wheel_deg = 2.0 * lock * ratio                      # lock to lock at the steering wheel
        out.update({
            "overall_ratio": round(ratio, 3),
            "lock_to_lock": round(math.radians(wheel_deg), 4),
            "lock_to_lock_turns": round(wheel_deg / 360.0, 3),
            "_note": ("overall_ratio from SteeringRatioRange and lock_to_lock from SteeringLockRange "
                      "(road wheel angle, degrees) in the chassis file; lock to lock = 2 x lock x "
                      "ratio. rack_position and rack_travel are placeholders."),
        })
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Metadata helpers
# ─────────────────────────────────────────────────────────────────────────────

def _metadata(car: str, extracted_root: str, **files) -> Dict:
    """Build the SVJ _metadata section."""
    src = {k: (Path(v).name if v else None) for k, v in files.items()}
    game = _physics_dirs(extracted_root).get("game", "pc2")
    label = GAME_LABEL[game]
    if game == "pc1":
        detail = (
            "Extracted from Project CARS ShCB physics files via PCarsTools 1.1.4 (--game-type PC1). "
            "Springs, dampers, alignment, inertia, CG, steering, brakes, differentials and aero: chassis file. "
            "Engine torque curve, gear ratios and suspension links and masses: engine, gearbox and "
            "suspension files. Wheel centres, track, wheelbase and tyre size: the car's .vhf and tyre meshes. "
            "Mass, drivetrain, weight split: spec sheet (.mrdf) in HRDFPERSISTENT, or the chassis file "
            "when a car has none. Tire Pacejka D/C/E: hdtbin in PHYSICSBOOTFLOW; B estimated."
        )
    else:
        detail = (
            "Extracted from Project CARS 2 ShCB physics files via PCarsTools 1.1.4. "
            "Engine torque curve: edfbin hash 8b0ab771. "
            "Gear ratios: raw byte-scan of gdfbin (validated GT86/FR3.5/NSX). "
            "Springs, dampers, alignment, inertia, CG, steering, brakes, differentials: chassis file. "
            "Wheel centres, wheelbase, track, tire size: vdfm. "
            "Tire Pacejka D/C/E: hdtbin b31155cd/b1033b86/c0eaf58d; B estimated."
        )
    return {
        "specification":         "SVJ",
        "version":               SVJ_VERSION,
        "description":           f"{label} — {car} ({label} auto-converted)",
        "coordinate_system":     "SAE_J670",
        "units":                 "SI",
        "source_format":         label,
        "data_origin": {
            "type":       "simulation",
            "detail":     detail,
            "confidence": "low",
        },
        "alignment_convention":  "relative_to_centerline",
        "_converter":            "pCARS->SVJ Converter v0.4",
        "x_converter_repo":      "https://github.com/RFloEng/PC2-to-svj",
        "_source_game":          label,
        "_car_name":             car,
        "_source_files":         src,
        "_scipy_available":      HAS_SCIPY,
        "_known_placeholders": [
            "powertrain.layout (defaults to FR)",
            "vehicle_info.drive_type (defaults to RWD)",
            "chassis.center_of_gravity (height Z not stored; X from weight split when available)",
            "chassis.mass_total (default 1200 kg; no mrdf found)",
            "constraints / chassis.mass_bodies suspension links (estimated on corners "
            "whose topology has _est: link layout and all hardpoints except wheel_center)",
            "wheel_stations.*.brake (disc type/size estimated)",
            "steering.* (all placeholders)",
        ],
        # Notes for sections whose schema forbids extra keys
        "_section_notes": {
            "vehicle_info": ("make/model parsed from the physics file name; "
                             "drive_type defaults to RWD; year omitted when not in the name."),
            "wheel_stations": (
                "spring.rate from sdfbin hash 670b57ab block structure (N/mm→N/m), front/rear "
                "assigned by KC0 camber-gain ratio. damper: not decoded from the game; "
                "estimated wheel-level curves from a damping ratio (0.30 bump / 0.55 rebound), "
                "the spring rate and the corner mass (flagged _est)."
            ),
            "powertrain.engine": "displacement/configuration are not encoded in pCARS2 physics files.",
        },
    }


def _vehicle_info(car: str) -> Dict:
    """
    Build the SVJ vehicle_info section by parsing the car name.

    pCARS2 car names follow the pattern: make_model[_variant][_year]
    e.g. toyota_gt86, acura_nsx_2017, mclaren_p1, ferrari_458_gt3
    """
    parts = car.split("_")
    make = parts[0].title() if parts else "Unknown"

    # Detect 4-digit year in name
    year = None
    for p in parts:
        if p.isdigit() and len(p) == 4 and 1950 <= int(p) <= 2030:
            year = int(p)

    # Model: everything after make, without year
    model_parts = [p for p in parts[1:] if not (p.isdigit() and len(p) == 4)]
    model = " ".join(
        p.upper() if (p.isalnum() and not p.isalpha() and len(p) <= 6)
        else p.title()
        for p in model_parts
    ) if model_parts else "Unknown"

    # No _note (schema forbids extra keys here) and no year when unknown
    # (schema types it as integer); provenance is in _metadata._section_notes.
    info: Dict[str, Any] = {
        "make":       make,
        "model":      model,
        "drive_type": "RWD",    # default; pCARS2 drivetrain type not yet decoded
    }
    if year is not None:
        info["year"] = year
    return info


# ─────────────────────────────────────────────────────────────────────────────
# Top-level converter
# ─────────────────────────────────────────────────────────────────────────────

def convert_car(car: str, extracted_root: str) -> Dict:
    """Convert a single pCARS2 car to an SVJ-compliant dict."""
    print(f"\n  Converting: {car}")
    dirs = _physics_dirs(extracted_root)

    # Locate source files
    edf_path = _car_file(dirs, "engines",    car, "edfbin")
    cdf_path = _car_file(dirs, "chassis",    car, "cdfbin")
    gdf_path = _car_file(dirs, "gearbox",    car, "gdfbin")
    sdf_path = _car_file(dirs, "suspension", car, "sdfbin")
    hdt_path = _car_file(dirs, "tyres",      car, "hdtbin")

    _report_files(edfbin=edf_path, cdfbin=cdf_path, gdfbin=gdf_path,
                  sdfbin=sdf_path, hdtbin=hdt_path)

    # Parse ShCB files (with graceful fallback)
    sf_edf = _safe_shcb(edf_path, "engine")
    sf_cdf = _safe_shcb(cdf_path, "chassis")
    sf_gdf = _safe_shcb(gdf_path, "gearbox")
    sf_sdf = _safe_shcb(sdf_path, "suspension")
    sf_hdt = _safe_shcb(hdt_path, "tyres")

    # Parse vdfm for real wheel-centre positions and tire dimensions
    vdfm_data = _parse_vdfm(dirs, car)
    if vdfm_data and str(vdfm_data.get("_vdfm_file", "")).startswith("geometry.json"):
        print("  [OK] geometry: wheel centres and tyre size from the car's .vhf and tyre meshes")
    elif vdfm_data:
        print("  [OK] vdfm: wheel centres read from the descriptor")
    else:
        print("  [WARN] vdfm: not found - hardpoints will be empty")

    # The chassis file first: Project CARS has no spec sheet, so it stands in for it
    chassis_data: Dict[str, Any] = {}
    chassis_diffs: Dict[str, Any] = {}
    if sf_cdf is not None:
        try:
            chassis_data = _chassis_mod.parse_chassis(sf_cdf)
            chassis_diffs = _chassis_mod.parse_diffs(sf_cdf)
        except Exception as exc:                 # a malformed file must not stop the car
            print(f"  [WARN] chassis file not decoded: {exc}")

    # Game spec sheet: mass, drivetrain, weight distribution, displacement
    mrdf_data = _parse_mrdf(dirs, car) or {}
    if not mrdf_data and dirs.get("game") == "pc1":
        mrdf_data = _mrdf_from_chassis(chassis_data, chassis_diffs)
        print("  [INFO] no spec sheet for this car: mass, drivetrain and weight split from the chassis file")
    if mrdf_data.get("_from_chassis"):
        print(f"  [OK] chassis file: {mrdf_data.get('mass_kg')} kg with driver, "
              f"{mrdf_data.get('drive_type')} (from the differentials), rear weight "
              f"{mrdf_data.get('rear_weight_fraction')}")
    elif mrdf_data:
        print(f"  [OK] mrdf: {mrdf_data.get('mass_kg')} kg, "
              f"{mrdf_data.get('drive_type')}, {mrdf_data.get('displacement_l')} L")
    else:
        print("  [WARN] mrdf: not found - mass and drivetrain use defaults")

    print("  Building SVJ sections ...")

    # Engine
    engine = build_engine(sf_edf, car)
    if mrdf_data.get("displacement_l"):
        engine["displacement"] = mrdf_data["displacement_l"]
    if mrdf_data.get("engine_configuration"):
        engine["configuration"] = mrdf_data["engine_configuration"]
    if mrdf_data.get("boost_type"):
        engine["x_aspiration"] = mrdf_data["boost_type"]
    if mrdf_data.get("engine_configuration") or mrdf_data.get("boost_type"):
        engine["_spec_sheet_note"] = ("configuration and aspiration from the game's spec sheet "
                                      "(engine type and boost type fields)")

    drive = mrdf_data.get("drive_type")
    # FWD -> FF and AWD -> AWD are certain. For RWD the mrdf doesn't say
    # front/mid/rear engine, so FR stays a default.
    layout = {"FWD": "FF", "AWD": "AWD"}.get(drive, "FR")

    # Gearbox — extract final_drive for differentials
    gearbox_raw  = build_gearbox(sf_gdf, car)
    final_drive  = gearbox_raw.pop("final_drive", 3.600)
    gearbox_svj  = {k: v for k, v in gearbox_raw.items()}   # type, ratios, _note

    # The chassis file's gear settings pick the ratios out of the gearbox file's list; where
    # that works it replaces the heuristic reading above.
    driveline = chassis_data.get("driveline") or {}
    trans = None
    if sf_gdf is not None and driveline:
        try:
            trans = _chassis_mod.select_transmission(_chassis_mod.parse_gearbox_lists(sf_gdf._buf), driveline)
        except Exception as exc:                 # a malformed file must not stop the car
            print(f"  [WARN] transmission not decoded: {exc}")
    if trans:
        gearbox_svj["ratios"] = trans["ratios"]
        gearbox_svj.pop("_est", None)
        if trans.get("final_drive"):
            final_drive = trans["final_drive"]
        gearbox_svj["_note"] = (
            "ratios: the gear settings of the chassis file picked from the gearbox file's list "
            f"({len(trans['ratios'])} forward gears, {trans['index_order']} index); final drive from "
            "the final-drive setting times the bevel"
            + (" (the setting pointed past the last option: the last was used)"
               if trans.get("final_drive_clamped") else "")
            + ". Checked on the Toyota GT86 against the game's setup screen.")
        if trans.get("final_drive_options"):
            gearbox_svj["x_pcars2"] = {"final_drive_options": trans["final_drive_options"]}
    else:
        gearbox_svj["_est"] = True
        print("  [WARN] gearbox: gear settings do not fit the gearbox file - ratios from the older heuristic")
    if driveline.get("UpshiftDelay"):
        gearbox_svj["shift_time"] = round(driveline["UpshiftDelay"], 3)
        gearbox_svj.setdefault("x_pcars2", {}).update({
            "upshift_delay_s": driveline.get("UpshiftDelay"), "upshift_clutch_time_s": driveline.get("UpshiftClutchTime"),
            "downshift_delay_s": driveline.get("DownshiftDelay"), "downshift_clutch_time_s": driveline.get("DownshiftClutchTime"),
            "shift_time_note": "shift_time is the game's UpshiftDelay"})

    # driveline.rg: gearbox type, clutch, crank inertia, which diffs exist
    dl_info = _parse_driveline(dirs, car)
    clutch_svj = None
    if dl_info:
        print(f"  [OK] driveline: {dl_info['group']}")
        if sf_gdf is not None:
            gearbox_svj["type"] = dl_info["gearbox_type"]
            gearbox_svj["_note"] += (" type from driveline.rg group "
                                     f"{dl_info['group']}.")
        if dl_info.get("clutch"):
            clutch_svj = dict(dl_info["clutch"])
            clutch_svj["_source"] = f"driveline.rg group {dl_info['group']}"
        if dl_info.get("crank_inertia"):
            engine["inertia"] = dl_info["crank_inertia"]
            engine["_inertia_source"] = ("driveline.rg crankshaft mass (1-D shaft inertia, "
                                         "game units taken as kg*m^2)")
    else:
        print("  [WARN] driveline: no entry for this car - gearbox type and clutch use defaults")
        if "SemiAutomatic" in driveline:
            gearbox_svj["type"] = "sequential" if driveline["SemiAutomatic"] == 1 else "manual"
            gearbox_svj["_note"] = gearbox_svj.get("_note", "") + " type from the chassis file (SemiAutomatic)."
    if clutch_svj is None and driveline.get("ClutchTorque"):
        clutch_svj = {"max_torque": round(driveline["ClutchTorque"], 1), "_source": "chassis file (ClutchTorque)"}
        if driveline.get("ClutchInertia"):
            clutch_svj["inertia_driven"] = round(driveline["ClutchInertia"], 5)

    if chassis_data.get("corners"):
        print("  [OK] chassis: springs, dampers, inertia, alignment from the cdfbin")
    suspension = build_suspension(sf_sdf, sf_hdt, car, vdfm=vdfm_data,
                                  mass_kg=mrdf_data.get("mass_kg"),
                                  rear_fraction=mrdf_data.get("rear_weight_fraction"),
                                  chassis=chassis_data)

    # Build top-level tires section from suspension corner data
    fl_wheel = suspension.get("FL", {}).get("wheel", {})
    tire_model = fl_wheel.get("_tire_model", {})
    tires = {
        "sets": {
            "default": {
                "description": "Estimated from pCARS2 hdtbin Pacejka mu values",
                "source":      "estimated",
                "reference": {
                    "pressure": fl_wheel.get("_pressure_nominal", 200000.0),
                },
                "pacejka": {
                    "model":        "MF52",
                    "lateral":      tire_model.get("lateral", {}),
                    "longitudinal": tire_model.get("longitudinal", {}),
                    "_mu_lateral":      tire_model.get("mu_lateral", 1.4),
                    "_mu_longitudinal": tire_model.get("mu_longitudinal", 1.3),
                    "_B_estimated":     True,
                    "_note": ("Simplified BCDE approximation — not full MF52 parameterisation. "
                              + tire_model.get("_B_note", "")),
                },
            }
        }
    }

    # Build top-level brakes section from suspension corners
    brakes_corners = {}
    for corner in ("FL", "FR", "RL", "RR"):
        disc = suspension.get(corner, {}).get("brake", {}).get("disc", {})
        if disc:
            brakes_corners[corner] = {"disc": disc}
    brakes = {
        "booster": {"type": "vacuum"},
        "corners":  brakes_corners,
        "_note": "Brake disc dimensions estimated; type/size not decoded from pCARS2 physics.",
    }
    cc = chassis_data.get("corners") or []
    if len(cc) >= 4 and cc[0].get("brake_torque_Nm") and cc[2].get("brake_torque_Nm"):
        brakes["_max_brake_torque_Nm"] = {"front": cc[0]["brake_torque_Nm"], "rear": cc[2]["brake_torque_Nm"]}
        if chassis_data.get("brake_bias_rear") is not None:
            brakes["_brake_bias_rear"] = round(chassis_data["brake_bias_rear"], 3)
        brakes["_chassis_file_note"] = ("BrakeTorque per axle and RearBrakeRange (rear share of "
                                        "the brake force) from the chassis file; no SVJ field yet.")

    metadata = _metadata(
        car=car, extracted_root=extracted_root,
        edfbin=edf_path, cdfbin=cdf_path,
        gdfbin=gdf_path, sdfbin=sdf_path, hdtbin=hdt_path,
        vdfm=(vdfm_data or {}).get("_vdfm_file"),
        mrdf=mrdf_data.get("_mrdf_file"),
    )
    if vdfm_data:
        # Game-frame Z of the front axle: lets later passes (collision hulls)
        # place game-frame points in SVJ coordinates without the vdfm.
        metadata["_vhf_front_axle_z"] = vdfm_data["_front_axle_x"]
    # Drop placeholder entries the spec sheet resolved
    resolved = set()
    if mrdf_data.get("mass_kg"):
        resolved.add("chassis.mass_total")
    if drive:
        resolved.add("vehicle_info.drive_type")
    if drive in ("FWD", "AWD"):
        resolved.add("powertrain.layout")
    if all(not suspension.get(c, {}).get("topology", {}).get("_est")
           for c in ("FL", "FR", "RL", "RR")):
        resolved.add("constraints")
    metadata["_known_placeholders"] = [
        p for p in metadata["_known_placeholders"]
        if p.split(" ")[0] not in resolved
    ]

    vehicle_info = _vehicle_info(car)
    if drive:
        vehicle_info["drive_type"] = drive

    diff_loc = "front" if drive == "FWD" else "rear"

    svj: Dict[str, Any] = {
        "_metadata": metadata,
        "vehicle_info": vehicle_info,
        "chassis":      build_chassis(car, vdfm=vdfm_data, mrdf=mrdf_data, cdf=chassis_data),
        "steering":     build_steering(car, chassis_data),
        "suspension":   suspension,
        "powertrain": {
            "layout":  layout,
            "engine":  engine,
            **({"clutch": clutch_svj} if clutch_svj else {}),
            "gearbox": gearbox_svj,
            "differentials": [
                {
                    "id":          f"diff_{diff_loc}",
                    "location":    diff_loc,
                    "type":        "open",
                    "final_drive": round(final_drive, 5),
                    "_note": (
                        "type defaults to 'open': the game names LSD type and preload "
                        "per car in variables that aren't in the extracted files."
                    ),
                }
            ] + _extra_diffs(dl_info, drive, diff_loc),
            "_note": ("layout: FF/AWD from the mrdf drivetrain field; RWD cars "
                      "default to FR because engine position isn't decoded."),
        },
        "tires":        tires,
        "brakes":       brakes,
        "aerodynamics": build_aerodynamics(sf_cdf, car),
        "assets": {
            "meshes": [],   # no mesh data available from pCARS2 physics-only extraction
        },
    }
    bench = []
    for key, bid, unit in (("top_speed_ms", "top_speed", "m/s"), ("accel_0_100kph_s", "accel_0_100kph", "s"),
                           ("power_W", "peak_power", "W"), ("torque_Nm", "peak_torque", "N*m")):
        if mrdf_data.get(key) is not None:
            bench.append({"id": bid, "value": mrdf_data[key], "unit": unit, "type": "simulated",
                          "source": "the game's own spec sheet (statistics .mrdf)"})
    if bench:
        svj["benchmarks"] = bench
    if sf_cdf is not None:
        try:
            _apply_chassis_diffs(svj["powertrain"]["differentials"], chassis_diffs)
        except Exception as exc:               # a malformed file must not stop the car
            print(f"  [WARN] differentials not read from the chassis file: {exc}")
    return _unify_suspension(svj)


# ─────────────────────────────────────────────────────────────────────────────
# Output helpers (shared with the GUI)
# ─────────────────────────────────────────────────────────────────────────────

def glb_node_names(glb_path: Path) -> Optional[set]:
    """Node names in a .glb (reads the JSON chunk directly), or None if unreadable."""
    try:
        data = glb_path.read_bytes()
        if data[:4] != b"glTF":
            return None
        chunk_len = struct.unpack_from("<I", data, 12)[0]
        doc = json.loads(data[20:20 + chunk_len])
        return {n.get("name") for n in doc.get("nodes", []) if n.get("name")}
    except (OSError, ValueError, struct.error):
        return None


def _prune_bindings(svj: Dict, svj_dir: Path) -> None:
    """
    Drop visual bindings that don't resolve: mesh_ref not in assets.meshes,
    or a node name the referenced GLB doesn't contain (when it can be read).
    """
    meshes = {m["id"]: m.get("uri") for m in svj.get("assets", {}).get("meshes", [])
              if isinstance(m, dict) and "id" in m}
    names_cache: Dict[str, Optional[set]] = {}

    def ok(v: Dict) -> bool:
        ref = v.get("mesh_ref") or (next(iter(meshes)) if len(meshes) == 1 else None)
        if ref not in meshes:
            return False
        if ref not in names_cache:
            uri = meshes[ref]
            names_cache[ref] = glb_node_names(svj_dir / uri) if uri else None
        names = names_cache[ref]
        return names is None or v.get("node") in names

    def walk(o: Any) -> None:
        if isinstance(o, dict):
            if isinstance(o.get("visual"), dict) and not ok(o["visual"]):
                del o["visual"]
            for v in o.values():
                walk(v)
        elif isinstance(o, list):
            for v in o:
                walk(v)
    walk(svj)


_STATION = {"fl": "FL", "fr": "FR", "rl": "RL", "rr": "RR"}


def apply_bindings(svj: Dict, mesh_id: str, nodes: Any) -> None:
    """
    Write SVJ visual bindings (spec section 22.3) for the glTF nodes a GLB really
    contains, at the binding site each node belongs to in the unified layout:
    chassis, steering wheel, wheel bodies (mass_bodies) and wheel_stations brakes.
    Nodes with no site get no binding; pCARS2 has no separate upright mesh, so
    uprights stay unbound.
    """
    def bind(node: str) -> Dict:
        return {"mesh_ref": mesh_id, "node": node}

    def strip(o: Any) -> None:
        if isinstance(o, dict):
            o.pop("visual", None)
            for v in o.values():
                strip(v)
        elif isinstance(o, list):
            for v in o:
                strip(v)
    strip(svj)

    bodies = {b.get("id"): b for b in svj.get("chassis", {}).get("mass_bodies", [])
              if isinstance(b, dict)}
    stations = {s.get("id"): s for s in svj.get("wheel_stations", []) if isinstance(s, dict)}
    for node in sorted(nodes):
        parts = node.split("::")
        if len(parts) != 3:
            continue
        _, category, ident = parts
        if node == "SVJ::body::chassis" and "chassis" in svj:
            svj["chassis"]["visual"] = bind(node)
        elif node == "SVJ::steering::wheel" and "steering" in svj:
            svj["steering"].setdefault("steering_wheel", {})["visual"] = bind(node)
        elif category in ("wheel", "brake"):
            part, _, corner = ident.rpartition("_")
            station = stations.get(_STATION.get(corner, ""))
            if station is None:
                continue
            if part == "wheel" and f"wheel_{corner}" in bodies:
                bodies[f"wheel_{corner}"]["visual"] = bind(node)
            elif part == "disc" and "disc" in station.get("brake", {}):
                station["brake"]["disc"]["visual"] = bind(node)
            elif part == "caliper" and "brake" in station:
                station["brake"].setdefault("caliper", {})["visual"] = bind(node)


def _bound_nodes(o: Any, out: Optional[set] = None) -> set:
    """Every SVJ:: node name bound by a 'visual' block anywhere in o."""
    out = set() if out is None else out
    if isinstance(o, dict):
        v = o.get("visual")
        if isinstance(v, dict) and str(v.get("node", "")).startswith("SVJ::"):
            out.add(v["node"])
        for x in o.values():
            _bound_nodes(x, out)
    elif isinstance(o, list):
        for x in o:
            _bound_nodes(x, out)
    return out


def collision_sidecar(svj_path: Any) -> Path:
    """<car>.collision.json next to <car>.svj.json: the parsed collision XML of the car."""
    p = Path(svj_path)
    return p.with_name(p.name.replace(".svj.json", "") + ".collision.json")


def apply_collision_sidecar(svj: Dict, svj_path: Any) -> Optional[Dict[str, int]]:
    """Add collision-derived bodies, hinges and proxies if the car has a sidecar."""
    side = collision_sidecar(svj_path)
    if not side.is_file() or "_vhf_front_axle_z" not in svj.get("_metadata", {}):
        return None
    try:
        data = json.loads(side.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return _collision_mod.apply_collision(svj, data)


def write_svj(svj: Dict, out_path: str, indent: Optional[int] = 2) -> None:
    """
    Write an SVJ dict to disk, preserving mesh links and visual bindings
    from a previous run.

    The physics converter can't know about meshes, so it always emits
    assets.meshes = [] and no visual bindings. Mesh extraction fills those in
    afterwards; without this, re-running the physics conversion would
    silently drop them.
    """
    p = Path(out_path)
    if p.is_file() and not svj.get("assets", {}).get("meshes"):
        try:
            old = json.loads(p.read_text(encoding="utf-8"))
            old_meshes = old.get("assets", {}).get("meshes")
            if old_meshes:
                for mesh in old_meshes:
                    # Files from before the id fix may carry hyphenated ids,
                    # which the schema's ^[a-z0-9_]+$ pattern rejects.
                    if isinstance(mesh, dict) and isinstance(mesh.get("id"), str):
                        mesh["id"] = mesh_id(mesh["id"])
                svj.setdefault("assets", {})["meshes"] = old_meshes
                fresh = json.loads(json.dumps(svj))
                apply_bindings(fresh, old_meshes[0]["id"], _bound_nodes(old))
                # Keep only bindings that resolve to a node in the GLB; this
                # also drops pre-0.99.2 bindings to nodes the GLB never had.
                _prune_bindings(fresh, p.parent)
                svj.clear(); svj.update(fresh)
        except (OSError, ValueError):
            pass   # unreadable old file: just overwrite it
    apply_collision_sidecar(svj, p)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as fh:
        json.dump(svj, fh, indent=indent, ensure_ascii=False)


_SCHEMA_PATH = (_HERE / "svj_spec" / "SVJ-standard-vehicle-json-main"
                / "schema" / "svj.schema.json")
_validator = None


def validate_svj(svj: Dict) -> Optional[List[str]]:
    """
    Validate against the bundled SVJ JSON schema.

    Returns a list of error strings (empty = valid), or None when validation
    isn't possible (jsonschema not installed or schema file missing).
    """
    global _validator
    if _validator is None:
        try:
            import jsonschema
            schema = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))
            _validator = jsonschema.validators.validator_for(schema)(schema)
        except (ImportError, OSError, ValueError):
            _validator = False
    if _validator is False:
        return None
    return [
        f"{'/'.join(str(p) for p in e.absolute_path) or '<root>'}: {e.message}"
        for e in _validator.iter_errors(svj)
    ]


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _report_files(**kwargs):
    print()
    for label, path in kwargs.items():
        status = Path(path).name if path else "NOT FOUND"
        print(f"  {label.upper():<10} {status}")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    sys.stdout.reconfigure(encoding="utf-8")

    ap = argparse.ArgumentParser(
        prog="pcars2_to_svj",
        description=(
            "Convert extracted Project CARS 2 physics files to SVJ format.\n"
            "Run `--list` to see available cars, then convert with `--car`."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples
--------
  # List cars in an extracted folder
  python pcars2_to_svj.py --dir ./extracted --list

  # Convert a single car
  python pcars2_to_svj.py --dir ./extracted --car acura_nsx_2017

  # Convert and specify output path
  python pcars2_to_svj.py --dir ./extracted --car mclaren_p1 --out p1.svj.json

  # Batch: convert every available car to an output directory
  python pcars2_to_svj.py --dir ./extracted --all --outdir ./cars

See PCARS2_EXTRACTION_NOTES.md for the full extraction recipe.
        """,
    )
    ap.add_argument("--dir", required=True,
                    help="Root of extracted pCARS2 physics "
                         "(parent of PHYSICSMENU/ and PHYSICSPERSISTENT/)")
    ap.add_argument("--car", default=None,
                    help="Car name to convert (e.g. acura_nsx_2017)")
    ap.add_argument("--out", default=None,
                    help="Output file (default: <car>.svj.json)")
    ap.add_argument("--all", action="store_true",
                    help="Convert all available cars")
    ap.add_argument("--outdir", default=None,
                    help="Output directory for --all batch mode")
    ap.add_argument("--list", action="store_true",
                    help="List available car names and exit")
    ap.add_argument("--compact", action="store_true",
                    help="Write compact (non-pretty) JSON")

    args = ap.parse_args()

    if not os.path.isdir(args.dir):
        ap.error(f"--dir not found: {args.dir}")

    if args.list:
        cars = list_cars(args.dir)
        if not cars:
            print("No cars found. Check that the extracted root contains "
                  "PHYSICSMENU/vehicles/physics/engines/*.edfbin")
        else:
            print(f"{len(cars)} cars available:")
            for c in cars:
                print(f"  {c}")
        return

    print("=" * 62)
    print("  Project CARS / Project CARS 2 -> SVJ Converter  v0.4")
    print("=" * 62)
    print(f"  Source  : {args.dir}")
    print(f"  Scipy   : {'available' if HAS_SCIPY else 'NOT found (simplified Pacejka)'}")

    indent = None if args.compact else 2

    if args.all:
        cars = list_cars(args.dir)
        if not cars:
            print("No cars found.")
            return
        # One folder per car (<outdir>/<car>/<car>.svj.json); the mesh step adds
        # <car>.glb next to it. Default <dir>/cars.
        outdir = Path(args.outdir) if args.outdir else Path(args.dir) / "cars"
        outdir.mkdir(parents=True, exist_ok=True)
        print(f"\n  Batch mode: {len(cars)} cars -> {outdir}")
        ok = err = invalid = 0
        for car in cars:
            try:
                svj = convert_car(car, args.dir)
                out_path = outdir / car / f"{car}.svj.json"
                write_svj(svj, str(out_path), indent=indent)
                problems = validate_svj(svj)
                if problems:
                    invalid += 1
                    print(f"  OK {car} -> {car}/{out_path.name}  "
                          f"[SCHEMA: {len(problems)} errors, first: {problems[0]}]")
                else:
                    print(f"  OK {car} -> {car}/{out_path.name}")
                ok += 1
            except Exception as exc:
                print(f"  FAIL {car}: {exc}")
                err += 1
        schema_msg = ("schema check skipped (jsonschema not installed)"
                      if validate_svj({}) is None
                      else f"{invalid} schema-invalid")
        print(f"\n  Batch done: {ok}/{len(cars)} OK, {err} errors, {schema_msg}.")
        return

    # Single car
    car = args.car
    if not car:
        ap.error("Specify --car <name> or --list to see available cars.")

    out_path = args.out or f"{car}.svj.json"

    try:
        svj = convert_car(car, args.dir)
    except Exception as exc:
        print(f"\n  ERROR: {exc}")
        sys.exit(1)

    write_svj(svj, out_path, indent=indent)
    problems = validate_svj(svj)
    if problems:
        print(f"\n  [SCHEMA] {len(problems)} validation errors:")
        for p in problems[:10]:
            print(f"    {p}")
    print(f"\n  Done -- SVJ written to: {out_path}")
    print("=" * 62)


if __name__ == "__main__":
    main()
