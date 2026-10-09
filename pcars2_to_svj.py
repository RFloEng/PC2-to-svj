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
  chassis.mass_total         cdfbin           ⚠ scan-based, verify against known specs
  chassis.center_of_gravity  —                ❌ not located; zeros written
  powertrain.layout          —                ❌ defaults to FR
  powertrain.engine          edfbin           ✅ full RPM + torque curve
  powertrain.gearbox.ratios  gdfbin           ✅ validated on GT86, FR3.5, NSX
  powertrain.differentials   gdfbin           ⚠ final_drive extracted; type defaults open
  suspension.*.spring.rate   sdfbin           ✅ block structure (N/mm→N/m); front/rear by KC0
  suspension.*.damper        sdfbin           ✅ 777 Ns/m road reference; race=hash=063ea245
  suspension.*.wheel_center  vdfm             ✅ exact wheel-centre XYZ from physics
  suspension.*.wheel         hdtbin+vdfm      ✅ D from b31155cd; C from b1033b86; E from c0eaf58d
  suspension.*.tire_dims     vdfm             ✅ section_width + free_radius per axle
  suspension.*.brake         —                ❌ defaults only
  aerodynamics (_ext.)       cdfbin           ⚠ likely CdA; see _aerodynamics key

Usage
-----
  python pcars2_to_svj.py --dir ./extracted --list
  python pcars2_to_svj.py --dir ./extracted --car acura_nsx_2017 --out nsx.svj.json
  python pcars2_to_svj.py --dir ./extracted --all --outdir ./svj_output

Requirements
------------
  shcb_reader.py must be in the same directory as this script.
  scipy / numpy optional: enables Pacejka fitting (falls back to estimation).

SVJ schema version targeted: 0.97
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

def _physics_dirs(extracted_root: str) -> Dict[str, Optional[str]]:
    """
    Locate the per-type physics subdirectories under the extracted root.
    Returns a dict of {type_name: absolute_path_or_None}.
    """
    root = Path(extracted_root)
    menu = root / "PHYSICSMENU" / "vehicles" / "physics"
    pers = root / "PHYSICSPERSISTENT" / "vehicles" / "physics"

    def d(p): return str(p) if p.is_dir() else None

    return {
        "engines":    d(menu / "engines"),
        "chassis":    d(menu / "chassis"),
        "gearbox":    d(menu / "gearbox"),
        "suspension": d(menu / "suspension"),
        "tyres":      d(pers / "tyres"),
        "vehicles":   d(pers / "vehicles"),   # .vdfm master descriptors
        "statistics": d(pers / "statistics"), # .mrdf car-select spec sheets
    }


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
        return None
    # Resolve car name → vdfm stem (may differ from car name)
    stem = _VDFM_ALIASES.get(car, car)
    p = Path(veh_dir) / f"{stem}.vdfm"
    if not p.is_file():
        ranked = rank_candidates(stem, Path(veh_dir).glob("*.vdfm"), strict=True)
        if not ranked:
            return None
        p = ranked[0]

    try:
        data = p.read_bytes()
        if len(data) < 216:
            return None
        fl_v = struct.unpack_from("<3f", data, 136)
        fr_v = struct.unpack_from("<3f", data, 148)
        rl_v = struct.unpack_from("<3f", data, 160)
        rr_v = struct.unpack_from("<3f", data, 172)
        twf  = struct.unpack_from("<f",  data, 184)[0]
        odf  = struct.unpack_from("<f",  data, 188)[0]
        twr  = struct.unpack_from("<f",  data, 200)[0]
        odr  = struct.unpack_from("<f",  data, 204)[0]

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


_MRDF_DRIVE = {0: "RWD", 1: "AWD", 2: "FWD"}


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
    return out


def _parse_sdf_physics(sf_sdf: Optional[ShCBFile]) -> Dict[str, float]:
    """
    Extract per-axle spring rates from sdfbin hash=670b57ab block structure.

    The record body is a stream of 66-byte (road) or 69-byte (some race) repeating
    blocks.  Each block starts with a spring-rate f32 (N/mm); a damper-reference
    constant (~777 Ns/m) appears 20 bytes later and is used to locate the blocks.
    Blocks[1] and blocks[3] form the two axle pairs; the axle with smaller KC0
    (camber-gain ratio at base+76) is assigned as front.

    Returns:
        dict with keys  front_spring_Nm, rear_spring_Nm  (SI: N/m),
        or empty dict when the record is absent / unparseable.
    """
    if sf_sdf is None:
        return {}
    rec = sf_sdf.get_one("670b57ab")
    if not rec:
        return {}
    body = rec.body

    # Locate damper-reference positions (770–790 Ns/m) at every byte offset
    dpos: List[int] = []
    for off in range(len(body) - 3):
        v = struct.unpack_from("<f", body, off)[0]
        if not (math.isnan(v) or math.isinf(v)) and 770.0 <= v <= 790.0:
            dpos.append(off)
    if not dpos:
        return {}

    strides = [dpos[i + 1] - dpos[i] for i in range(min(3, len(dpos) - 1))]
    stride = strides[0] if strides else 0
    if stride < 10:
        return {}

    # Parse blocks: spring at base, KC0 at base+76
    blocks: List[Dict] = []
    for dp in dpos[:6]:
        base = dp - 20
        if base < 0:
            continue
        spring_v = struct.unpack_from("<f", body, base)[0]
        spring = (round(spring_v, 4)
                  if not (math.isnan(spring_v) or math.isinf(spring_v))
                  and 0.5 < spring_v < 500.0
                  else None)

        kc0: Optional[float] = None
        k_off = base + 76
        if k_off + 4 <= len(body):
            kv = struct.unpack_from("<f", body, k_off)[0]
            if not (math.isnan(kv) or math.isinf(kv)) and 0.01 < abs(kv) < 10.0:
                kc0 = abs(kv)

        blocks.append({"spring": spring, "kc0": kc0})

    if len(blocks) < 4:
        return {}

    # Front = axle with smaller KC0 (less suspension camber gain)
    kc1 = blocks[1].get("kc0") or 0.0
    kc3 = blocks[3].get("kc0") or 0.0
    front_blk, rear_blk = (1, 3) if kc1 <= kc3 else (3, 1)

    result: Dict[str, float] = {}
    for key, bi in [("front_spring_Nm", front_blk), ("rear_spring_Nm", rear_blk)]:
        s = blocks[bi].get("spring")
        if s and 1.0 < s < 500.0:
            result[key] = round(s * 1000.0)   # N/mm → N/m
    return result


def _safe_shcb(path: Optional[str], label: str) -> Optional[ShCBFile]:
    """Open a ShCBFile, printing a warning and returning None on failure."""
    if not path:
        print(f"  [WARN] {label}: file not found, section will use defaults")
        return None
    try:
        return ShCBFile(path)
    except Exception as exc:
        print(f"  [WARN] {label}: parse error — {exc}")
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
                  mrdf: Optional[Dict] = None) -> Dict:
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
    mass_source = "pcars2_mrdf" if mass else "default"
    if not mass:
        mass = _DEFAULT_MASS
        print(f"  [WARN] {car}: no mrdf mass; using default {_DEFAULT_MASS:.0f} kg")

    cg = [0.0, 0.0, 0.0]
    frac = (mrdf or {}).get("rear_weight_fraction")
    if frac and vdfm:
        cg[0] = round(-frac * vdfm["wheelbase"], 4)

    chassis: Dict[str, Any] = {
        "mass_total":        mass,
        "center_of_gravity": cg,
        "visual": {
            "mesh_ref": car,
            "node":     "SVJ::body::chassis",
        },
        "_mass_source": mass_source,
        "_note": (
            "mass_total: game spec-sheet weight from statistics .mrdf (default if absent). "
            "center_of_gravity: X from mrdf rear weight fraction x vdfm wheelbase "
            "(0 if either is missing); Y centreline; Z height not stored by pCARS2 "
            "(placeholder 0). wheelbase/track: vdfm wheel centres."
        ),
    }
    if vdfm:
        chassis["wheelbase"]   = vdfm["wheelbase"]
        chassis["track_front"] = vdfm["track_front"]
        chassis["track_rear"]  = vdfm["track_rear"]
    return chassis


def build_aerodynamics(sf_cdf: Optional[ShCBFile], car: str) -> Dict:
    """
    Build SVJ-schema-compliant aerodynamics section from cdfbin.

    Hash e0a125de (type 0xa2): stores a two-f32 pair.
      - body[0]: cross-checked across cars as drag-related (0.20 for NSX 2017)
      - body[4]: lift/downforce related (0.50 for NSX 2017)

    Hash 6f70f3c7 (type 0xa2): first plausible f32 ≈ 0.70 for NSX 2017.
      Cross-check: NSX Cd × frontal_area = 0.32 × 2.2 ≈ 0.70 → likely CdA (m²).
    """
    DEFAULT_FRONTAL_AREA = 2.1   # m², typical sports car

    if sf_cdf is None:
        return {
            "reference":    {"frontal_area": DEFAULT_FRONTAL_AREA},
            "coefficients": {"Cd": 0.35, "Cl": 0.0},
            "_note": "cdfbin not found — aerodynamic defaults only",
        }

    cd = cl = cda = None

    r_aero = sf_cdf.get_one("e0a125de")
    if r_aero and len(r_aero.body) >= 8:
        cd = struct.unpack_from("<f", r_aero.body, 0)[0]
        cl = struct.unpack_from("<f", r_aero.body, 4)[0]

    r_cda = sf_cdf.get_one("6f70f3c7")
    if r_cda and len(r_cda.body) >= 8:
        cda = struct.unpack_from("<f", r_cda.body, 4)[0]  # offset 4 in type a2

    # CdA (hash 6f70f3c7) appears to store Cd × frontal_area directly.
    # NSX validation: 0.70 m² = 0.32 (real Cd) × 2.19 m² (real A) ✓
    if cda and 0.2 < cda < 4.0:
        cd_derived = round(cda / DEFAULT_FRONTAL_AREA, 4)
    else:
        cd_derived = 0.35   # generic fallback

    cl_out = round(cl, 4) if (cl is not None and 0.0 <= abs(cl) <= 5.0) else 0.0

    return {
        "reference": {
            "frontal_area": DEFAULT_FRONTAL_AREA,
        },
        "coefficients": {
            "Cd": cd_derived,
            "Cl": cl_out,
        },
        "_cl_sign_note": "Negative Cl = downforce",
        "_cda_raw":      round(cda, 4) if cda else None,
        "_note": (
            "Cd derived from CdA (hash 6f70f3c7) / 2.1 m² default frontal area. "
            "Validated: NSX CdA=0.70 -> Cd=0.333 (real=0.32). "
            "Cl from hash e0a125de[4]; sign/scale requires per-car calibration."
        ),
    }


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


# Wheel-centre fallback when no vdfm is found (SVJ SAE J670, front-axle origin)
_DEFAULT_WHEELBASE = 2.60
_DEFAULT_TRACK     = 1.55
_DEFAULT_WHEEL_R   = 0.33


def build_suspension(sf_sdf: Optional[ShCBFile],
                     sf_hdt: Optional[ShCBFile],
                     car: str,
                     vdfm: Optional[Dict] = None) -> Dict:
    """
    Build SVJ suspension as FL/FR/RL/RR per-corner dicts.

    Spring rates: extracted from sdfbin hash=670b57ab block structure.
      Each block starts with spring rate (N/mm); blocks are found via the
      damper-reference constant (~777 Ns/m at block+20).  Front/rear assignment
      uses KC0 (camber-gain ratio): lower KC = front axle.
    Damper: pCARS2 uses a reference constant 777 Ns/m in the sdfbin for road
      cars (all values ~776.99, car-independent).  Race cars with explicit
      damper records (hash=063ea245) use those values instead.
    Topology: defaults to double_wishbone (most pCARS2 cars).

    Tire and brake data are embedded within each corner per the SVJ schema.
    """
    # Real spring rates from sdfbin block structure (N/m)
    sdf_physics = _parse_sdf_physics(sf_sdf)
    front_spring = sdf_physics.get("front_spring_Nm", 55_000.0)
    rear_spring  = sdf_physics.get("rear_spring_Nm",  50_000.0)

    # Damper: pCARS2 stores ~777 Ns/m reference in sdfbin for road cars.
    # Race cars with hash=063ea245 have explicit per-car values.
    front_damper = rear_damper = 777.0
    if sf_sdf is not None:
        for h in ("063ea245", "06bea245"):
            r_damp = sf_sdf.get_one(h)
            if r_damp:
                # Scan body for values in realistic race damper range (500-8000 Ns/m)
                damp_candidates = []
                for k in range(0, len(r_damp.body) - 3, 4):
                    v = struct.unpack_from("<f", r_damp.body, k)[0]
                    if not (math.isnan(v) or math.isinf(v)) and 400.0 <= v <= 8000.0:
                        damp_candidates.append(v)
                if damp_candidates:
                    # Use median of candidates to avoid outliers
                    damp_candidates.sort()
                    mid = damp_candidates[len(damp_candidates) // 2]
                    front_damper = rear_damper = round(mid, 1)
                break

    tire_f = _tire_data(sf_hdt, car, "front")
    tire_r = _tire_data(sf_hdt, car, "rear")

    def _corner(axle: str, side: str, spring: float, damper: float,
                tire: Dict) -> Dict:
        tag  = axle[0].upper() + side[0].upper()   # FL, FR, RL, RR
        node = f"upright_{tag.lower()}"

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

        hardpoints, links = _estimated_double_wishbone(wc_sae, axle)

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
        if free_radius is not None:
            wheel_dict["_free_radius"]   = free_radius
            wheel_dict["_section_width"] = section_width
            wheel_dict["_tire_source"]   = "pcars2_vdfm"

        return {
            "topology": {
                "system_type": "double_wishbone",   # default — pCARS2 doesn't expose this
                "upright": {
                    "id":       node,
                    "mass":     12.0,
                    "hardpoints": hardpoints,
                    "body_ref": node,
                },
                "links": links,
                "_est": True,
                "_wheel_center_source": wc_source,
                "_note": (
                    "Only wheel_center comes from game data (when _wheel_center_source "
                    "is pcars2_vdfm). pCARS2 models kinematics with K&C lookup tables, "
                    "not link geometry, so system_type, the other hardpoints and the "
                    "links are representative double-wishbone estimates."
                ),
            },
            "wheel": wheel_dict,
            "spring": {
                "type":    "coil",
                "rate":    spring,
                "_source": "pcars2_sdfbin" if sdf_physics else "default",
            },
            "damper": {
                "type": "twin_tube",
                # [velocity_m_s, force_N] pairs; force = coefficient × velocity
                "bump_curve":    [[0.0, 0.0],
                                  [0.5, round(damper * 0.5, 1)],
                                  [1.0, round(damper, 1)]],
                "rebound_curve": [[0.0, 0.0],
                                  [0.5, round(damper * 0.75, 1)],
                                  [1.0, round(damper * 1.5, 1)]],
                "_source": "pcars2_sdfbin",
                "_note":   "pCARS2 stores ~777 Ns/m reference constant in sdfbin (road cars). "
                           "Race cars with hash=063ea245 use explicit per-car values.",
            },
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
            "visual": {
                "mesh_ref": car,
                "node":     f"SVJ::body::{node}",
            },
        }

    return {
        "FL": _corner("front", "left",  front_spring, front_damper, tire_f),
        "FR": _corner("front", "right", front_spring, front_damper, tire_f),
        "RL": _corner("rear",  "left",  rear_spring,  rear_damper,  tire_r),
        "RR": _corner("rear",  "right", rear_spring,  rear_damper,  tire_r),
        # No top-level _note: the schema forbids extra keys here (FL/FR/RL/RR
        # only). Provenance lives in _metadata._section_notes.suspension.
    }


def build_steering(car: str) -> Dict:
    """Placeholder steering section (pCARS2 steering data not yet decoded)."""
    return {
        "type":              "rack_and_pinion",
        "rack_position":     [0.0, 0.0, 0.0],
        "rack_travel":       0.08,
        "overall_ratio":     15.0,
        "lock_to_lock":      22.0,
        "lock_to_lock_turns": 3.5,
        "_est": True,
        "_note": "Placeholder — steering data not yet located in pCARS2 files.",
    }


# ─────────────────────────────────────────────────────────────────────────────
# Metadata helpers
# ─────────────────────────────────────────────────────────────────────────────

def _metadata(car: str, extracted_root: str, **files) -> Dict:
    """Build the SVJ _metadata section."""
    src = {k: (Path(v).name if v else None) for k, v in files.items()}
    return {
        "specification":         "SVJ",
        "version":               "0.97",
        "description":           f"Project CARS 2 — {car} (pCARS2→SVJ auto-converted)",
        "coordinate_system":     "SAE_J670",
        "units":                 "SI",
        "source_format":         "Project CARS 2",
        "data_origin": {
            "type":       "simulation",
            "detail": (
                "Extracted from Project CARS 2 ShCB physics files via PCarsTools 1.1.4. "
                "Engine torque curve: edfbin hash 8b0ab771. "
                "Gear ratios: raw byte-scan of gdfbin (validated GT86/FR3.5/NSX). "
                "Springs: sdfbin hash 670b57ab block structure. "
                "Wheel centres, wheelbase, track, tire size: vdfm. "
                "Tire Pacejka D/C/E: hdtbin b31155cd/b1033b86/c0eaf58d; B estimated."
            ),
            "confidence": "low",
        },
        "alignment_convention":  "relative_to_centerline",
        "_converter":            "pCARS2->SVJ Converter v0.3",
        "_source_game":          "Project CARS 2",
        "_car_name":             car,
        "_source_files":         src,
        "_scipy_available":      HAS_SCIPY,
        "_known_placeholders": [
            "powertrain.layout (defaults to FR)",
            "vehicle_info.drive_type (defaults to RWD)",
            "chassis.center_of_gravity (height Z not stored; X from weight split when available)",
            "chassis.mass_total (default 1200 kg; no mrdf found)",
            "suspension.*.topology (system_type, links and all hardpoints except "
            "wheel_center are estimates)",
            "suspension.*.brake (disc type/size estimated)",
            "steering.* (all placeholders)",
        ],
        # Notes for sections whose schema forbids extra keys
        "_section_notes": {
            "vehicle_info": ("make/model parsed from the physics file name; "
                             "drive_type defaults to RWD; year omitted when not in the name."),
            "suspension": (
                "spring.rate from sdfbin hash 670b57ab block structure (N/mm→N/m), front/rear "
                "assigned by KC0 camber-gain ratio. damper: ~777 Ns/m pCARS2 reference "
                "constant (road cars); race cars with hash 063ea245 use a per-car value."
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
    if vdfm_data:
        print("  [OK] vdfm: wheel centres read from pcars2_vdfm")
    else:
        print("  [WARN] vdfm: not found — hardpoints will be empty")

    # Game spec sheet: mass, drivetrain, weight distribution, displacement
    mrdf_data = _parse_mrdf(dirs, car) or {}
    if mrdf_data:
        print(f"  [OK] mrdf: {mrdf_data.get('mass_kg')} kg, "
              f"{mrdf_data.get('drive_type')}, {mrdf_data.get('displacement_l')} L")
    else:
        print("  [WARN] mrdf: not found — mass and drivetrain use defaults")

    print("  Building SVJ sections ...")

    # Engine
    engine = build_engine(sf_edf, car)
    if mrdf_data.get("displacement_l"):
        engine["displacement"] = mrdf_data["displacement_l"]

    drive = mrdf_data.get("drive_type")
    # FWD -> FF and AWD -> AWD are certain. For RWD the mrdf doesn't say
    # front/mid/rear engine, so FR stays a default.
    layout = {"FWD": "FF", "AWD": "AWD"}.get(drive, "FR")

    # Gearbox — extract final_drive for differentials
    gearbox_raw  = build_gearbox(sf_gdf, car)
    final_drive  = gearbox_raw.pop("final_drive", 3.600)
    gearbox_svj  = {k: v for k, v in gearbox_raw.items()}   # type, ratios, _note

    suspension = build_suspension(sf_sdf, sf_hdt, car, vdfm=vdfm_data)

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

    metadata = _metadata(
        car=car, extracted_root=extracted_root,
        edfbin=edf_path, cdfbin=cdf_path,
        gdfbin=gdf_path, sdfbin=sdf_path, hdtbin=hdt_path,
        vdfm=(vdfm_data or {}).get("_vdfm_file"),
        mrdf=mrdf_data.get("_mrdf_file"),
    )
    # Drop placeholder entries the spec sheet resolved
    resolved = set()
    if mrdf_data.get("mass_kg"):
        resolved.add("chassis.mass_total")
    if drive:
        resolved.add("vehicle_info.drive_type")
    if drive in ("FWD", "AWD"):
        resolved.add("powertrain.layout")
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
        "chassis":      build_chassis(car, vdfm=vdfm_data, mrdf=mrdf_data),
        "steering":     build_steering(car),
        "suspension":   suspension,
        "powertrain": {
            "layout":  layout,
            "engine":  engine,
            "gearbox": gearbox_svj,
            "differentials": [
                {
                    "id":          f"diff_{diff_loc}",
                    "location":    diff_loc,
                    "type":        "open",
                    "final_drive": round(final_drive, 5),
                    "_note": (
                        "type defaults to 'open'; differential configuration "
                        "not yet decoded from pCARS2 physics files. AWD cars get a "
                        "single driven-axle entry; centre/front diffs are not decoded."
                    ),
                }
            ],
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
    return svj


# ─────────────────────────────────────────────────────────────────────────────
# Output helpers (shared with the GUI)
# ─────────────────────────────────────────────────────────────────────────────

def write_svj(svj: Dict, out_path: str, indent: Optional[int] = 2) -> None:
    """
    Write an SVJ dict to disk, preserving mesh links from a previous run.

    The physics converter can't know about meshes, so it always emits
    assets.meshes = []. Mesh extraction fills that in afterwards; without
    this, re-running the physics conversion would silently drop the link.
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
                        mesh["id"] = re.sub(r"[^a-z0-9_]", "_", mesh["id"].lower())
                svj.setdefault("assets", {})["meshes"] = old_meshes
        except (OSError, ValueError):
            pass   # unreadable old file: just overwrite it
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
  python pcars2_to_svj.py --dir ./extracted --all --outdir ./svj

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
    print("  Project CARS 2 -> SVJ Converter  v0.2")
    print("=" * 62)
    print(f"  Source  : {args.dir}")
    print(f"  Scipy   : {'available' if HAS_SCIPY else 'NOT found (simplified Pacejka)'}")

    indent = None if args.compact else 2

    if args.all:
        cars = list_cars(args.dir)
        if not cars:
            print("No cars found.")
            return
        # Use --outdir directly if given; otherwise default to <dir>/svj_output
        outdir = Path(args.outdir) if args.outdir else Path(args.dir) / "svj_output"
        outdir.mkdir(parents=True, exist_ok=True)
        print(f"\n  Batch mode: {len(cars)} cars -> {outdir}")
        ok = err = invalid = 0
        for car in cars:
            try:
                svj = convert_car(car, args.dir)
                out_path = outdir / f"{car}.svj.json"
                write_svj(svj, str(out_path), indent=indent)
                problems = validate_svj(svj)
                if problems:
                    invalid += 1
                    print(f"  OK {car} -> {out_path.name}  "
                          f"[SCHEMA: {len(problems)} errors, first: {problems[0]}]")
                else:
                    print(f"  OK {car} -> {out_path.name}")
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
