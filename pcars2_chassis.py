#!/usr/bin/env python3
"""
pcars2_chassis.py - the chassis file (.cdfbin) of a Project CARS 2 car
======================================================================

Despite its name the chassis file holds most of a car's setup-able physics:
inertia, centre of gravity, springs, dampers, ride height, camber, toe, caster,
tyre pressure, brake torque and bias, steering ratio and lock, differentials,
aero. Its fields are keyed by 4-byte hashes. The names in pcars2_hashnames.json
come from the Project CARS / AMS2 chassis translation by JDougNY, revised for
AMS2 by GvsE (https://github.com/mikevapour/ams2-modding-docs); the file layout
below follows that document.

Field encoding (the "ending" of a hash token):
  <hash:4> <type> <body...>
    type high nibble  A=float,float  5=int,int  1=int,byte  2=float,byte
                      8=byte,float   0=byte,byte
    type low bits     2 = two values, 3 = three values (the third value's kind
                      is the first body byte: 0 byte, 1 int, 2 float)
  A "Range" is (start, step, count) and its "Setting" an index, so the value in
  force is start + setting x step (a missing setting means 0).
  A one-value token is <prefix><hash:4><value>: prefix 0x20 byte, 0x21 int,
  0x22 float, 0x28 none (value 0). Settings are such tokens, packed after the
  range they belong to.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_HERE = Path(__file__).parent
NAMES: Dict[str, str] = json.loads((_HERE / "pcars2_hashnames.json").read_text(encoding="utf-8"))

_KIND = {0xA: "ff", 0x5: "ii", 0x1: "ib", 0x2: "fb", 0x8: "bf", 0x0: "bb"}


def _read(code: str, b: bytes, o: int) -> Tuple[Any, int]:
    if code == "f":
        return struct.unpack_from("<f", b, o)[0], o + 4
    if code == "i":
        return struct.unpack_from("<i", b, o)[0], o + 4
    return b[o], o + 1


def decode_values(type_code: int, body: bytes) -> Optional[Tuple[List[Any], bytes]]:
    """Values of a multi-value token and the bytes that follow them, or None."""
    n, hi = type_code & 0xF, type_code >> 4
    if n not in (2, 3) or hi not in _KIND:
        return None
    try:
        kinds, o = _KIND[hi], 0
        if n == 3:
            third = {0: "b", 1: "i", 2: "f"}.get(body[0])
            if third is None:
                return None
            kinds, o = kinds + third, 1
        vals = []
        for k in kinds:
            v, o = _read(k, body, o)
            vals.append(v)
        return vals, body[o:]
    except (IndexError, struct.error):
        return None


def settings(rest: bytes) -> Dict[str, Any]:
    """Name -> value of the one-value tokens packed after a range."""
    out: Dict[str, Any] = {}
    o = 0
    while o + 5 <= len(rest) and rest[o] in (0x20, 0x21, 0x22, 0x28):
        p, h = rest[o], rest[o + 1:o + 5].hex()
        o += 5
        v: Any = None
        try:
            if p == 0x20:
                v, o = rest[o], o + 1
            elif p in (0x21, 0x22):
                v = struct.unpack_from("<i" if p == 0x21 else "<f", rest, o)[0]
                o += 4
        except (IndexError, struct.error):
            break
        out[NAMES.get(h, h)] = v
    return out


def _value(vals: List[Any], st: Dict[str, Any], setting_name: str) -> Optional[float]:
    start, step = vals[0], vals[1]
    return start + (st.get(setting_name) or 0) * step


def parse_chassis(sf) -> Dict[str, Any]:
    """
    Decode a ShCBFile of a .cdfbin. Returns inertia, CG, steering, brake and
    alignment values and a list of four corner dicts (document order; the first
    two are the front axle, the last two the rear).
    """
    out: Dict[str, Any] = {"corners": []}
    corner: Optional[Dict[str, Any]] = None
    for r in sf.records:
        name = NAMES.get(r.hash_hex, "")
        if r.type_code in (0x20, 0x21, 0x22, 0x28):
            continue
        d = decode_values(r.type_code, r.body)
        if d is None:
            continue
        vals, rest = d
        st = settings(rest)
        if name == "CamberRange":
            corner = {"camber_deg": _value(vals, st, "CamberSetting")}
            out["corners"].append(corner)
        elif name == "Inertia" and len(vals) == 3:
            out["inertia_game_axes"] = [float(v) for v in vals]      # about X (lateral), Y (up), Z (long.)
        elif name == "CGRearRange":
            out.setdefault("cg_rear_fraction", _value(vals, st, "CGRearSetting"))
        elif name == "FrontToeInRange":
            out["toe_front_deg"] = _value(vals, st, "FrontToeInSetting")
        elif name == "RearToeInRange":
            out["toe_rear_deg"] = _value(vals, st, "RearToeInSetting")
        elif name == "LeftCasterRange":
            out["caster_deg"] = _value(vals, st, "LeftCastersetting")
        elif name == "SteeringRatioRange":
            out["steering_ratio"] = _value(vals, st, "SteeringRatioSetting")
        elif name == "SteeringLockRange":
            out["steering_lock_deg"] = vals[0]
        elif name == "RearBrakeRange":
            out["brake_bias_rear"] = _value(vals, st, "RearBrakeSetting")
        elif name == "BrakePressureRange":
            out["brake_pressure_max"] = _value(vals, st, "BrakePressureSetting")
        elif corner is not None:
            if name == "PressureRange":
                corner["pressure_kpa"] = _value(vals, st, "PressureSetting")
            elif name == "PackerRange":
                corner["spring_mult"] = st.get("SpringMult")
            elif name == "SpringRange":
                corner["spring_Nm"] = _value(vals, st, "SpringSetting")
            elif name == "RideHeightRange":
                corner["ride_height_m"] = _value(vals, st, "RideHeightSetting")
                corner["damper_mult"] = st.get("DamperMult")
            elif name in ("SlowBumpRange", "FastBumpRange", "SlowReboundRange", "FastReboundRange"):
                corner.setdefault("damper", {})[name[:-5]] = _value(vals, st, name[:-5] + "Setting")
            elif name in ("BumpTransitionRange", "ReboundTransitionRange"):
                corner.setdefault("transition", {})[name[:-5]] = _value(vals, st, name[:-5] + "Setting")
            elif name == "BrakeFailure":
                corner["brake_torque_Nm"] = st.get("BrakeTorque")
            elif name == "BrakePadRange":
                corner["brake_disc_inertia"] = st.get("BrakeDiscInertia")
    return out
