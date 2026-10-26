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
                      8=byte,float   0=byte,byte   4=byte,int   6=float,int   9=int,float
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

_KIND = {0xA: "ff", 0x5: "ii", 0x1: "ib", 0x2: "fb", 0x8: "bf", 0x0: "bb", 0x4: "bi", 0x6: "fi", 0x9: "if"}


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
        if kinds[0] == "b" and vals[0] > 127:
            vals[0] -= 256         # a byte start is signed: the GT86 preload 156 reads -100 (-100 + 14 x 10 = 40 Nm)
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


class Token:
    """A hashed multi-value token read from the raw file: name, type byte and the bytes after it."""
    __slots__ = ("hash_hex", "type_code", "body", "offset")

    def __init__(self, hash_hex: str, type_code: int, body: bytes, offset: int):
        self.hash_hex, self.type_code, self.body, self.offset = hash_hex, type_code, body, offset


def iter_tokens(raw: bytes, reach: int = 48):
    """
    Every <0x24><hash:4><type> token whose hash has a documented name, in file order.

    The record splitter of ShCBFile cuts a record wherever a value contains the byte
    0x24 (a 36-step range, a float with that byte), which truncates the field. Reading
    from the raw file by the documented hashes avoids that: a token's body is simply the
    `reach` bytes after its type byte, and decode_values takes what it needs.
    """
    i = raw.find(b"\x24")
    while i != -1:
        if i + 6 <= len(raw):
            h = raw[i + 1:i + 5].hex()
            if h in NAMES:
                yield Token(h, raw[i + 5], raw[i + 6:i + 6 + reach], i)
        i = raw.find(b"\x24", i + 1)


_CORNER_FIELDS = ("CamberRange", "PressureRange", "PackerRange", "SpringRange", "RideHeightRange",
                  "SlowBumpRange", "FastBumpRange", "SlowReboundRange", "FastReboundRange",
                  "BumpTransitionRange", "ReboundTransitionRange", "BrakeFailure", "BrakePadRange")


def parse_chassis(sf) -> Dict[str, Any]:
    """
    Decode a ShCBFile of a .cdfbin. Returns inertia, CG, steering, brake and
    alignment values and a list of four corner dicts (document order; the first
    two are the front axle, the last two the rear).
    """
    out: Dict[str, Any] = {"corners": []}
    # CGHeight is a one-value token (prefix 0x22, a float) that the record splitter leaves
    # inside the body of another record, so look for it in the raw file.
    inv = {v: k for k, v in NAMES.items()}
    raw = getattr(sf, "_buf", b"")
    tag = bytes([0x22]) + bytes.fromhex(inv.get("CGHeight", "")) if inv.get("CGHeight") else b""
    at = raw.find(tag) if tag else -1
    if at >= 0 and at + 9 <= len(raw):
        out["cg_height_m"] = struct.unpack_from("<f", raw, at + 5)[0]
    # Mass (all mass except fuel, driver included) is a one-value token like CGHeight
    mh = inv.get("Mass")
    if mh:
        for prefix, fmt in ((bytes([0x21]), "<i"), (bytes([0x22]), "<f")):
            i = raw.find(prefix + bytes.fromhex(mh))
            if i >= 0 and i + 9 <= len(raw):
                v = struct.unpack_from(fmt, raw, i + 5)[0]
                if v == v and 50 < v < 20000:
                    out["mass_with_driver_kg"] = float(v)
                    break
    out["driveline"] = parse_driveline_settings(raw)
    corner: Optional[Dict[str, Any]] = None
    seen_in_corner: set = set()
    for r in iter_tokens(raw):
        name = NAMES.get(r.hash_hex, "")
        if r.type_code in (0x20, 0x21, 0x22, 0x28):
            continue
        d = decode_values(r.type_code, r.body)
        if d is None:
            continue
        vals, rest = d
        st = settings(rest)
        if name in _CORNER_FIELDS:
            # a corner's fields come together; the next corner starts when one repeats
            if corner is None or name in seen_in_corner:
                corner = {}
                out["corners"].append(corner)
                seen_in_corner = set()
            seen_in_corner.add(name)
            if name == "CamberRange":
                corner["camber_deg"] = _value(vals, st, "CamberSetting")
            elif name == "PressureRange":
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
        elif name == "FrontAntiSwayRange":
            out["arb_front_Nm"] = _value(vals, st, "FrontAntiSwaySetting")
        elif name == "RearAntiSwayRange":
            out["arb_rear_Nm"] = _value(vals, st, "RearAntiSwaySetting")
        elif name == "RearBrakeRange":
            out["brake_bias_rear"] = _value(vals, st, "RearBrakeSetting")
        elif name == "BrakePressureRange":
            out["brake_pressure_max"] = _value(vals, st, "BrakePressureSetting")
    return out


_DIFF_KINDS = ("Spool", "GearedLsd", "ClutchLsd", "ViscousLsd", "Ratcheting")


def _first_setting(st: Dict[str, Any]) -> Any:
    return next((v for k, v in st.items() if "etting" in k), None)


def parse_diffs(sf) -> Dict[str, Dict[str, Any]]:
    """
    Differentials of the chassis file, by "front" / "center" / "rear".

    A type is on when its Option setting is 1 (a car can have several on; they
    are listed in `active`). Parameters are start + setting x step of the
    matching ranges: geared bias ratios (power, coast), clutch preload (Nm),
    ramp angles (degrees) and clutch count, viscous lock.
    """
    out: Dict[str, Dict[str, Any]] = {}
    for r in iter_tokens(getattr(sf, "_buf", b"")):
        name = NAMES.get(r.hash_hex, "")
        if not name.startswith("Diff") or r.type_code in (0x20, 0x21, 0x22, 0x28):
            continue
        axle = next((a for a in ("Front", "Center", "Rear") if name.startswith("Diff" + a)), None)
        d = decode_values(r.type_code, r.body)
        if axle is None or d is None:
            continue
        vals, rest = d
        st = settings(rest)
        sub = name[4 + len(axle):]
        slot = out.setdefault(axle.lower(), {"active": []})
        if sub.endswith("Option") and sub[:-6] in _DIFF_KINDS:
            if _first_setting(st):
                slot["active"].append(sub[:-6])
        elif sub == "BiasRatioPowerRange":
            slot["bias_power"] = vals[0] + (_first_setting(st) or 0) * vals[1]
        elif sub == "BiasRatioCoastRange":
            slot["bias_coast"] = vals[0] + (_first_setting(st) or 0) * vals[1]
        elif sub == "ClutchLsd":
            slot["clutches"] = vals[0] + (_first_setting(st) or 0) * vals[1]
        elif sub == "ClutchLsdPreloadRange":
            slot["preload_Nm"] = vals[0] + (_first_setting(st) or 0) * vals[1]
        elif sub == "ClutchLsdPowerRampRange":
            slot["ramp_power_deg"] = vals[0] + (_first_setting(st) or 0) * vals[1]
        elif sub == "ClutchLsdCoastRampRange":
            slot["ramp_coast_deg"] = vals[0] + (_first_setting(st) or 0) * vals[1]
        elif sub == "ViscousLockRange":
            slot["viscous_lock"] = vals[0] + (_first_setting(st) or 0) * vals[1]
    return out


_AERO_PREFIXES = ("FW", "RW", "Body", "Diffuser", "Radiator", "BrakeDuct")


def parse_aero(sf) -> Dict[str, Any]:
    """
    The aero fields of the chassis file by their documented names: body, front
    and rear wing, diffuser, radiator and brake ducts. Vector fields come as
    lists (a missing component of a vector is 0), one-value fields as numbers.

    The game keeps forces as coefficients of speed squared (the Lexus LFA's
    BodyFore z of 0.435 is about half the air density times its drag area), and
    computes the forward drag at run time from BodyDragBase and the ride
    heights, so BodyFore and RWFore store 0 there. Names ending in Setting or
    Range are setup choices, not coefficients, and are left out.
    """
    out: Dict[str, Any] = {}
    for r in iter_tokens(getattr(sf, "_buf", b"")):
        name = NAMES.get(r.hash_hex, "")
        if not name.startswith(_AERO_PREFIXES) or name.endswith(("Setting", "Range")) and not name.startswith(("Radiator", "BrakeDuct", "FWRange", "RWRange")):
            continue
        if r.type_code in (0x21, 0x22) and len(r.body) >= 4:
            v = struct.unpack_from("<i" if r.type_code == 0x21 and False else "<f", r.body, 0)[0]
            if v == v and abs(v) < 1e9:
                out.setdefault(name, round(float(v), 6))
            continue
        d = decode_values(r.type_code, r.body)
        if d:
            vals = [round(float(v), 6) for v in d[0] if isinstance(v, (int, float)) and v == v]
            if vals:
                out.setdefault(name, vals)
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Transmission: the gearbox file lists ratios, the chassis file picks which ones
# ─────────────────────────────────────────────────────────────────────────────

_GEAR_TOKEN = bytes.fromhex("249d58f964")      # a gear: (driver teeth, driven teeth)
_BEVEL_TOKEN = bytes.fromhex("243c5bef62")     # bevel gear: (driver, driven)
_GEARS_END = bytes.fromhex("e09765b7af")       # end of the primary gears; final-drive gears follow

_GEAR_SETTINGS = ("GearOneSetting", "GearTwoSetting", "GearThreeSetting", "GearFourSetting",
                  "GearFiveSetting", "GearSixSetting", "GearSevenSetting", "GearEightSetting",
                  "GearNineSetting")
_DRIVELINE_FIELDS = ("ForwardGears", "ReverseGearSetting", "FinalDriveSetting", "SemiAutomatic",
                     "UpshiftDelay", "UpshiftClutchTime", "DownshiftDelay", "DownshiftClutchTime",
                     "DownshiftBlipThrottle", "ClutchEngageRate", "ClutchInertia", "ClutchTorque",
                     "ClutchFriction", "BaulkTorque") + _GEAR_SETTINGS


def parse_driveline_settings(raw: bytes) -> Dict[str, float]:
    """
    The chassis file's gear and clutch fields, by name. A one-value token is
    <prefix><hash><value> with prefix 0x20 byte, 0x21 int, 0x22 float, or 0x28 for a
    value of zero (no value stored); a field that is absent is left out.
    """
    inv = {v: k for k, v in NAMES.items()}
    out: Dict[str, float] = {}
    for name in _DRIVELINE_FIELDS:
        h = inv.get(name)
        if not h:
            continue
        hb = bytes.fromhex(h)
        for prefix, fmt in ((0x20, "<B"), (0x21, "<i"), (0x22, "<f"), (0x28, None)):
            i = raw.find(bytes([prefix]) + hb)
            if i >= 0:
                try:
                    out[name] = 0.0 if fmt is None else float(struct.unpack_from(fmt, raw, i + 5)[0])
                except struct.error:
                    break
                break
    return out


def parse_gearbox_lists(raw: bytes) -> Optional[Dict[str, Any]]:
    """
    The ratios a gearbox file offers: gear pairs before the end marker, the bevel pair,
    and the final-drive pairs after it. A pair (driver, driven) is a ratio driven / driver.
    Road cars list one pair per gear; race cars list every ratio they can be fitted with,
    sorted from the shortest, and the chassis file's gear settings pick among them.
    """
    def pair(i: int, token: bytes) -> Optional[Tuple[float, float]]:
        t = raw[i + len(token)] if i + len(token) < len(raw) else None
        d = decode_values(t, raw[i + len(token) + 1:i + len(token) + 17]) if t is not None else None
        if d and len(d[0]) >= 2 and d[0][0] and d[0][1]:
            return float(d[0][0]), float(d[0][1])
        return None

    end = raw.find(_GEARS_END)
    gears: List[float] = []
    finals: List[float] = []
    i = raw.find(_GEAR_TOKEN)
    while i != -1:
        p = pair(i, _GEAR_TOKEN)
        if p:
            (gears if (end == -1 or i < end) else finals).append(p[1] / p[0])
        i = raw.find(_GEAR_TOKEN, i + 1)
    bevel = 1.0
    b = raw.find(_BEVEL_TOKEN)
    if b != -1:
        p = pair(b, _BEVEL_TOKEN)
        if p:
            bevel = p[1] / p[0]
    return {"gears": gears, "finals": finals, "bevel": bevel} if gears else None


def select_transmission(lists: Optional[Dict[str, Any]], dl: Dict[str, float]) -> Optional[Dict[str, Any]]:
    """
    Forward ratios and total final drive picked by the chassis settings, or None when the
    settings do not fit the lists or give no sensible gearbox.

    The settings index the gear list either from its start (road cars and most race
    cars) or from its end (a few race cars list the ratios from the longest gear); the
    reading whose ratios fall from gear to gear is taken. A gear with no stored setting
    (the dictionary names gears one to seven) makes the whole selection fail. Reverse
    is not returned: its setting indexes a list whose layout is not established.
    """
    if not lists or not dl.get("ForwardGears"):
        return None
    n = int(dl["ForwardGears"])
    if not 1 <= n <= len(_GEAR_SETTINGS):
        return None
    gears = lists["gears"]
    if any(name not in dl for name in _GEAR_SETTINGS[:n]):
        # The dictionary names gears one to seven: a car with more gears has no stored
        # setting for the rest. When the file offers exactly one ratio per gear and the
        # settings that are stored count up from zero, the list is the gearbox.
        known = [int(dl[name]) for name in _GEAR_SETTINGS[:n] if name in dl]
        if len(gears) != n or known != list(range(len(known))):
            return None
        idx = list(range(n))
    else:
        idx = [int(dl[name]) for name in _GEAR_SETTINGS[:n]]
    if any(not 0 <= k < len(gears) for k in idx):
        return None
    ratios = None
    order = "forward"
    by_size = sorted(gears, reverse=True)
    for label, pick in (("forward", lambda k: gears[k]), ("reversed", lambda k: gears[len(gears) - 1 - k]),
                        ("sorted", lambda k: by_size[k])):
        r = [pick(k) for k in idx]
        if all(a > b for a, b in zip(r, r[1:])) or (n == 1):
            ratios, order = r, label
            break
    if ratios is None:
        return None
    out: Dict[str, Any] = {"ratios": [round(r, 5) for r in ratios], "gear_count": n, "index_order": order}
    finals = lists["finals"]
    if finals:
        fi = int(dl.get("FinalDriveSetting", 0))
        if order == "sorted":
            finals = sorted(finals, reverse=True)
        if order == "reversed":
            fi = len(finals) - 1 - fi if 0 <= fi < len(finals) else fi
        if fi >= len(finals):          # some files store the count of options: the last one
            fi, out["final_drive_clamped"] = len(finals) - 1, True
        if 0 <= fi < len(finals):
            out["final_drive"] = round(finals[fi] * lists["bevel"], 5)
            out["final_drive_options"] = [round(f * lists["bevel"], 3) for f in finals]
    return out
