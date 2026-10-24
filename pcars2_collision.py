#!/usr/bin/env python3
"""
pcars2_collision.py - Project CARS 2 collision XML -> SVJ bodies, hinges and proxies
=====================================================================================

Each car's vehicle archive carries three plain-text XML files in
vehicles/physics/collision/:

  <car>.xml      one convex hull per part (chassis, bonnet, boot, bumpers, ...)
                 with the game's mass for the part
  <car>.joi.xml  hinge descriptors: anchor, axis and angle limits of every part
                 that can come loose or open

This module parses them and writes, into an SVJ document:

  * chassis.collision[]       a box proxy per chassis hull (spec section 28)
  * chassis.mass_bodies[]     one body per hinged part, with the game's mass and
                              a box-estimated inertia tensor (spec section 24.2)
  * constraints[]             one revolute joint per hinged part (section 24.4)
  * poses[]                   one "<part>_open" pose per hinged part (section 24.7)
  * collision_ignore[]        pairs whose box proxies overlap only because a box
                              is looser than the hull it replaces

Hulls become axis-aligned boxes in vehicle axes: SVJ proxies are sphere, box or
cylinder only. Masses, inertias and the closed-pose angle are flagged estimates.

Game frame: X = left, Y = up, Z = rear, origin on the ground. SVJ frame (SAE
J670): X forward, Y right, Z down, origin at the front-axle midpoint on the ground:

    SAE = (-Z - front_axle_z, -X, -Y)

That mapping mirrors the space (determinant -1). Rotation senses are therefore
not carried over from the game: each hinge axis is oriented so that a positive
angle lifts the part, which is the opening direction.
"""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

Vec = List[float]

# Game part name -> SVJ canonical part name (docs/naming_convention.md).
_PART_NAMES = {
    "bonnet": "hood", "hood": "hood",
    "boot": "trunk", "trunk": "trunk",
    "tailgate": "tailgate", "hatch": "tailgate",
}
_SIDE_NAMES = {"l": "fl", "r": "fr", "lf": "fl", "rf": "fr", "lr": "rl", "rr": "rr"}


# ─────────────────────────────────────────────────────────────────────────────
# Parsing
# ─────────────────────────────────────────────────────────────────────────────

def _vec(s: Optional[str]) -> Optional[Vec]:
    try:
        v = [float(x) for x in s.split(";")]
    except (AttributeError, ValueError):
        return None
    return v if len(v) == 3 else None


def _prop(block: str, name: str) -> Optional[str]:
    m = re.search(r'<prop name="%s" data="([^"]*)"' % re.escape(name), block)
    return m.group(1) if m else None


def _blocks(text: str, cls: str) -> List[str]:
    return re.findall(r'<data class="%s".*?(?=<data class="%s"|</Reflection>)' % (cls, cls),
                      text, re.S)


_PART_START = {
    "chassis", "cha2", "chaswing", "bonnet", "hood", "boot", "trunk", "tailgate", "hatch",
    "bumper", "bump", "wing", "rwing", "fwing", "rearwing", "rearwng", "frontwing", "nose",
    "door", "mirror", "panel", "driver", "exhaust", "intake", "ladx", "wheels",
    "susp", "suspf", "suspr",
}


def part_name(raw: str) -> str:
    """
    'COLLISION_CONVEX_TOY_GT86_13_BUMPER_F_LODA' -> 'bumper_f' (SVJ canonical name when listed).
    The part starts at the first token that names a part (the car prefix before it varies
    from 'TOY_GT86_13' to 'DALLARA_DW12_CHEVY_ROAD'); without one, the last token is used.
    """
    n = re.sub(r"_LOD[A-D0-9]$", "", raw.upper().replace("COLLISION_CONVEX_", "")).lower()
    n = re.sub(r"_kit\d+", "", n)
    toks = n.split("_")
    idx = next((i for i, t in enumerate(toks) if t in _PART_START or re.fullmatch(r"chas\d*", t)),
               len(toks) - 1)
    toks = toks[idx:]
    if toks[0] == "bump":
        toks[0] = "bumper"
    if len(toks) == 1 and toks[0] in _PART_NAMES:
        return _PART_NAMES[toks[0]]
    if toks[0] == "door" and len(toks) == 2 and toks[1] in _SIDE_NAMES:
        return f"door_{_SIDE_NAMES[toks[1]]}"
    return "_".join(toks)


def parse_collision(directory: str, car: str) -> Optional[Dict[str, Any]]:
    """
    Read <directory>/<car>.xml and <car>.joi.xml (game frame, untouched).
    Returns {"shapes": [...], "joints": [...]} or None when the hull file is absent.
    """
    d = Path(directory)
    xml = d / f"{car}.xml"
    if not xml.is_file():
        return None
    shapes = []
    for blk in _blocks(xml.read_text(encoding="utf-8", errors="replace"), "ShapeDesc"):
        name = _prop(blk, "Name")
        verts = [v for v in (_vec(x) for x in re.findall(r'<prop name="Vertex" data="([^"]*)"', blk)) if v]
        if not name or len(verts) < 4:
            continue
        mass = _prop(blk, "Mass")
        shapes.append({"name": name, "part": part_name(name),
                       "mass": float(mass) if mass else None, "vertices": verts})
    joints = []
    joi = d / f"{car}.joi.xml"
    if joi.is_file():
        for blk in _blocks(joi.read_text(encoding="utf-8", errors="replace"), "JointDesc"):
            anchor, axis = _vec(_prop(blk, "Anchor1")), _vec(_prop(blk, "Axis1"))
            if not (anchor and axis):
                continue
            joints.append({
                "name": _prop(blk, "Name"), "type": int(_prop(blk, "Type") or 0),
                "object1": _prop(blk, "Object1"), "object2": _prop(blk, "Object2"),
                "anchor": anchor, "axis": axis,
                "limits_enabled": _prop(blk, "LimitsEnabled") == "true",
                "limit_min": float(_prop(blk, "LimitMin") or 0.0),
                "limit_max": float(_prop(blk, "LimitMax") or 0.0),
                "breakable": _prop(blk, "Breakable") == "true",
            })
    return {"shapes": shapes, "joints": joints} if shapes else None


def parse_collision_dir(root: str) -> Optional[Dict[str, Any]]:
    """Find the collision folder under an extracted vehicle archive and parse it."""
    for d in sorted(Path(root).rglob("collision")):
        if not d.is_dir():
            continue
        for xml in sorted(d.glob("*.xml")):
            if xml.name.endswith((".joi.xml", ".sys.xml")):
                continue
            data = parse_collision(str(d), xml.stem)
            if data:
                return data
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Geometry
# ─────────────────────────────────────────────────────────────────────────────

def _to_sae(v: Vec, front_z: float) -> Vec:
    return [-v[2] - front_z, -v[0], -v[1]]


def _dir_to_sae(v: Vec) -> Vec:
    return [-v[2], -v[0], -v[1]]


def _norm(v: Vec) -> Vec:
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]


def _cross(a: Vec, b: Vec) -> Vec:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def quat_z_to(axis: Vec) -> List[float]:
    """Unit quaternion [x, y, z, w] turning +Z onto `axis` (the joint frame convention)."""
    a = _norm(axis)
    c = a[2]
    if c < -0.999999:
        return [1.0, 0.0, 0.0, 0.0]
    x, y, z = -a[1], a[0], 0.0              # z-hat cross a
    w = 1.0 + c
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / n, y / n, z / n, w / n]


def _box(verts: List[Vec]) -> Tuple[Vec, Vec]:
    lo = [min(v[i] for v in verts) for i in range(3)]
    hi = [max(v[i] for v in verts) for i in range(3)]
    return [(l + h) / 2 for l, h in zip(lo, hi)], [h - l for l, h in zip(lo, hi)]


def _box_inertia(mass: float, size: Vec) -> Dict[str, float]:
    sx, sy, sz = size
    return {"Ixx": round(mass / 12 * (sy * sy + sz * sz), 4),
            "Iyy": round(mass / 12 * (sx * sx + sz * sz), 4),
            "Izz": round(mass / 12 * (sx * sx + sy * sy), 4),
            "Ixy": 0.0, "Ixz": 0.0, "Iyz": 0.0}


def _r(v: Vec, n: int = 5) -> Vec:
    return [round(x, n) for x in v]


# ─────────────────────────────────────────────────────────────────────────────
# SVJ output
# ─────────────────────────────────────────────────────────────────────────────

def _hinge(part: str, joints: List[Dict], centroid: Vec, front_z: float) -> Optional[Dict]:
    """One revolute joint for a part from its (one or two) game joint descriptors."""
    if not joints:
        return None
    j0 = joints[0]
    if len(joints) >= 2:
        # a part held at two anchors hinges about the line through them
        a, b = (_to_sae(j["anchor"], front_z) for j in joints[:2])
        pos = [(x + y) / 2 for x, y in zip(a, b)]
        axis = [y - x for x, y in zip(a, b)]
        if sum(x * x for x in axis) < 1e-8:
            axis = _dir_to_sae(j0["axis"])
    else:
        pos = _to_sae(j0["anchor"], front_z)
        axis = _dir_to_sae(j0["axis"])
    axis = _norm(axis)
    # positive angle must lift the part (Z is down, so upward velocity has z < 0)
    lift = _cross(axis, [c - p for c, p in zip(centroid, pos)])[2]
    if lift > 0:
        axis = [-x for x in axis]

    lo, hi = j0["limit_min"], j0["limit_max"]
    if not j0["limits_enabled"]:
        span = None
    elif lo <= 0 <= hi:
        span = max(abs(lo), abs(hi))
    else:
        rest = lo if abs(lo) < abs(hi) else hi          # closed pose = limit nearest zero
        span = abs((hi if rest == lo else lo) - rest)
    out = {"axis": axis, "position": pos}
    if span:
        out["span_rad"] = math.radians(span)
    return out


def apply_collision(svj: Dict, data: Dict[str, Any]) -> Dict[str, int]:
    """
    Add the collision-derived bodies, hinges, poses and proxies to `svj`
    (idempotent: items from an earlier pass are replaced). Returns counts.
    """
    front_z = svj.get("_metadata", {}).get("_vhf_front_axle_z")
    if front_z is None:
        raise ValueError("document lacks _metadata._vhf_front_axle_z (needs the vdfm wheel centres)")
    ch = svj.setdefault("chassis", {})
    SRC = "pcars2_collision"
    _drop(svj, SRC)

    by_object: Dict[str, List[Dict]] = {}
    for j in data.get("joints", []):
        by_object.setdefault(j["object2"], []).append(j)

    bodies = ch.setdefault("mass_bodies", [])
    existing = {b.get("id") for b in bodies if isinstance(b, dict)}
    constraints = svj.setdefault("constraints", [])
    poses = svj.setdefault("poses", [])
    chassis_proxies: List[Dict] = []
    moving: List[str] = []
    counts = {"bodies": 0, "joints": 0, "proxies": 0}

    for sh in data["shapes"]:
        verts = [_to_sae(v, front_z) for v in sh["vertices"]]
        centre, size = _box(verts)
        size = [max(s, 0.01) for s in size]
        proxy = {"type": "box", "position": _r(centre), "size": _r(size),
                 "source_node": sh["name"], "x_source": SRC}
        part = part_name(sh["name"])
        joints = by_object.get(sh["name"], [])
        is_chassis = re.fullmatch(r"chas\w*\d*", part) is not None
        if is_chassis or not joints:
            proxy["id"] = f"{part}_hull"
            chassis_proxies.append(proxy)
            counts["proxies"] += 1
            continue

        centroid = [sum(v[i] for v in verts) / len(verts) for i in range(3)]
        hinge = _hinge(part, joints, centroid, front_z)
        mass = sh["mass"] if sh["mass"] and sh["mass"] > 0 else None
        if hinge is None or mass is None or part in existing:
            proxy["id"] = f"{part}_hull"
            chassis_proxies.append(proxy)
            counts["proxies"] += 1
            continue

        proxy["id"] = f"{part}_hull"
        bodies.append({
            "id": part, "name": part.replace("_", " ").title(), "category": "structural",
            "parent": "chassis", "mass": round(mass, 3), "position": _r(centre),
            "inertia": _box_inertia(mass, size), "collision": [proxy],
            "x_source": SRC, "_est": True,
            "_note": ("mass from the game's collision shape; inertia of a uniform box the size "
                      "of the hull; position is the hull box centre"),
        })
        frame = {"position": _r(hinge["position"]), "orientation": [round(q, 7) for q in quat_z_to(hinge["axis"])]}
        jd = {
            "id": f"hinge_{part}", "type": "joint", "joint_type": "revolute",
            "body_a": "chassis", "body_b": part,
            "frame_a": dict(frame), "frame_b": dict(frame),
            "x_source": SRC, "_est": True,
        }
        if "span_rad" in hinge:
            jd["range"] = {"min": 0.0, "max": round(hinge["span_rad"], 5)}
        jd["_note"] = ("anchor and limits from the game's joint descriptor; zero is the closed "
                       "(authored) pose, the axis is oriented so that a positive angle lifts the part")
        constraints.append(jd)
        moving.append(part)
        counts["bodies"] += 1
        counts["joints"] += 1
        counts["proxies"] += 1
        if "span_rad" in hinge:
            poses.append({"id": f"{part}_open", "values": {f"hinge_{part}": round(hinge["span_rad"], 5)},
                          "x_source": SRC})

    if chassis_proxies:
        ch["collision"] = [p for p in ch.get("collision", []) if p.get("x_source") != SRC] + chassis_proxies
    if not constraints:
        svj.pop("constraints")
    if not poses:
        svj.pop("poses")
    counts["ignored_pairs"] = _ignore_loose_overlaps(svj)
    return counts


def _ignore_loose_overlaps(svj: Dict) -> int:
    """
    List, in collision_ignore, the body pairs whose box proxies overlap in the
    closed pose: the hulls they stand for fit together, the looser boxes do not.
    """
    try:
        import sys
        tools = Path(__file__).parent / "svj_spec" / "SVJ-standard-vehicle-json-main" / "tools"
        if str(tools) not in sys.path:
            sys.path.insert(0, str(tools))
        import collision_check
    except ImportError:
        return 0
    pairs = {tuple(sorted((c["body_a"], c["body_b"]))) for c in collision_check.clashes(svj, None, -0.001)}
    have = {tuple(sorted(p)) for p in svj.get("collision_ignore", [])}
    new = sorted(pairs - have)
    if new:
        svj.setdefault("collision_ignore", []).extend([list(p) for p in new])
    return len(new)


def _drop(svj: Dict, src: str) -> None:
    """Remove everything an earlier apply_collision pass added."""
    ch = svj.get("chassis", {})
    mine = {b.get("id") for b in ch.get("mass_bodies", []) if isinstance(b, dict) and b.get("x_source") == src}
    if isinstance(svj.get("collision_ignore"), list):
        svj["collision_ignore"] = [p for p in svj["collision_ignore"] if not mine & set(p)]
        if not svj["collision_ignore"]:
            svj.pop("collision_ignore")
    if isinstance(ch.get("mass_bodies"), list):
        ch["mass_bodies"] = [b for b in ch["mass_bodies"] if not (isinstance(b, dict) and b.get("x_source") == src)]
    if isinstance(ch.get("collision"), list):
        ch["collision"] = [p for p in ch["collision"] if p.get("x_source") != src]
        if not ch["collision"]:
            ch.pop("collision")
    for key in ("constraints", "poses"):
        if isinstance(svj.get(key), list):
            svj[key] = [c for c in svj[key] if not (isinstance(c, dict) and c.get("x_source") == src)]
