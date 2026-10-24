"""
SVJ Suspension Migration (to v0.100)
====================================
Converts the corner-based `suspension` section into the unified model of spec §25:
bodies (`chassis.mass_bodies`), joints / springs / dampers (`constraints`), a thin
`wheel_stations[]` list and `anti_roll_bars[]`. One-way and lossy only where the old data
is: what it does not say is noted, never invented. Everything the tool creates is marked
`x_derived: true` so the checker stays lenient until an author refines it.

What happens to each part of a corner:

  upright / axle body, links, wheel   -> bodies with their canonical part names, meaning in `description`
  hardpoints                          -> markers on the wheel carrier (nothing is lost)
  link joints (explicit or inferred)  -> `joint` constraints with coincident inline frames
  tie rods                            -> spherical joints to a `rack` body on a lateral slide
  leaf spring link                    -> body: front eye revolute, rear slipper translational, seat fixed to the axle
  spring, damper                      -> `spring`, `damper`. With mounts: at the mounts, rates converted by the motion
                                         ratio (given, else estimated from the geometry and flagged). Without mounts:
                                         `reference: "wheel"` elements at the wheel (rates kept as quoted)
  bump / rebound stop                 -> wheel-level `spring` with a force curve that is zero until the stop engages
  arb                                 -> `anti_roll_bars[]` (data: the old files carry no bar geometry)
  wheel, tire, brake, alignment, ...  -> `wheel_stations[]`
  system_type                         -> `wheel_stations[].topology_hint`

svj-py ships an identical copy as svj/migrate_suspension.py — keep the two files in sync
(tests/test_vehicle.py::test_migration_in_sync enforces it).

Usage:
    python tools/migrate_suspension.py <in.svj.json> [out.svj.json]    # writes the migrated file (default: prints a summary)
"""

import copy
import json
import math
import re
import sys

from . import naming as cc
from .multibody_check import infer_joint_types
from .suspension_view import _MAP_TYPE, _PREFIX, _hardpoint, _mean, _norm, _pos, _slug, _stations, _sub, quat_z_to

VERSION = "0.100"
UP = [0.0, 0.0, -1.0]                      # SAE: Z is down
PART_TEXT = {
    "upright": "Upright / knuckle / hub carrier", "hub": "Rotating hub", "upper_wishbone": "Upper A-arm", "lower_wishbone": "Lower A-arm",
    "trailing_arm": "Trailing arm", "semi_trailing_arm": "Semi-trailing arm", "leading_arm": "Leading arm", "tie_rod": "Steering tie rod / track rod",
    "toe_link": "Non-steered toe control link", "camber_link": "Camber control link", "drag_link": "Steering drag link", "pushrod": "Inboard-suspension actuation rod",
    "pullrod": "Inboard-suspension actuation rod", "rocker": "Bellcrank", "strut": "MacPherson / Chapman strut body", "torque_rod": "Torque rod on a beam axle",
    "panhard_rod": "Panhard bar", "axle_body": "Beam axle, housing or de Dion tube", "wheel": "Rim + tyre", "rack": "Steering rack / gear housing",
    "spring": "Spring", "leaf_spring": "Leaf spring pack", "damper": "Damper", "upper_link_front": "Upper multi-link arm, front", "upper_link_rear": "Upper multi-link arm, rear",
    "lower_link_front": "Lower multi-link arm, front", "lower_link_rear": "Lower multi-link arm, rear", "watts_link_rod": "Watt's linkage rod",
    "watts_pivot": "Watt's linkage pivot",
}


def _title(s):
    return " ".join(w.capitalize() if not re.fullmatch(r"[A-Z0-9]+", w) else w for w in str(s).replace("_", " ").split())


def _naming(raw_name, station):
    """(id base, display name, description) for a part: the canonical name when it is a known part."""
    base = cc.canonical_part_id(raw_name) or _slug(raw_name)
    plain = re.sub(r"_(fl|fr|rl|rr|l|r)$", "", base)
    desc = PART_TEXT.get(plain)
    return base, f"{_title(plain)} {station}", desc


def _axle_of(st):
    m = re.fullmatch(r"(FL|FR|RL|RR)|A(\d{1,2})[LRC]", st)
    if not m:
        return None, None
    if m.group(1):
        return ("A1" if m.group(1) in ("FL", "FR") else "A2"), m.group(1)[1]
    return "A" + m.group(2), st[-1]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _seg_dist(p, a, b):
    ab = _sub(b, a)
    n = _dot(ab, ab)
    t = 0.0 if n < 1e-12 else max(0.0, min(1.0, _dot(_sub(p, a), ab) / n))
    return math.dist(p, [a[i] + t * ab[i] for i in range(3)])


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def estimate_motion_ratio(chassis_pt, body_pt, hinge, wheel_centre):
    """First-order motion ratio of a spring / damper whose outboard end sits at `body_pt`.

    `hinge` is (point, axis) of the link that carries the mount, or None when the mount rides on the wheel carrier
    (strut, axle, upright), which moves with the wheel. MR = (element length change) / (vertical wheel travel);
    approximate, because SVJ does not solve the suspension loop. Returns None when it cannot be computed.
    """
    e = _norm(_sub(body_pt, chassis_pt))
    if e is None:
        return None
    if hinge is None:
        mr = abs(_dot(e, UP))
    else:
        p0, axis = hinge
        vm, vw = _cross(axis, _sub(body_pt, p0)), _cross(axis, _sub(wheel_centre, p0))
        wz = _dot(vw, UP)
        if abs(wz) < 1e-9:
            return None
        mr = abs(_dot(vm, e) / wz)
    return round(mr, 4) if 0.05 <= mr <= 2.0 else None


def _convert_curve(curve, scale_x, scale_y):
    return [[round(p[0] * scale_x, 6), round(p[1] * scale_y, 6)] for p in curve]


_LINK_USED = {"name", "type", "inboard_points", "outboard_ref", "joint_type", "inboard_joint_type", "body_ref", "mass", "cg_position",
              "visual", "bushings", "outboard_bushing"}


def _keep_link_data(body, ln, pts, notes_here):
    """Carry everything the old link stated onto its body, so the migration loses nothing: inertia and other body data,
    the pickups beyond the two an arm has (as markers), and the link type and outboard reference (x_legacy_link)."""
    for k, v in ln.items():
        if k in _LINK_USED:
            continue
        if k == "inertia" and isinstance(v, (dict, list)) and not body.get("inertia"):
            body["inertia"] = copy.deepcopy(v)               # a link's scalar inertia is no tensor: it goes to x_legacy_link below
        elif k == "inertia" and isinstance(v, (dict, list)):
            body.setdefault("x_legacy_link", {})[k] = copy.deepcopy(v)
        elif k == "inertia":
            body.setdefault("x_legacy_link", {})[k] = v
        elif k in body and body[k] != v:
            body.setdefault("x_legacy_link", {})[k] = copy.deepcopy(v)
        else:
            body.setdefault(k, copy.deepcopy(v))
    meta = {k: ln[k] for k in ("type", "outboard_ref") if ln.get(k) is not None}
    if meta:
        body.setdefault("x_legacy_link", {}).update(meta)
    for i, pt in enumerate(pts[2:], start=3):
        body.setdefault("markers", {}).setdefault(f"pickup_{i}", {"position": [round(x, 6) for x in pt]})
        notes_here.append(f"link '{ln.get('name')}': inboard point {i} kept as marker pickup_{i} (no joint)")


def migrate(doc):
    """Return (migrated document, notes). The input is not modified."""
    out = copy.deepcopy(doc)
    susp = out.pop("suspension", None)
    notes = []
    if not isinstance(susp, dict):
        return out, ["no suspension section: nothing to migrate"]
    ch = out.setdefault("chassis", {})
    bodies = ch.setdefault("mass_bodies", [])
    body_by_id = {b.get("id"): b for b in bodies if isinstance(b, dict)}
    cons = out.setdefault("constraints", [])
    con_ids = {c.get("id") for c in cons if isinstance(c, dict)}
    stations_out, bars = [], {}
    racks = {}
    rack_joint = {}
    axle_carrier = {}

    def add_body(b):
        if b["id"] in body_by_id:
            return body_by_id[b["id"]]
        b["x_derived"] = True
        bodies.append(b)
        body_by_id[b["id"]] = b
        return b

    def add_con(c):
        base, n = c["id"], 2
        while c["id"] in con_ids:
            c["id"] = f"{base}_{n}"
            n += 1
        con_ids.add(c["id"])
        c["x_derived"] = True
        cons.append(c)
        return c

    def frame(pos, axis=None):
        return {"position": [round(x, 6) for x in pos], "orientation": [round(x, 7) for x in quat_z_to(axis)] if axis else [0.0, 0.0, 0.0, 1.0]}

    def joint(jid, jt, a, b, pos, axis=None, extra=None):
        f = frame(pos, axis)
        c = {"id": jid, "type": "joint", "joint_type": jt, "body_a": a, "frame_a": copy.deepcopy(f), "body_b": b, "frame_b": f}
        if extra:
            c.update(extra)
        return add_con(c)

    for st in _stations(susp):
        corner, topo = susp[st], susp[st]["topology"]
        sfx = _slug(st)
        sysT = topo.get("system_type")
        up = topo.get("upright") if isinstance(topo.get("upright"), dict) else {}
        hps = {k: _pos(v) for k, v in (up.get("hardpoints") or {}).items() if _pos(v)}
        axle = topo.get("axle_body") if isinstance(topo.get("axle_body"), dict) and topo["axle_body"].get("id") else None
        wc = hps.get("wheel_center")
        station_visual = corner.get("visual")
        notes_here = []

        # ── wheel carrier ─────────────────────────────────────────────────────────
        if axle:
            if axle["id"] in axle_carrier:
                carrier = axle_carrier[axle["id"]]    # an axle body shared by two stations is one body
            else:
                carrier = _slug(axle["id"])
                canon = cc.canonical_part_id(axle["id"])
                if canon and canon != carrier and carrier not in body_by_id and canon not in body_by_id:
                    carrier = canon                  # a known part keeps its canonical name (the original stays in `name`)
                axle_carrier[axle["id"]] = carrier
            base, label, desc = _naming("axle_body", "")
            if carrier not in body_by_id:
                b = {"id": carrier, "name": f"Axle body {axle['id']}", "description": PART_TEXT["axle_body"], "category": "structural",
                     "mass": axle.get("mass", 0.0), "position": _pos(axle.get("position")) or wc or [0.0, 0.0, 0.0], "markers": {}}
                if axle.get("inertia"):
                    b["inertia"] = copy.deepcopy(axle["inertia"])
                legacy = {k: v for k, v in axle.items() if k not in ("id", "mass", "position", "inertia", "visual")}
                if legacy:
                    b["x_legacy_axle_body"] = legacy
                    notes_here.append(f"axle body '{axle['id']}': {sorted(legacy)} kept under x_legacy_axle_body (no unified counterpart yet)")
                if axle.get("visual"):
                    b["visual"] = copy.deepcopy(axle["visual"])
                add_body(b)
            cb = body_by_id[carrier]
            mark_name = lambda k: f"{k}_{sfx}"
        else:
            raw = up.get("body_ref") or up.get("id") or f"upright_{sfx}"
            carrier = raw if up.get("body_ref") else _slug(raw)
            if carrier not in body_by_id:
                base, label, desc = _naming(re.sub(r"_(fl|fr|rl|rr|a\d{1,2}[lrc])$", "", _slug(raw)) or "upright", st)
                b = {"id": carrier, "name": label, "category": "structural", "mass": up.get("mass", 0.0),
                     "position": _pos(up.get("cg_position")) or wc or [0.0, 0.0, 0.0], "markers": {}}
                if desc:
                    b["description"] = desc
                if up.get("inertia"):
                    b["inertia"] = copy.deepcopy(up["inertia"])
                if up.get("visual"):
                    b["visual"] = copy.deepcopy(up["visual"])
                add_body(b)
            cb = body_by_id[carrier]
            if up.get("inertia") and not cb.get("inertia"):
                cb["inertia"] = copy.deepcopy(up["inertia"])     # the body existed already (body_ref) but without the upright's tensor
            mark_name = lambda k: k
        cb.setdefault("markers", {})
        for k, v in hps.items():
            cb["markers"].setdefault(mark_name(k), {"position": [round(x, 6) for x in v]})
        if not cb.get("mass"):
            notes_here.append("the wheel carrier has no mass")

        # ── wheel ──────────────────────────────────────────────────────────────────
        wh = copy.deepcopy(corner.get("wheel")) if isinstance(corner.get("wheel"), dict) else {}
        wheel_id = _slug(f"wheel_{sfx}")
        wvis = wh.pop("visual", None)
        if wheel_id not in body_by_id:
            spin = wh.get("rotational_inertia")
            wb = {"id": wheel_id, "name": f"Wheel {st}", "description": PART_TEXT["wheel"], "category": "structural", "mass": wh.get("mass", 0.0),
                  "position": wc or [0.0, 0.0, 0.0]}
            if isinstance(spin, (int, float)):
                wb["inertia"] = {"Ixx": spin / 2, "Iyy": spin, "Izz": spin / 2, "Ixy": 0.0, "Ixz": 0.0, "Iyz": 0.0}
            if wvis:
                wb["visual"] = wvis
            add_body(wb)
        if wc:
            joint(f"hinge_{wheel_id}", "revolute", carrier, wheel_id, wc, [0.0, 1.0, 0.0])
        else:
            notes_here.append("no wheel_center hardpoint: the wheel is not connected to its carrier")

        # ── links ──────────────────────────────────────────────────────────────────
        link_info = []          # (body id, segment start, end, hinge or None) for the outboard-body heuristic
        coupling_ref_for_station = None
        steering_joint = None
        for ln in topo.get("links") or []:
            if not isinstance(ln, dict) or not ln.get("name"):
                continue
            pts = [p for p in (_pos(x) for x in (ln.get("inboard_points") or [])) if p]
            ob_pt = _hardpoint(up.get("hardpoints"), ln.get("outboard_ref"))
            if not pts or ob_pt is None:
                notes_here.append(f"link '{ln.get('name')}': inboard points or outboard hardpoint missing; skipped")
                continue
            lname = str(ln["name"])
            is_leaf = "leaf" in lname.lower() or (str((corner.get("spring") or {}).get("type")) == "leaf" and ln.get("type") == "arm" and sysT == "solid_axle")
            is_tie = ("tie_rod" in lname.lower() or "track_rod" in lname.lower() or str(ln.get("outboard_ref", "")).endswith("steering_tie_rod_end")) and not is_leaf
            bid = ln.get("body_ref") or _slug(f"{_naming(lname, st)[0]}_{sfx}")
            if bid not in body_by_id:
                base, label, desc = _naming(lname, st)
                cg = _pos(ln.get("cg_position")) or [(a + b) / 2 for a, b in zip(_mean(pts), ob_pt)]
                lb = {"id": bid, "name": label, "category": "structural", "mass": ln.get("mass", 0.0), "position": cg}
                if desc:
                    lb["description"] = desc
                if ln.get("visual"):
                    lb["visual"] = copy.deepcopy(ln["visual"])
                add_body(lb)
            _keep_link_data(body_by_id[bid], ln, pts, notes_here)
            io, ii = infer_joint_types(sysT, ln.get("type"))
            ob_t, ib_t = ln.get("joint_type") or io, ln.get("inboard_joint_type") or ii
            direction = _norm(_sub(ob_pt, pts[0]))
            hinge = None

            if is_leaf:
                # Q4: a leaf pack is a link body: front eye revolute, rear slipper translational, the axle clamped at the seat
                joint(f"hinge_{bid}_front_eye", "revolute", "chassis", bid, pts[0], [0.0, 1.0, 0.0])
                if len(pts) > 1:
                    joint(f"slide_{bid}_rear_end", "translational", "chassis", bid, pts[1], _norm(_sub(pts[1], pts[0])))
                joint(f"fixed_{bid}_seat", "fixed", bid, carrier, ob_pt)
                notes_here.append(f"leaf spring '{lname}': eye / slipper / seat joints assumed (shackle swing not modelled)")
                link_info.append((bid, pts[0], ob_pt, (pts[0], [0.0, 1.0, 0.0])))
                continue

            # inboard end: chassis (or the rack for a tie rod) <-> link
            if ib_t is None and not is_tie:
                notes_here.append(f"link '{lname}': inboard joint type unknown ({sysT}/{ln.get('type')}); give inboard_joint_type")
            else:
                jt = _MAP_TYPE.get(ib_t or "spherical", ib_t or "spherical")
                extra, axis = {}, None
                if isinstance(ln.get("bushings"), list) and ln["bushings"] and isinstance(ln["bushings"][0], dict):
                    extra["bushing"] = copy.deepcopy(ln["bushings"][0])       # compliance stays on the joint, whatever its type
                if jt == "revolute":
                    axis = _norm(_sub(pts[1], pts[0])) if len(pts) >= 2 else ([1.0, 0.0, 0.0] if ln.get("type") == "pivot" else [0.0, 1.0, 0.0])
                    hinge = (pts[0], axis)
                elif jt in ("translational", "cylindrical", "planar"):
                    axis = direction
                elif jt == "universal":
                    axis = [0.0, 1.0, 0.0]
                parent = "chassis"
                if is_tie:
                    axle_key = _axle_of(st)[0] or "A1"
                    if axle_key not in racks:
                        rid = "rack" if not racks else f"rack_{_slug(axle_key)}"
                        racks[axle_key] = rid
                        rack_pos = [pts[0][0], 0.0, pts[0][2]]            # on the vehicle centreline
                        add_body({"id": rid, "name": "Steering rack", "description": PART_TEXT["rack"], "category": "structural", "mass": 0.0,
                                  "position": rack_pos})
                        joint(f"slide_{rid}", "translational", "chassis", rid, rack_pos, [0.0, 1.0, 0.0])
                        rack_joint[axle_key] = f"slide_{rid}"
                        notes.append(f"{axle_key}: steering rack added as a body on a lateral slide (travel and steering-wheel coupling not given by the old data)")
                    parent = racks[axle_key]
                    steering_joint = rack_joint.get(axle_key)
                if jt == "revolute" and len(pts) >= 2 and math.dist(pts[0], pts[1]) > 1e-6 and not is_tie:
                    # An arm with two pivots is two spherical joints at the real pickup points (the line through them is the
                    # hinge axis), so that each pickup can carry its own bushing. Bushings are matched to inboard_points by index.
                    bushes = ln.get("bushings") if isinstance(ln.get("bushings"), list) else []
                    for k, pt in enumerate(pts[:2]):
                        ex = {"bushing": copy.deepcopy(bushes[k])} if k < len(bushes) and isinstance(bushes[k], dict) else {}
                        add_con({"id": f"{_PREFIX['spherical']}_{bid}_inboard_{k + 1}", "type": "joint", "joint_type": "spherical", "body_a": parent,
                                 "frame_a": frame(pt), "body_b": bid, "frame_b": frame(pt), **ex})
                else:
                    add_con({"id": f"{_PREFIX[jt]}_{bid}_inboard", "type": "joint", "joint_type": jt, "body_a": parent, "frame_a": frame(pts[0], axis),
                             "body_b": bid, "frame_b": frame(pts[0], axis), **extra})
            # outboard end: link <-> carrier
            if ob_t is None:
                notes_here.append(f"link '{lname}': outboard joint type unknown ({sysT}/{ln.get('type')}); give joint_type")
            else:
                jt = _MAP_TYPE.get(ob_t, ob_t)
                axis = direction if jt in ("translational", "cylindrical", "planar") else ([0.0, 1.0, 0.0] if jt in ("revolute", "universal") else None)
                ob_ex = {"bushing": copy.deepcopy(ln["outboard_bushing"])} if isinstance(ln.get("outboard_bushing"), dict) else None
                joint(f"{_PREFIX[jt]}_{bid}_outboard", jt, bid, carrier, ob_pt, axis, ob_ex)
            link_info.append((bid, _mean(pts), ob_pt, hinge))

        # ── spring and damper ───────────────────────────────────────────────────────
        sp = corner.get("spring") if isinstance(corner.get("spring"), dict) else None
        dm = corner.get("damper") if isinstance(corner.get("damper"), dict) else None
        wc_pt = wc or [0.0, 0.0, 0.0]

        def outboard_body(point):
            best = (min([math.dist(point, p) for p in hps.values()] or [1e9]), carrier, None)
            for bid, a, b, hg in link_info:
                d = _seg_dist(point, a, b)
                if d <= best[0] + 1e-9:
                    best = (d, bid, hg)
            return best[1], best[2]

        def element(kind, d_old):
            """Build a spring / damper constraint from the old object; returns the new constraint or None."""
            ib, ob = _pos(d_old.get("inboard_mount")), _pos(d_old.get("outboard_mount"))
            given_mr = d_old.get("motion_ratio") if isinstance(d_old.get("motion_ratio"), (int, float)) and d_old["motion_ratio"] > 0 else None
            eid = f"{kind}_{sfx}"
            c = {"id": eid, "type": kind}
            geometric = ib is not None and ob is not None
            mr = None
            if geometric:
                body_b, hg = outboard_body(ob)
                mr = given_mr or estimate_motion_ratio(ib, ob, hg, wc_pt)
                c.update({"body_a": "chassis", "frame_a": frame(ib), "body_b": body_b, "frame_b": frame(ob)})
                c["x_outboard_body_inferred"] = True
                if mr is None:
                    geometric = False                     # cannot convert: fall back to a wheel-level element
                    notes_here.append(f"{kind}: mounts given but the motion ratio cannot be derived; kept as a wheel-level element")
            if not geometric:
                free = d_old.get("free_length") if isinstance(d_old.get("free_length"), (int, float)) and d_old["free_length"] > 0 else 0.30
                top = [wc_pt[0], wc_pt[1], wc_pt[2] - free]
                c.update({"body_a": "chassis", "frame_a": frame(top), "body_b": carrier, "frame_b": frame(wc_pt), "reference": "wheel"})
            return c, (mr if geometric else None), given_mr

        if sp and sp.get("type") not in ("none",):
            c, mr, given = element("spring", sp)
            hydraulic = sp.get("type") in ("hydraulic", "hydropneumatic")
            for k in ("type",):
                c["spring_type"] = sp.get(k)
            if sp.get("mass") is not None:
                c["mass"] = sp["mass"]
            for k in ("free_length", "preload", "coupling_ref", "end_type_front", "end_type_rear", "end_friction", "hysteresis_force", "leaf_count",
                      "effective_area", "nominal_pressure", "torsional_rate", "arm_length", "arm_start_angle", "hydropneumatic", "_est", "x_est"):
                if sp.get(k) is not None:
                    c[k] = copy.deepcopy(sp[k])
            kind_tors = sp.get("type") == "torsion_bar"
            if kind_tors:
                c["kind"] = "torsional"
                notes_here.append("torsion bar spring: kept with its old torsional fields; give the bar axis frames to make it an element about the axle")
            scale = 1.0
            if c.get("reference") != "wheel" and mr:
                scale = 1.0 / (mr * mr)
                c["x_converted_from_wheel_rate"] = {"motion_ratio": mr, "estimated": given is None,
                                                    **{f"wheel_{k}": sp[k] for k in ("rate", "preload") if sp.get(k) is not None}}
            if sp.get("rate") is not None:
                c["rate"] = round(sp["rate"] * scale, 4)
            if sp.get("rate_curve") is not None:
                c["rate_curve"] = _convert_curve(sp["rate_curve"], mr if (c.get("reference") != "wheel" and mr) else 1.0, 1.0 / mr if (c.get("reference") != "wheel" and mr) else 1.0)
            if "preload" in c and c.get("reference") != "wheel" and mr:
                c["preload"] = round(c["preload"] / mr, 4)
            if sp.get("visual"):
                c["visual"] = copy.deepcopy(sp["visual"])
            if c.get("rate") is None and c.get("rate_curve") is None and not hydraulic:
                notes_here.append("spring has no rate: skipped")
            else:
                add_con(c)
        elif sp:
            notes_here.append("spring type 'none': the station is sprung through its suspension coupling")
            if sp.get("coupling_ref"):
                coupling_ref_for_station = sp["coupling_ref"]
        if dm and (dm.get("bump_curve") or dm.get("rebound_curve") or dm.get("type") not in ("none", "hydraulic_strut")):
            c, mr, given = element("damper", dm)
            dref = dm.get("reference", "damper")
            for k in ("bump_curve", "rebound_curve"):
                if dm.get(k) is not None:
                    crv = dm[k]
                    if c.get("reference") == "wheel":
                        c[k] = copy.deepcopy(crv)                       # quoted at the wheel and kept there
                    elif dref == "wheel" and mr:
                        c[k] = _convert_curve(crv, mr, 1.0 / mr)        # wheel values -> element values
                    else:
                        c[k] = copy.deepcopy(crv)                       # already at the damper
            if c.get("reference") == "wheel" and dref == "damper" and dm.get("motion_ratio") not in (None, 1, 1.0):
                notes_here.append(f"damper: motion_ratio {dm['motion_ratio']} given without mounts; curves kept as quoted, at the wheel")
            if dref == "wheel" and mr and c.get("reference") != "wheel":
                c["x_converted_from_wheel_rate"] = {"motion_ratio": mr, "estimated": given is None}
            for k in ("mass", "_est"):
                if dm.get(k) is not None:
                    c[k if k != "mass" else "mass"] = dm[k]
            if dm.get("visual"):
                c["visual"] = copy.deepcopy(dm["visual"])
            if dm.get("type"):
                c["x_damper_type"] = dm["type"]
            if c.get("bump_curve") or c.get("rebound_curve"):
                add_con(c)

        # ── bump and rebound stops (wheel-level springs with a stop curve) ──────────────
        for key, sign in (("bump_stop", 1), ("rebound_stop", -1)):
            bs = corner.get(key)
            if not isinstance(bs, dict):
                continue
            gap = bs.get("gap", 0.0) or 0.0
            free = (sp or {}).get("free_length") if isinstance((sp or {}).get("free_length"), (int, float)) else 0.30
            if isinstance(bs.get("rate_curve"), list) and bs["rate_curve"]:
                pts_c = sorted([[sign * p[0], sign * p[1]] for p in bs["rate_curve"]])
                if [0.0, 0.0] not in pts_c:
                    pts_c = sorted(pts_c + [[0.0, 0.0]])
                curve = ([[-1.0, 0.0]] + [p for p in pts_c if p[0] > -1.0]) if sign == 1 else ([p for p in pts_c if p[0] < 1.0] + [[1.0, 0.0]])
            elif isinstance(bs.get("rate"), (int, float)):
                span = 0.1
                curve = [[-span, 0.0], [0.0, 0.0], [span, bs["rate"] * span]] if sign == 1 else [[-span, -bs["rate"] * span], [0.0, 0.0], [span, 0.0]]
            else:
                notes_here.append(f"{key}: neither rate nor rate_curve; skipped")
                continue
            top = [wc_pt[0], wc_pt[1], wc_pt[2] - free]
            c = {"id": f"{key}_{sfx}", "type": "spring", "spring_type": bs.get("type", "rubber"), "reference": "wheel", "body_a": "chassis",
                 "frame_a": frame(top), "body_b": carrier, "frame_b": frame(wc_pt),
                 "free_length": round(free - sign * gap, 6), "rate_curve": curve}
            if bs.get("_est") is not None:
                c["_est"] = bs["_est"]
            add_con(c)

        # ── anti-roll bar (data) ────────────────────────────────────────────────────────
        arb = corner.get("arb")
        if isinstance(arb, dict) and arb.get("bar_id"):
            bar = bars.setdefault(arb["bar_id"], {"id": arb["bar_id"], "stations": []})
            bar["stations"].append(st)
            for k_old, k_new in (("bar_rate", "bar_rate"), ("wheel_rate_equiv", "wheel_rate_equiv"), ("motion_ratio", "motion_ratio"), ("mass", "mass")):
                if arb.get(k_old) is not None:
                    bar.setdefault(k_new, arb[k_old])
            dl = _pos(arb.get("drop_link_mount"))
            if dl:
                bar.setdefault("drop_link_mounts", {})[st] = dl

        # ── the wheel station ───────────────────────────────────────────────────────────
        axle_key, side = _axle_of(st)
        s_out = {"id": st, "name": st, "wheel_body": wheel_id, "upright_body": carrier}
        if wc:
            s_out["wheel_center_marker"] = mark_name("wheel_center")
        if steering_joint:
            s_out["steering_joint"] = steering_joint
        if coupling_ref_for_station:
            s_out["coupling_ref"] = coupling_ref_for_station
        if axle_key:
            s_out["axle"], s_out["side"] = axle_key, side
        if sysT:
            s_out["topology_hint"] = sysT
        if wh:
            s_out["wheel"] = wh
        for k in ("tire", "brake", "alignment", "static_setup", "compliance_summary", "_est", "_est_note"):
            if corner.get(k) is not None:
                s_out[k] = copy.deepcopy(corner[k])
        if station_visual:
            s_out["visual"] = copy.deepcopy(station_visual)
        for k, v in topo.items():
            if k.startswith("x_") or k == "lateral_location":
                s_out[k if k.startswith("x_") else "x_lateral_location"] = copy.deepcopy(v)
        stations_out.append(s_out)
        notes.extend(f"{st}: {n}" for n in notes_here)

    out["wheel_stations"] = stations_out
    if bars:
        out["anti_roll_bars"] = list(bars.values())
    if not out["constraints"]:
        del out["constraints"]
    out.setdefault("_metadata", {})["version"] = VERSION
    return out, notes


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    with open(argv[0], "r", encoding="utf-8") as f:
        doc = json.load(f)
    new, notes = migrate(doc)
    out_path = argv[2] if len(argv) > 2 and argv[1] == "-o" else (argv[1] if len(argv) > 1 else None)      # `-o file` or just `file`
    if out_path:
        with open(out_path, "w", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(new, indent=2, ensure_ascii=False) + "\n")
    print(f"{argv[0]}: {len(new.get('wheel_stations', []))} stations, {len(new.get('constraints', []))} constraints, "
          f"{len(new.get('chassis', {}).get('mass_bodies', []))} bodies")
    for n in notes:
        print(f"  NOTE  {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
