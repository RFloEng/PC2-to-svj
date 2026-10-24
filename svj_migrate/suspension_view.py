"""
SVJ Derived Suspension View (v0.99.8)
=====================================
Expands the authored suspension (corners, uprights, links, hardpoints) into the same
body / joint model as spec §24, so solvers, exporters and attachments see one graph:
bodies with mass properties, and `joint` constraints with inline coincident frames.

Nothing authored changes. The view is generated, never stored. Springs, dampers and
anti-roll bars stay on their corner: their rates are wheel-referenced (§9.3, §9.4) and
turning them into element rates needs motion-ratio kinematics that SVJ does not solve.
The expanded joints form closed loops; poses do not solve them (spec §24.8).

svj-py ships an identical copy as svj/suspension_view.py — keep the two files in sync
(tests/test_vehicle.py::test_suspension_view_in_sync enforces it).

Usage:
    python tools/suspension_view.py <file.svj.json>            # list bodies and joints
    python tools/suspension_view.py --json <file.svj.json>     # the expansion as JSON
"""

import copy
import json
import math
import re
import sys

from .multibody_check import infer_joint_types

SPECIAL = ("x_", "_")
Q_ID = [0.0, 0.0, 0.0, 1.0]
_PREFIX = {"revolute": "hinge", "translational": "slide", "cylindrical": "cyl", "spherical": "ball",
           "universal": "universal", "planar": "planar", "fixed": "fixed"}
_MAP_TYPE = {"convel": "universal", "hooke": "universal", "bushing": "fixed"}


def _slug(s):
    return re.sub(r"[^a-z0-9_]+", "_", str(s).strip().lower()).strip("_") or "part"


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _norm(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v] if n > 1e-12 else None


def quat_z_to(axis):
    """Unit quaternion [x, y, z, w] rotating +Z onto `axis` (the frame convention of spec §24.3)."""
    a = _norm(axis)
    if a is None:
        return list(Q_ID)
    cx, cy, cz = -a[1], a[0], 0.0                      # z × a
    w = 1.0 + a[2]
    if w < 1e-9:                                       # axis = -Z: half turn about X
        return [1.0, 0.0, 0.0, 0.0]
    n = math.sqrt(cx * cx + cy * cy + cz * cz + w * w)
    return [cx / n, cy / n, cz / n, w / n]


def _pos(v):
    if isinstance(v, dict):
        v = v.get("position")
    return list(v) if isinstance(v, (list, tuple)) and len(v) == 3 else None


def _hardpoint(hps, ref):
    if isinstance(ref, str) and ref.startswith("hardpoints."):
        return _pos((hps or {}).get(ref[len("hardpoints."):]))
    return None


def _mean(points):
    return [sum(p[i] for p in points) / len(points) for i in range(3)]


def _stations(susp):
    return [k for k in (susp or {}) if not k.startswith(SPECIAL) and isinstance(susp[k], dict) and isinstance(susp[k].get("topology"), dict)]


def _inertia_obj(v):
    return copy.deepcopy(v) if isinstance(v, dict) or (isinstance(v, list) and len(v) == 3) else None


def expand_suspension(doc):
    """Return {'mass_bodies', 'constraints', 'notes'} for every wheel station of `doc`.

    Bodies already present in `chassis.mass_bodies` (through `body_ref`) are reused, not duplicated.
    Joint ids follow docs/naming_convention.md: `<hinge|ball|slide|fixed|…>_<body>_<inboard|outboard>`.
    """
    susp = doc.get("suspension") if isinstance(doc.get("suspension"), dict) else {}
    existing = {b.get("id") for b in (doc.get("chassis") or {}).get("mass_bodies", []) if isinstance(b, dict)}
    out_bodies, out_cons, notes = [], [], []
    seen_ids = set(existing)

    def add_body(b):
        if b["id"] in seen_ids:
            return b["id"]
        seen_ids.add(b["id"])
        out_bodies.append(b)
        return b["id"]

    def add_joint(jid, jt, a, b, pos, axis=None, extra=None):
        frame = {"position": [round(x, 6) for x in pos], "orientation": [round(x, 7) for x in quat_z_to(axis)] if axis else list(Q_ID)}
        c = {"id": jid, "type": "joint", "joint_type": jt, "body_a": a, "frame_a": copy.deepcopy(frame), "body_b": b, "frame_b": frame, "x_derived": True}
        if extra:
            c.update(extra)
        out_cons.append(c)

    for st in _stations(susp):
        corner, topo = susp[st], susp[st]["topology"]
        sfx = _slug(st)
        sysT = topo.get("system_type")
        up = topo.get("upright") if isinstance(topo.get("upright"), dict) else {}
        hps = up.get("hardpoints") if isinstance(up.get("hardpoints"), dict) else {}
        axle = topo.get("axle_body") if isinstance(topo.get("axle_body"), dict) else None

        # the body that carries the wheel: the axle beam when there is one (shared by both stations), else the upright
        if axle and axle.get("id"):
            carrier_id = _slug(axle["id"])
            if carrier_id not in seen_ids:
                add_body({"id": carrier_id, "name": f"Axle body {axle['id']}", "category": "structural", "mass": axle.get("mass", 0.0),
                          "position": _pos(axle.get("position")) or _pos(hps.get("wheel_center")) or [0.0, 0.0, 0.0],
                          **({"inertia": _inertia_obj(axle["inertia"])} if _inertia_obj(axle.get("inertia")) else {}), "x_derived": True})
        else:
            carrier_id = up.get("body_ref") or _slug(up.get("id") or f"upright_{sfx}")
            if carrier_id not in seen_ids:
                add_body({"id": carrier_id, "name": f"Upright {st}", "category": "structural", "mass": up.get("mass", 0.0),
                          "position": _pos(up.get("cg_position")) or _pos(hps.get("wheel_center")) or [0.0, 0.0, 0.0],
                          **({"inertia": _inertia_obj(up["inertia"])} if _inertia_obj(up.get("inertia")) else {}), "x_derived": True})
        if not (up.get("mass") or (axle or {}).get("mass")):
            notes.append(f"{st}: the upright / axle body has no mass; the derived body is massless")

        # wheel: spin about the lateral axis through the wheel centre
        wc = _pos(hps.get("wheel_center"))
        wh = corner.get("wheel") if isinstance(corner.get("wheel"), dict) else {}
        if wc:
            wid = _slug(f"wheel_{sfx}")
            spin = wh.get("rotational_inertia")
            inertia = {"Ixx": spin / 2, "Iyy": spin, "Izz": spin / 2, "Ixy": 0.0, "Ixz": 0.0, "Iyz": 0.0} if isinstance(spin, (int, float)) else None
            add_body({"id": wid, "name": f"Wheel {st}", "category": "structural", "mass": wh.get("mass", 0.0), "position": wc,
                      **({"inertia": inertia} if inertia else {}), "x_derived": True})
            add_joint(f"hinge_{wid}", "revolute", carrier_id, wid, wc, [0.0, 1.0, 0.0])
            if inertia:
                notes.append(f"{st}: the wheel tensor is a disc approximation of its spin inertia (Iyy = rotational_inertia, Ixx = Izz = half)")

        # links
        for ln in topo.get("links") or []:
            if not isinstance(ln, dict) or not ln.get("name"):
                continue
            pts = [_pos(p) for p in (ln.get("inboard_points") or [])]
            pts = [p for p in pts if p]
            ob_pt = _hardpoint(hps, ln.get("outboard_ref"))
            if not pts or ob_pt is None:
                notes.append(f"{st} {ln.get('name')}: inboard points or outboard hardpoint missing; link skipped")
                continue
            bid = ln.get("body_ref") or _slug(f"{ln['name']}_{sfx}")
            cg = _pos(ln.get("cg_position")) or [(a + b) / 2 for a, b in zip(_mean(pts), ob_pt)]
            if bid not in seen_ids:
                add_body({"id": bid, "name": f"{ln['name']} {st}", "category": "structural", "mass": ln.get("mass", 0.0), "position": cg, "x_derived": True})
            io, ii = infer_joint_types(sysT, ln.get("type"))
            ob_t = ln.get("joint_type") or io
            ib_t = ln.get("inboard_joint_type") or ii
            direction = _norm(_sub(ob_pt, pts[0]))

            # inboard: chassis <-> link
            if ib_t is None:
                notes.append(f"{st} {ln['name']}: inboard joint type unknown ({sysT}/{ln.get('type')}); give inboard_joint_type")
            else:
                jt = _MAP_TYPE.get(ib_t, ib_t)
                extra = {}
                bushes = ln.get("bushings") if isinstance(ln.get("bushings"), list) else []
                if bushes and isinstance(bushes[0], dict):
                    extra["bushing"] = copy.deepcopy(bushes[0])        # compliance stays on the joint, whatever its type
                axis = None
                if jt == "revolute":
                    axis = _norm(_sub(pts[1], pts[0])) if len(pts) >= 2 else ([1.0, 0.0, 0.0] if ln.get("type") == "pivot" else [0.0, 1.0, 0.0])
                    if len(pts) < 2:
                        notes.append(f"{st} {ln['name']}: single inboard point; hinge axis assumed {'longitudinal' if ln.get('type') == 'pivot' else 'lateral'}")
                elif jt in ("translational", "cylindrical", "planar"):
                    axis = direction
                elif jt == "universal":
                    axis = [0.0, 1.0, 0.0]
                body = next((b for b in out_bodies if b["id"] == bid), None)
                if jt == "revolute" and len(pts) >= 2 and math.dist(pts[0], pts[1]) > 1e-6:
                    # an arm with two pivots: two spherical joints at the real pickups (their line is the hinge axis), one bushing each
                    for k, pt in enumerate(pts[:2]):
                        ex = {"bushing": copy.deepcopy(bushes[k])} if k < len(bushes) and isinstance(bushes[k], dict) else {}
                        add_joint(f"{_PREFIX['spherical']}_{bid}_inboard_{k + 1}", "spherical", "chassis", bid, pt, None, ex)
                else:
                    add_joint(f"{_PREFIX[jt]}_{bid}_inboard", jt, "chassis", bid, pts[0], axis, extra)
                for i, pt in enumerate(pts[2:] if jt == "revolute" and len(pts) >= 2 else pts[1:], start=3 if jt == "revolute" and len(pts) >= 2 else 2):
                    if body is not None:
                        body.setdefault("markers", {})[f"pickup_{i}"] = {"position": [round(x, 6) for x in pt]}
                        notes.append(f"{st} {ln['name']}: inboard point {i} kept as marker pickup_{i} (no joint)")
                    else:
                        notes.append(f"{st} {ln['name']}: inboard point {i} not carried (the link body already exists)")

            # outboard: link <-> upright / axle body
            if ob_t is None:
                notes.append(f"{st} {ln['name']}: outboard joint type unknown ({sysT}/{ln.get('type')}); give joint_type")
            else:
                jt = _MAP_TYPE.get(ob_t, ob_t)
                axis = None
                if jt in ("translational", "cylindrical", "planar"):
                    axis = direction
                elif jt in ("revolute", "universal"):
                    axis = [0.0, 1.0, 0.0]
                add_joint(f"{_PREFIX[jt]}_{bid}_outboard", jt, bid, carrier_id, ob_pt, axis)

    return {"mass_bodies": out_bodies, "constraints": out_cons, "notes": notes}


def with_expanded_suspension(doc):
    """A deep copy of `doc` with the derived bodies and joints appended to `chassis.mass_bodies` and `constraints`.

    Derived entries carry `x_derived: true`. The copy has closed kinematic loops; it is meant for export,
    inspection and for attachments that mount on suspension bodies, not for pose solving.
    """
    view = expand_suspension(doc)
    out = copy.deepcopy(doc)
    ch = out.setdefault("chassis", {})
    ch.setdefault("mass_bodies", []).extend(view["mass_bodies"])
    have = {c.get("id") for c in out.get("constraints", []) if isinstance(c, dict)}
    out.setdefault("constraints", []).extend(c for c in view["constraints"] if c["id"] not in have)
    return out


def main(argv):
    as_json = "--json" in argv
    files = [a for a in argv if a != "--json"]
    if not files:
        print(__doc__)
        return 2
    for path in files:
        with open(path, "r", encoding="utf-8") as f:
            view = expand_suspension(json.load(f))
        if as_json:
            print(json.dumps(view, indent=2))
            continue
        print(f"{path}: {len(view['mass_bodies'])} derived bodies, {len(view['constraints'])} joints")
        for c in view["constraints"]:
            print(f"  {c['id']:<52} {c['joint_type']:<13} {c['body_a']} -> {c['body_b']}")
        for n in view["notes"]:
            print(f"  NOTE  {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
