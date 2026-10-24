"""
SVJ Multibody Topology Checker (v0.96 fields, suspension geometry rules v0.99.4)
===============================================================================
Cross-reference rules for the multibody fields that JSON Schema cannot express
(proposals/svj_multibody_topology_proposal.md): `body_ref`, `mass_body.parent`,
`orientation` quaternions, `joint_type` / `inboard_joint_type`, and the
`outboard_ref` hardpoint reference. Since v0.99.4 also: zero-length links (the
MacPherson `strut_lower` convention), the spring/damper `*_point` misspelling of
`*_mount`, and `damper.reference`. Also provides `effective_joints()`, which
applies the proposal's inference rules so every converter resolves missing joint
types the same way.

Used automatically by tools/validate.py; can also run standalone.

svj-py ships an identical copy as svj/multibody.py — keep the two files in sync
(tests/test_vehicle.py::test_multibody_checker_in_sync enforces it).

Usage:
    python tools/multibody_check.py <file.svj.json> [file2 ...]
    python tools/multibody_check.py --joints <file.svj.json>    # print effective joints
"""

import json
import math
import sys

ROOT_BODIES = {"chassis", "ground"}
QUAT_ERROR = 1e-2     # |norm - 1| above this is an error (not a rotation)
QUAT_WARN = 1e-3
MASS_REL_TOL = 0.01   # body vs link/upright mass
CG_TOL = 0.005        # m, body position vs link/upright cg_position
ZERO_LEN = 1e-3       # m, inboard point closer than this to the outboard hardpoint

# (outboard, inboard) joint types inferred when a link states none, keyed by
# (system_type group, link type). None = not inferable (compliant or special
# members such as leaf springs): converters must be given the joint explicitly.
_STRUT_FAMILY = {"macpherson", "chapman_strut"}
_ARM_FAMILY = {"double_wishbone", "macpherson", "chapman_strut", "multi_link", "custom"}
_PIVOT_ARM_FAMILY = {"trailing_arm", "semi_trailing_arm", "torsion_beam", "parallelogram"}


def infer_joint_types(system_type, link_type):
    """Return (outboard, inboard) for a link, or (None, None) when not inferable."""
    lt = link_type or "arm"
    if lt in ("rod", "torque_rod"):
        return "spherical", "spherical"
    if lt == "strut" and system_type in _STRUT_FAMILY:
        return "translational", "spherical"          # the strut top mount pivots; a fixed top would lock the upright
    if lt == "arm":
        if system_type in _ARM_FAMILY:
            return "spherical", "revolute"
        if system_type in _PIVOT_ARM_FAMILY:
            return "revolute", "revolute"
    if lt == "pivot" and system_type == "pendulum_axle":
        return "revolute", None
    return None, None


def _stations(susp):
    return [k for k in (susp or {}) if not k.startswith(("x_", "_")) and isinstance(susp[k], dict)]


def effective_joints(doc):
    """List every suspension link with its effective joint types.

    Each item: {station, link, system_type, outboard, inboard, outboard_source,
    inboard_source, body_ref}; a *_source is 'explicit', 'inferred' or None.
    """
    out = []
    susp = doc.get("suspension") if isinstance(doc.get("suspension"), dict) else {}
    for st in _stations(susp):
        topo = susp[st].get("topology") or {}
        sysT = topo.get("system_type")
        for ln in topo.get("links") or []:
            if not isinstance(ln, dict):
                continue
            io, ii = infer_joint_types(sysT, ln.get("type"))
            ob, ib = ln.get("joint_type"), ln.get("inboard_joint_type")
            out.append({
                "station": st, "link": ln.get("name"), "system_type": sysT,
                "outboard": ob or io, "inboard": ib or ii,
                "outboard_source": "explicit" if ob else ("inferred" if io else None),
                "inboard_source": "explicit" if ib else ("inferred" if ii else None),
                "body_ref": ln.get("body_ref"),
            })
    return out


def _walk_orientations(obj, path, found):
    if isinstance(obj, dict):
        for k, v in obj.items():
            p = f"{path}.{k}" if path else k
            if k == "orientation" and isinstance(v, list) and len(v) == 4 \
                    and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v):
                found.append((p, v))
            else:
                _walk_orientations(v, p, found)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            _walk_orientations(v, f"{path}[{i}]", found)


def check(doc):
    errors, warnings = [], []
    chassis = doc.get("chassis") if isinstance(doc.get("chassis"), dict) else {}
    bodies = [b for b in (chassis.get("mass_bodies") or []) if isinstance(b, dict)]
    by_id = {}
    for b in bodies:
        bid = b.get("id")
        if bid in by_id:
            errors.append(f"chassis.mass_bodies: duplicate id '{bid}'")
        by_id[bid] = b

    # ── parent hierarchy ────────────────────────────────────────────────────
    for b in bodies:
        par = b.get("parent")
        if par is None:
            continue
        if par == b.get("id"):
            errors.append(f"mass_body '{b['id']}': parent is itself")
        elif par not in ROOT_BODIES and par not in by_id:
            errors.append(f"mass_body '{b.get('id')}': parent '{par}' is not a mass_body id, 'chassis' or 'ground'")
    for b in bodies:
        seen, cur = set(), b.get("id")
        while cur in by_id and by_id[cur].get("parent") not in (None, *ROOT_BODIES):
            if cur in seen:
                errors.append(f"mass_body '{b.get('id')}': parent chain forms a cycle")
                break
            seen.add(cur)
            cur = by_id[cur]["parent"]

    # ── body_ref / outboard_ref / joints / masses ──────────────────────────
    claimed = {}

    def claim(ref, owner):
        if ref not in by_id:
            errors.append(f"{owner}: body_ref '{ref}' is not a chassis.mass_bodies id")
            return None
        if ref in claimed and claimed[ref] != owner:
            errors.append(f"{owner}: body_ref '{ref}' is already used by {claimed[ref]}")
        claimed.setdefault(ref, owner)
        return by_id[ref]

    def compare(owner, item, body):
        m, bm = item.get("mass"), body.get("mass")
        if isinstance(m, (int, float)) and isinstance(bm, (int, float)) and bm > 0 \
                and abs(m - bm) > MASS_REL_TOL * max(m, bm):
            warnings.append(f"{owner}: mass {m} differs from body '{body.get('id')}' mass {bm}")
        cg, bp = item.get("cg_position"), body.get("position")
        if isinstance(cg, list) and isinstance(bp, list) and len(cg) == 3 and len(bp) == 3 \
                and all(isinstance(x, (int, float)) for x in cg + bp) \
                and max(abs(a - b) for a, b in zip(cg, bp)) > CG_TOL:
            warnings.append(f"{owner}: cg_position {cg} differs from body '{body.get('id')}' position {bp}")

    susp = doc.get("suspension") if isinstance(doc.get("suspension"), dict) else {}
    for st in _stations(susp):
        topo = susp[st].get("topology") or {}
        up = topo.get("upright") if isinstance(topo.get("upright"), dict) else {}
        if up.get("body_ref") is not None:
            owner = f"suspension.{st}.topology.upright"
            body = claim(up["body_ref"], owner)
            if body:
                compare(owner, up, body)
        hps = up.get("hardpoints") if isinstance(up.get("hardpoints"), dict) else {}
        for i, ln in enumerate(topo.get("links") or []):
            if not isinstance(ln, dict):
                continue
            owner = f"suspension.{st}.topology.links[{i}] '{ln.get('name')}'"
            if ln.get("body_ref") is not None:
                body = claim(ln["body_ref"], owner)
                if body:
                    compare(owner, ln, body)
            ref = ln.get("outboard_ref")
            if isinstance(ref, str) and ref.startswith("hardpoints.") and up and ref[len("hardpoints."):] not in hps:
                errors.append(f"{owner}: outboard_ref '{ref}' is not an upright hardpoint")
            bush, pts = ln.get("bushings"), ln.get("inboard_points")
            ob = hps.get(ref[len("hardpoints."):]) if isinstance(ref, str) and ref.startswith("hardpoints.") else None
            if isinstance(ob, dict):
                ob = ob.get("position")
            if ln.get("type") != "pivot" and isinstance(ob, list) and len(ob) == 3 and isinstance(pts, list):
                for pi, ip in enumerate(pts):
                    if isinstance(ip, list) and len(ip) == 3 and all(isinstance(x, (int, float)) for x in ip + ob)                             and math.dist(ip, ob) < ZERO_LEN:
                        hint = (" — a strut's outboard end is its lower clamp on the upright: use "
                                "'hardpoints.strut_lower'") if ln.get("type") == "strut" else ""
                        warnings.append(f"{owner}: inboard_points[{pi}] coincides with '{ref}' "
                                        f"(zero-length link){hint}")
            if isinstance(bush, list) and isinstance(pts, list) and len(bush) != len(pts):
                warnings.append(f"{owner}: {len(bush)} bushings for {len(pts)} inboard_points")
            if ln.get("type") == "arm" and ln.get("inboard_joint_type") == "spherical" \
                    and isinstance(pts, list) and len(pts) >= 2:
                warnings.append(
                    f"{owner}: an arm with {len(pts)} inboard points and a spherical inboard joint is "
                    "over-constrained in rotation; use 'revolute' or 'bushing'"
                )

    # ── spring / damper keys and damper reference (v0.99.4) ─────────────────
    for st in _stations(susp):
        corner = susp[st]
        for part in ("spring", "damper"):
            obj = corner.get(part)
            if not isinstance(obj, dict):
                continue
            for bad, good in (("inboard_point", "inboard_mount"), ("outboard_point", "outboard_mount")):
                if bad in obj:
                    warnings.append(f"suspension.{st}.{part}: unknown key '{bad}' is ignored — did you mean '{good}'?")
        dmp = corner.get("damper")
        if isinstance(dmp, dict) and dmp.get("reference") == "wheel":
            mr = dmp.get("motion_ratio")
            if isinstance(mr, (int, float)) and abs(mr - 1.0) > 1e-9:
                warnings.append(
                    f"suspension.{st}.damper: reference 'wheel' with motion_ratio {mr} — curves already are "
                    "at the wheel; omit motion_ratio or set it to 1"
                )

    # ── orientation quaternions ────────────────────────────────────────────
    found = []
    _walk_orientations({"suspension": susp, "chassis": chassis}, "", found)
    for path, q in found:
        n = math.sqrt(sum(x * x for x in q))
        dev = abs(n - 1.0)
        if dev > QUAT_ERROR:
            errors.append(f"{path}: quaternion {q} is not a unit quaternion (norm {n:.4f})")
        elif dev > QUAT_WARN:
            warnings.append(f"{path}: quaternion norm {n:.4f} deviates from 1 (renormalise)")
    return errors, warnings


def main(argv):
    show_joints = "--joints" in argv
    files = [a for a in argv if a != "--joints"]
    if not files:
        print(__doc__)
        return 2
    bad = False
    for path in files:
        with open(path, "r", encoding="utf-8") as f:
            doc = json.load(f)
        if show_joints:
            print(path)
            for j in effective_joints(doc):
                tag = lambda s: {"explicit": "", "inferred": "*", None: "?"}[s]
                print(f"  {j['station']:<4} {str(j['link']):<26} out={j['outboard'] or '-'}{tag(j['outboard_source'])}"
                      f"  in={j['inboard'] or '-'}{tag(j['inboard_source'])}  body={j['body_ref'] or '-'}")
            print("  (* inferred from system_type, ? not inferable)")
            continue
        errs, warns = check(doc)
        print(f"{path}: {len(errs)} error(s), {len(warns)} warning(s)")
        for w in warns:
            print(f"  WARN   {w}")
        for e in errs:
            print(f"  ERROR  {e}")
        bad = bad or bool(errs)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
