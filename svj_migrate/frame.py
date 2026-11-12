"""SAE J670 <-> ISO 8855 conversion of an SVJ document (spec 2.1, v0.109).
Vendored from RFloEng/SVJ-standard-vehicle-json tools/ (Apache-2.0); regenerate with vendor_svj_migrate.py."""
"""
SVJ axis-frame conversion: SAE J670 (X forward, Y right, Z down) <-> ISO 8855 (X forward, Y left, Z up).

The two frames differ by one fixed proper rotation, 180 degrees about X: R = diag(1, -1, -1), quaternion
(qx, qy, qz, qw) = (1, 0, 0, 0). Nothing mirrors, so left stays left and a left-hand part stays a left-hand part.
The origin (front axle centre at ground level) is unchanged. Rules (proposals/svj_iso_frame_inventory.md, section 1):

    position or direction [x, y, z]      -> [x, -y, -z]
    frame orientation quaternion q       -> (1,0,0,0) * q   (left-multiply) = [qw, -qz, qy, -qx]; back: the negative
    inertia tensor in vehicle axes       -> Ixy and Ixz change sign (Ixx, Iyy, Izz, Iyz unchanged)
    inertia in a body frame (inertia_orientation present) -> unchanged: the tensor is in the body's own axes and the
                                            orientation is the part that moves
    scalars, and quantities defined in words (camber top inward, toe-in, bump) -> unchanged

Only fields in the allow-list below are touched; everything else (dimensions, colours, curves, asset data) is copied.

Usage:
    python tools/convert_frame.py file.svj.json --to ISO_8855 [-o out.svj.json]
    python tools/convert_frame.py file.svj.json --to SAE_J670 [-o out.svj.json]
    python tools/convert_frame.py file.svj.json --detect          # prints the declared frame

Functions:
    frame_of(doc)                 -> "SAE_J670" or "ISO_8855" (a missing declaration means SAE_J670, the pre-0.109 rule)
    convert(doc, to)              -> (new_doc, notes): deep copy in the target frame, the declaration updated
    flip_vector(v), quat_to_iso(q), quat_to_sae(q)
"""

import argparse
import copy
import json
import sys

SAE, ISO = "SAE_J670", "ISO_8855"
FRAMES = (SAE, ISO)

# Keys whose 3-number value is a position or direction in the vehicle frame.
VECTOR_KEYS = {
    "position", "center_of_gravity", "rack_position", "end_points", "end_seats", "pivot_position", "pendulum_pivot",
    "pendulum_axis", "attach_point", "cg_position", "beam_position", "center_of_pressure", "linkage_points",
    "input_flange", "output_flange", "output_front", "output_rear", "output_left", "output_right",
}
# Dictionaries whose entries are named positions when the value is a bare 3-vector (corner or side keyed).
SIDE_KEYS = {"FL", "FR", "RL", "RR", "left", "right"}
QUAT_KEYS = {"orientation", "inertia_orientation"}
PRODUCTS = ("Ixy", "Ixz")


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _neg(v):
    return v if v == 0 else -v


def _is_vec(v, n):
    return isinstance(v, list) and len(v) == n and all(_num(x) for x in v)


def flip_vector(v):
    return [v[0], _neg(v[1]), _neg(v[2])]


def quat_to_iso(q):
    """(1,0,0,0) * q for q = [x, y, z, w]."""
    x, y, z, w = q
    return [w, _neg(z), y, _neg(x)]


def quat_to_sae(q):
    """Inverse of quat_to_iso, exactly (the conjugate of (1,0,0,0) is its negative)."""
    return [_neg(c) for c in quat_to_iso(q)]


def _version_tuple(v):
    try:
        return tuple(int(p) for p in str(v).split("."))
    except ValueError:
        return (0, 0)


def _named_frame(value):
    if not isinstance(value, str):
        return None
    name = value.upper().replace(" ", "_").replace("-", "_")
    if name.startswith("ISO") or name == "OPENDRIVE":
        return ISO
    if name.startswith("SAE"):
        return SAE
    return None


def frame_of(doc):
    """The declared physics frame. `_metadata.physics_frame`, then a string `_metadata.coordinate_system` (an object
    there is the glTF asset convention); without a declaration the version decides: SAE below 0.109, ISO from it."""
    md = doc.get("_metadata") or {}
    for key in ("physics_frame", "coordinate_system"):
        f = _named_frame(md.get(key))
        if f:
            return f
    return ISO if _version_tuple(md.get("version")) >= (0, 109) else SAE


def frame_findings(doc):
    """Warnings when the heights in the file disagree with the declared frame (centre of gravity and wheel centres are
    above the ground: z > 0 in ISO 8855, z < 0 in SAE J670)."""
    frame = frame_of(doc)
    want = 1 if frame == ISO else -1
    zs = []
    cg = (doc.get("chassis") or {}).get("center_of_gravity")
    if _is_vec(cg, 3):
        zs.append(("center_of_gravity", cg[2]))
    for v in _find(doc, "wheel_center"):
        pos = v.get("position") if isinstance(v, dict) else v
        if _is_vec(pos, 3):
            zs.append(("wheel_center", pos[2]))
    wrong = [(n, z) for n, z in zs if z * want < 0]
    if not any(n == "wheel_center" for n, _ in zs):       # an attachment or a tyre file: heights are in its own frame
        return []
    if wrong and len(wrong) * 2 > len(zs):
        other = SAE if frame == ISO else ISO
        return ["frame: the file declares %s but %d of %d heights (centre of gravity, wheel centres) are on the %s side; "
                "it looks like %s (declare it in _metadata.coordinate_system, or convert with tools/convert_frame.py)"
                % (frame, len(wrong), len(zs), "negative" if frame == ISO else "positive", other)]
    return []


def _vec_groups(value):
    """A value under a vector key is [x,y,z] or a list of [x,y,z] (end_points, end_seats, linkage_points)."""
    return _is_vec(value, 3) or (isinstance(value, list) and value and all(_is_vec(v, 3) for v in value))


def convert(doc, to=ISO):
    """Deep copy of `doc` in the frame `to`, and a list of notes on what needs a person's eye."""
    if to not in FRAMES:
        raise ValueError("target frame must be one of %s" % (FRAMES,))
    out = copy.deepcopy(doc)
    notes = []
    src = frame_of(doc)
    if src == to:
        return out, ["already %s: nothing to do" % to]
    quat = quat_to_iso if to == ISO else quat_to_sae
    counts = {"vectors": 0, "quaternions": 0, "inertia": 0}

    def flip_value(v):
        if _is_vec(v, 3):
            counts["vectors"] += 1
            return flip_vector(v)
        counts["vectors"] += len(v)
        return [flip_vector(x) for x in v]

    def walk(node, path):
        if isinstance(node, dict):
            body_frame = "inertia_orientation" in node        # inertia is then in the body's own axes
            for k, v in list(node.items()):
                here = path + "/" + k
                if k in QUAT_KEYS and _is_vec(v, 4):
                    node[k] = quat(v)
                    counts["quaternions"] += 1
                elif k == "inertia" and isinstance(v, dict):
                    if not body_frame:
                        for p in PRODUCTS:
                            if _num(v.get(p)):
                                v[p] = _neg(v[p])
                                counts["inertia"] += 1
                elif (k in VECTOR_KEYS or (k in SIDE_KEYS and _is_vec(v, 3))) and _vec_groups(v):
                    node[k] = flip_value(v)
                else:
                    walk(v, here)
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, "%s[%d]" % (path, i))

    walk(out, "")

    if "absolute" in list(_find(out, "alignment_convention")):
        notes.append("alignment_convention 'absolute' found: toe and any Y/Z rotation change sign, camber does not; "
                     "check these values by hand (relative_to_centerline values stay)")
    if any(_find(out, "station")):
        notes.append("station sides are defined by the sign of Y (spec 23.1): L/R now follow the new Y; "
                     "left-keyed data was moved with its position, so nothing to do unless names disagree with signs")
    md = out.setdefault("_metadata", {})
    if isinstance(md.get("coordinate_system"), dict):     # glTF asset convention: the physics frame goes to its own key
        md["physics_frame"] = to
    else:
        md["coordinate_system"] = to
    notes.append("converted %s -> %s: %d vectors, %d quaternions, %d inertia products" %
                 (src, to, counts["vectors"], counts["quaternions"], counts["inertia"]))
    return out, notes


def _find(node, key):
    if isinstance(node, dict):
        for k, v in node.items():
            if k == key:
                yield v
            yield from _find(v, key)
    elif isinstance(node, list):
        for v in node:
            yield from _find(v, key)


def tyres_to_iso(doc):
    """Rewrite every tyre set declared `axis_system: SAE_J670` to ISO signs (an undeclared set in a file below 0.109 is
    taken as SAE when its pKy1 is negative). Returns (new_doc, names of the sets rewritten)."""
    try:
        from svj.tiremodel import to_iso_set
    except ImportError:
        from tire_model import to_iso_set
    out = copy.deepcopy(doc)
    old = _version_tuple((out.get("_metadata") or {}).get("version")) < (0, 109)
    done = []
    for name, s in (((out.get("tires") or {}).get("sets")) or {}).items():
        pj = s.get("pacejka") if isinstance(s, dict) else None
        if not isinstance(pj, dict):
            continue
        sae = pj.get("axis_system") == "SAE_J670" or (
            old and "axis_system" not in pj and _num((pj.get("lateral") or {}).get("pKy1")) and pj["lateral"]["pKy1"] < 0)
        if sae:
            pj["axis_system"] = "SAE_J670"
            s["pacejka"] = to_iso_set(pj)
            done.append(name)
    return out, done


def main(argv=None):
    ap = argparse.ArgumentParser(description="Convert an SVJ file between SAE J670 and ISO 8855 axes.")
    ap.add_argument("file")
    ap.add_argument("--to", choices=FRAMES, default=ISO)
    ap.add_argument("-o", "--output")
    ap.add_argument("--detect", action="store_true")
    ap.add_argument("--tyres", action="store_true", help="also rewrite SAE-signed tyre sets to ISO coefficients (with --to ISO_8855)")
    a = ap.parse_args(argv)
    with open(a.file, encoding="utf-8") as f:
        doc = json.load(f)
    if a.detect:
        print(frame_of(doc))
        return 0
    new, notes = convert(doc, a.to)
    if a.tyres and a.to == ISO:
        new, sets = tyres_to_iso(new)
        notes.append("tyre sets rewritten to ISO signs: %s" % (", ".join(sets) or "none"))
    for n in notes:
        print("note:", n, file=sys.stderr)
    text = json.dumps(new, indent=2, ensure_ascii=False) + "\n"
    if a.output:
        with open(a.output, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
