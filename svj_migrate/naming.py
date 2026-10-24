"""Part-name vocabulary of the SVJ standard (spec 24.2).
Vendored from RFloEng/SVJ-standard-vehicle-json tools/ (Apache-2.0); regenerate with vendor_svj_migrate.py."""
import re

_SIDE_SUFFIX = {"fl": "fl", "lf": "fl", "fr": "fr", "rf": "fr", "rl": "rl", "lr": "rl", "lb": "rl", "rr": "rr", "rb": "rr",
                "l": "l", "r": "r", "left": "l", "right": "r"}
_SIDE_PREFIX = {"front_left": "fl", "front_right": "fr", "rear_left": "rl", "rear_right": "rr", "left": "l", "right": "r",
                "fl": "fl", "fr": "fr", "rl": "rl", "rr": "rr"}
PART_CANONICAL = {"door", "hood", "trunk", "tailgate", "fuel_flap", "charge_flap", "mirror", "wiper", "sunroof", "cab", "tipper_body", "loader_boom", "loader_bucket", "boom", "stick", "bucket", "swing_frame", "hitch_lift_arm", "hitch_lower_link", "hitch_top_link", "hitch_drawbar", "implement"}
PART_ALIASES = {
    "car_door": "door",
    "bonnet": "hood",
    "engine_hood": "hood",
    "engine_bonnet": "hood",
    "boot": "trunk",
    "boot_lid": "trunk",
    "trunk_lid": "trunk",
    "deck_lid": "trunk",
    "rear_lid": "trunk",
    "liftgate": "tailgate",
    "lift_gate": "tailgate",
    "hatch": "tailgate",
    "rear_hatch": "tailgate",
    "hatchback": "tailgate",
    "tail_gate": "tailgate",
    "fuel_door": "fuel_flap",
    "fuel_filler_flap": "fuel_flap",
    "filler_flap": "fuel_flap",
    "fuel_cap_door": "fuel_flap",
    "gas_door": "fuel_flap",
    "charge_port_door": "charge_flap",
    "charge_door": "charge_flap",
    "charging_flap": "charge_flap",
    "side_mirror": "mirror",
    "wing_mirror": "mirror",
    "door_mirror": "mirror",
    "windscreen_wiper": "wiper",
    "windshield_wiper": "wiper",
    "sun_roof": "sunroof",
    "moonroof": "sunroof",
    "cabin": "cab",
    "driver_cab": "cab",
    "cab_shell": "cab",
    "truck_cab": "cab",
    "tipper": "tipper_body",
    "dump_body": "tipper_body",
    "dump_box": "tipper_body",
    "dumper_body": "tipper_body",
    "tipper_box": "tipper_body",
    "front_loader_boom": "loader_boom",
    "loader_arm": "loader_boom",
    "front_loader_arm": "loader_boom",
    "front_loader_bucket": "loader_bucket",
    "loader_shovel": "loader_bucket",
    "front_bucket": "loader_bucket",
    "crane_boom": "boom",
    "excavator_boom": "boom",
    "dipper": "stick",
    "dipper_stick": "stick",
    "dipper_arm": "stick",
    "excavator_bucket": "bucket",
    "turret": "swing_frame",
    "upper_structure": "swing_frame",
    "superstructure": "swing_frame",
    "lift_arm": "hitch_lift_arm",
    "rockshaft_arm": "hitch_lift_arm",
    "lower_link": "hitch_lower_link",
    "lift_link": "hitch_lower_link",
    "three_point_lower_link": "hitch_lower_link",
    "top_link": "hitch_top_link",
    "upper_link": "hitch_top_link",
    "third_link": "hitch_top_link",
    "drawbar": "hitch_drawbar",
    "draw_bar": "hitch_drawbar",
    "tool": "implement",
    "working_implement": "implement",
}
_ACTUATOR_ALIAS_SUFFIX = ("_ram", "_piston")

# suspension, steering and brake parts (docs/naming_convention.md, "Canonical part names"): a body that is one of
# these keeps the canonical name; the mechanism is defined by the components, never by the name
_SUSPENSION_VOCAB = {
    "upright": ("knuckle", "hub_carrier", "spindle", "stub_axle"), "hub": ("hub_flange", "wheel_hub"),
    "upper_wishbone": ("upper_arm", "uca", "wishbone_upper", "a_arm_top"), "lower_wishbone": ("lower_arm", "lca", "wishbone_lower", "a_arm_bottom"),
    "upper_link_front": ("upper_fore",), "upper_link_rear": ("upper_aft",), "lower_link_front": ("lower_fore",), "lower_link_rear": ("lower_aft",),
    "trailing_arm": ("longitudinal_arm", "radius_arm"), "semi_trailing_arm": ("sta", "diagonal_arm"), "leading_arm": ("front_trailing_arm",),
    "tie_rod": ("track_rod", "steering_rod", "steering_link"), "toe_link": ("toe_rod", "toe_control_arm"), "camber_link": ("camber_rod",),
    "drag_link": ("relay_rod", "centre_link"), "pushrod": ("push_rod",), "pullrod": ("pull_rod",), "rocker": ("bellcrank", "rocker_arm"),
    "strut": ("macpherson_strut", "damper_strut"), "torque_rod": ("radius_rod", "torque_arm"), "panhard_rod": ("track_bar", "lateral_rod"),
    "watts_link_rod": ("watts_arm",), "watts_pivot": ("watts_centre",), "axle_body": ("axle_housing", "de_dion_tube", "beam_axle"),
    "spring": ("coil", "air_spring"), "leaf_spring": (), "damper": ("shock", "shock_absorber"),
    "arb": ("sway_bar", "stabiliser_bar", "torsion_bar_arb"), "drop_link": ("arb_link", "end_link"),
    "wheel": ("rim", "tire", "tyre"), "disc": ("rotor", "brake_disc", "drum"), "caliper": ("brake_caliper", "brake_cylinder"),
    "rack": ("steering_gear", "steering_box"), "column": ("steering_shaft",), "steering_wheel": ("handwheel",),
}
for _canon, _als in _SUSPENSION_VOCAB.items():
    PART_CANONICAL.add(_canon)
    for _al in _als:
        PART_ALIASES[_al] = _canon


def canonical_part_id(pid):
    """The canonical spelling of a movable-part id, or None when the id is not a known part.

    Normalises case and separators, moves a side prefix (`front_left_door`) to a suffix, maps known
    aliases (`bonnet` -> `hood`, `hatch` -> `tailgate`) and keeps the side (`door_lf` -> `door_fl`).
    """
    if not isinstance(pid, str):
        return None
    base = "_".join(pid.strip().lower().replace("-", "_").split())
    side = None
    for pre in sorted(_SIDE_PREFIX, key=len, reverse=True):
        if base.startswith(pre + "_"):
            side, base = _SIDE_PREFIX[pre], base[len(pre) + 1:]
            break
    else:
        head, _, tail = base.rpartition("_")
        if head and tail in _SIDE_SUFFIX:
            base, side = head, _SIDE_SUFFIX[tail]
        elif head and re.fullmatch(r"a\d{1,2}[lrc]", tail):          # multi-axle station suffix (a1l, a3r, a2c)
            base, side = head, tail
    base = PART_ALIASES.get(base, base)
    if base not in PART_CANONICAL:
        return None
    return base + ("_" + side if side else "")
