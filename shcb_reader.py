#!/usr/bin/env python3
"""
shcb_reader.py
==============
Parser and explorer for the ShCB binary container used by Project CARS 2
physics files (.edfbin, .cdfbin, .sdfbin, .gdfbin, .hdtbin).

Extracted from a stock pCARS2 install via PCarsTools 1.1.4:
  https://github.com/Nenkai/PCarsTools

Container grammar (reverse-engineered):
  File = ShCBHeader + PrimaryBlock + [SecondaryBlock]
  Record = 0x24 tag + 4-byte hash + 1-byte typeCode + body_bytes
  Records are delimited by the 0x24 marker byte.

Type codes decoded so far:
  0x21         body=4   : single f32
  0x22         body=8   : two f32
  0x23         body=var : N × f32
  0x03         body=9   : [1B flag, 4B u32, 4B f32]
  0x13         body=var : mixed scalar
  0x52         body=8   : two u32 (RPM pair, etc.)
  0x82         body=5   : [1B prefix, f32]
  0x83         body=var : first record in a curve (header)
  0x93         body=13  : data point in a curve: [1B, u32 RPM, f32 val1, f32 val2]
  0x86         body=22  : first record in multi-column curve
  0x96         body=25  : data point in multi-column curve: [1B, u32 RPM, 5×f32]
  0xa2/0xa3    body=var : compound/nested records

Known hash → property-name table (little-endian hex strings):
  All hashes observed across .edfbin files unless noted.
"""

import struct
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ─── Known property hash → human-readable name ──────────────────────────────
# Format: 4-byte LE hash stored as 8-char lowercase hex string → name
#
# Discovery method: value-range heuristics across multiple cars, cross-validated
# against known real-world specs (NSX 2017, McLaren P1, Ariel Atom 3, etc.).
# Entries marked with (?) are best-guess pending further validation.

HASH_NAMES: Dict[str, str] = {
    # ── Engine (.edfbin) ─────────────────────────────────────────────────────
    "8b0ab771": "engine.torque_curve",          # 30-point RPM→(friction, gross_torque)
    "515f5e83": "engine.efficiency_map",         # RPM→5 efficiency multipliers
    "4d239754": "engine.idle_rpm_range",         # (min_rpm u32, max_rpm u32)
    "7902b6bd": "engine.torque_peak_rpm_range",  # (low_rpm u32, high_rpm u32)
    "d39464af": "engine.rev_limiter",            # (cut_rpm u32, hysteresis_rpm u32)
    "0571c719": "engine.boost_pressure",         # (max_pa f32, low_pa f32) – turbo
    "a700d23a": "engine.fuel_params",            # (fuel_scale f32, ?, ?, ?) (?)
    "6ada2b3a": "engine.throttle_response",      # (?)
    "83152f20": "engine.inertia_flag",           # (?)
    "63233a14": "engine.aux_scalar",             # (?)
    "0acea858": "engine.misc_scalar",            # (?)
    "f4482252": "engine.misc_6b",               # (?)
    "bf847cf1": "engine.compound_A",             # (?)
    "ceb17525": "engine.compound_B",             # (?)
    "5217fb41": "engine.compound_C",             # (?)
    "fc89e89c": "engine.compound_D",             # (?)
    "d774451a": "engine.friction_curve_base",    # (?)
    "dea72eb7": "engine.thermal_params",         # (?)
    "c1f4543c": "engine.limiter_curve",          # (?)

    # ── Chassis (.cdfbin) ────────────────────────────────────────────────────
    "e33a4f71": "chassis.mass_components",       # contains f32 ~930 @ offset 36 (kg?)
    "6f70f3c7": "chassis.aero_drag",             # f32 = 0.70 for NSX (?)
    "e8122311": "chassis.aero_curve",            # first aero coefficient ~ 0.12-0.60 (?)
    "06f458ac": "chassis.aero_scale_1",          # f32 = 1.0 (?)
    "3363edfd": "chassis.lift_base",             # f32 = 0.19 NSX (?)
    "67caa092": "chassis.aero_scale_2",          # f32 = 1.0 (?)
    "1f13c185": "chassis.aero_scale_3",          # f32 = 1.2 (?)
    "56e0a3ab": "chassis.lift_modifier",         # f32 = 0.20 NSX (?)
    "47d0b1de": "chassis.aero_scale_4",          # f32 = 1.0 (?)
    "e0a125de": "chassis.aero_cl",              # (Cd=0.20, Cl=0.50 for NSX?) (?)
    "e1763224": "chassis.drag_coef",             # f32 = 0.50 (?)
    "e0d9c85b": "chassis.tire_load_scale",       # f32 = 0.975 (?)
    "f33fd698": "chassis.aero_ref",              # f32 = 0.1202 (?)
    "a612f962": "chassis.aero_scale_5",          # f32 = 1.0 (?)
    "6b4ea077": "chassis.compound_aero",         # (?)

    # ── Suspension (.sdfbin) ─────────────────────────────────────────────────
    "670b57ab": "suspension.spring_rate",        # f32 values incl. ~12533 (unit TBD)
    "a213897d": "suspension.params",             # large mixed array (?)
    "bb b39f0b": "suspension.bump_stop",         # (?)
}

# Hashes we've confirmed as RPM-indexed curves (hash 8b0ab771 = engine torque)
CURVE_HASHES = {"8b0ab771", "515f5e83"}

# ─── Header layout ──────────────────────────────────────────────────────────

SHCB_MAGIC = b"ShCB"
SHCB_CONST = bytes.fromhex("e60fdd53")


class ShCBHeader:
    """Parsed ShCB file header."""

    def __init__(self, buf: bytes):
        if buf[:4] != SHCB_MAGIC:
            raise ValueError(f"Not a ShCB file (magic={buf[:4]!r})")
        self.version    = struct.unpack_from("<I", buf, 0x0C)[0]
        self.hdr_size   = struct.unpack_from("<I", buf, 0x18)[0] if self.version == 2 else 0x1C
        self.field_14   = struct.unpack_from("<I", buf, 0x14)[0]  # primary-block size
        if self.version == 2:
            self.field_20   = struct.unpack_from("<I", buf, 0x20)[0]  # secondary-block size
            self.sec_offset = struct.unpack_from("<I", buf, 0x24)[0]  # offset to secondary
        else:
            self.field_20   = len(buf) - self.hdr_size - self.field_14
            self.sec_offset = self.hdr_size + self.field_14

    def __repr__(self):
        return (f"ShCBHeader(ver={self.version}, hdr={self.hdr_size}, "
                f"pri={self.field_14}B, sec={self.field_20}B)")


# ─── Record ──────────────────────────────────────────────────────────────────

class ShCBRecord:
    """A single tagged record within the ShCB primary block."""

    __slots__ = ("hash_hex", "type_code", "body", "offset")

    def __init__(self, hash_hex: str, type_code: int, body: bytes, offset: int):
        self.hash_hex  = hash_hex
        self.type_code = type_code
        self.body      = body
        self.offset    = offset

    @property
    def name(self) -> str:
        return HASH_NAMES.get(self.hash_hex, f"unknown_{self.hash_hex}")

    @property
    def is_curve_header(self) -> bool:
        return (self.type_code & 0xF0) == 0x80

    @property
    def is_curve_item(self) -> bool:
        return (self.type_code & 0xF0) == 0x90

    def as_f32_list(self) -> List[float]:
        """Interpret body as a sequence of f32 values (4-byte aligned from byte 0)."""
        n = len(self.body) // 4
        return [struct.unpack_from("<f", self.body, i * 4)[0] for i in range(n)]

    def as_u32_list(self) -> List[int]:
        """Interpret body as a sequence of u32 values."""
        n = len(self.body) // 4
        return [struct.unpack_from("<I", self.body, i * 4)[0] for i in range(n)]

    def decode_scalar_u32_pair(self) -> Optional[Tuple[int, int]]:
        """Decode a type-0x52 two-u32 record → (val_a, val_b)."""
        if len(self.body) >= 8:
            return (struct.unpack_from("<I", self.body, 0)[0],
                    struct.unpack_from("<I", self.body, 4)[0])
        return None

    def decode_single_f32(self) -> Optional[float]:
        """Decode a type-0x21 single-f32 record."""
        if len(self.body) >= 4:
            return struct.unpack_from("<f", self.body, 0)[0]
        return None

    def decode_curve_point(self) -> Optional[Tuple[int, float, float]]:
        """
        Decode a type-0x93 curve data point.
        Returns (rpm, val1_f32, val2_f32) or None.
        Layout: [0]=subtype_byte, [1..4]=RPM_u32, [5..8]=f32_val1, [9..12]=f32_val2
        """
        if self.type_code != 0x93 or len(self.body) < 13:
            return None
        rpm  = struct.unpack_from("<I", self.body, 1)[0]
        val1 = struct.unpack_from("<f", self.body, 5)[0]
        val2 = struct.unpack_from("<f", self.body, 9)[0]
        return (rpm, val1, val2)

    def decode_multicol_curve_point(self) -> Optional[Tuple[int, List[float]]]:
        """
        Decode a type-0x96 multi-column curve data point.
        Returns (rpm, [f32×5]) or None.
        Layout: [0]=byte, [1..4]=RPM_u32, [5..24]=5×f32
        """
        if self.type_code != 0x96 or len(self.body) < 25:
            return None
        rpm    = struct.unpack_from("<I", self.body, 1)[0]
        values = [struct.unpack_from("<f", self.body, 5 + i * 4)[0] for i in range(5)]
        return (rpm, values)

    def __repr__(self):
        return (f"ShCBRecord(hash={self.hash_hex}, "
                f"type=0x{self.type_code:02x}, body[{len(self.body)}])")


# ─── Main file class ─────────────────────────────────────────────────────────

class ShCBFile:
    """
    Parses a ShCB physics file and provides access to all tagged records.

    Usage::

        sf = ShCBFile("path/to/car.edfbin")
        print(sf.dump())
        curve = sf.get_engine_torque_curve()
    """

    def __init__(self, path: str):
        self.path   = path
        self._buf   = open(path, "rb").read()
        self.header = ShCBHeader(self._buf)
        self.records: List[ShCBRecord] = []
        self._parse()

    # ── Parsing ───────────────────────────────────────────────────────────────

    def _parse(self):
        """Walk the primary block and collect all tagged records."""
        hdr = self.header
        pri = memoryview(self._buf)[hdr.hdr_size : hdr.sec_offset]
        pri_bytes = bytes(pri)

        off = 0
        while off < len(pri_bytes) - 5:
            if pri_bytes[off] != 0x24:
                off += 1
                continue
            hash_hex  = pri_bytes[off + 1 : off + 5].hex()
            type_code = pri_bytes[off + 5]
            # Body runs until the next 0x24 marker
            nxt = off + 6
            while nxt < len(pri_bytes) and pri_bytes[nxt] != 0x24:
                nxt += 1
            body = pri_bytes[off + 6 : nxt]
            self.records.append(ShCBRecord(hash_hex, type_code, body, off))
            off = nxt

    # ── Lookup helpers ────────────────────────────────────────────────────────

    def get_all(self, hash_hex: str) -> List[ShCBRecord]:
        """Return all records matching the given 8-char LE hash string."""
        return [r for r in self.records if r.hash_hex == hash_hex]

    def get_one(self, hash_hex: str) -> Optional[ShCBRecord]:
        """Return the first record matching the given hash, or None."""
        for r in self.records:
            if r.hash_hex == hash_hex:
                return r
        return None

    def unique_hashes(self) -> Dict[str, int]:
        """Return a dict of {hash_hex: count}."""
        counts: Dict[str, int] = {}
        for r in self.records:
            counts[r.hash_hex] = counts.get(r.hash_hex, 0) + 1
        return counts

    # ── High-level physics extractors ─────────────────────────────────────────

    def get_engine_torque_curve(self) -> List[Tuple[int, float, float]]:
        """
        Engine torque curve from hash 8b0ab771.

        Returns a list of (rpm, friction_torque, gross_torque) tuples,
        sorted by RPM.  Both torque values are in the game's internal unit
        (approximately Nm for the ICE-only contribution — see notes).

        Notes
        -----
        - 'friction_torque' is negative (internal engine losses).
        - 'gross_torque' is positive (combustion output).
        - Net ICE torque at any RPM ≈ gross + friction.
        - Electric-motor contribution (hybrids) is NOT included here.
        - Absolute scale appears to match real-world ICE Nm closely for NSX/P1.
        """
        pts = [r.decode_curve_point()
               for r in self.get_all("8b0ab771")
               if r.type_code == 0x93]
        return sorted((p for p in pts if p is not None), key=lambda x: x[0])

    def get_engine_scalars(self) -> Dict[str, Any]:
        """
        Scalar RPM/boost parameters from the engine file.

        Returns a dict with keys:
          idle_rpm_min, idle_rpm_max  – from hash 4d239754
          torque_peak_rpm_low, torque_peak_rpm_high – from hash 7902b6bd
          rev_limiter_rpm, rev_limiter_hysteresis   – from hash d39464af
          boost_pressure_max_pa, boost_pressure_low_pa – from hash 0571c719
        """
        result: Dict[str, Any] = {}

        def rpm_value(raw_u32: int, body: bytes, off: int) -> int:
            # Some cars store these RPM fields as f32 rather than u32
            # (e.g. bmw_1m idle = 0x44610000 = 900.0f, which reads as
            # 1147207680 when taken as an integer). No real RPM exceeds
            # 100k, so anything larger is a float bit pattern.
            if raw_u32 > 100_000:
                return int(round(struct.unpack_from("<f", body, off)[0]))
            return raw_u32

        def u32_pair(h, k1, k2):
            r = self.get_one(h)
            if r:
                p = r.decode_scalar_u32_pair()
                if p:
                    result[k1] = rpm_value(p[0], r.body, 0)
                    result[k2] = rpm_value(p[1], r.body, 4)

        u32_pair("4d239754", "idle_rpm_min",       "idle_rpm_max")
        u32_pair("7902b6bd", "torque_peak_rpm_low", "torque_peak_rpm_high")
        u32_pair("d39464af", "rev_limiter_rpm",     "rev_limiter_hysteresis")

        # Boost pressure (f32 pair at offsets 0 and 4)
        r = self.get_one("0571c719")
        if r and len(r.body) >= 8:
            result["boost_pressure_max_pa"] = struct.unpack_from("<f", r.body, 0)[0]
            result["boost_pressure_low_pa"] = struct.unpack_from("<f", r.body, 4)[0]

        return result

    def get_chassis_mass(self) -> Optional[float]:
        """
        Attempt to extract vehicle mass (kg) from the chassis file.

        Looks for f32 ~930 in hash e33a4f71 at body offset 36.
        The unit/calibration is tentative — validate against known specs.
        """
        r = self.get_one("e33a4f71")
        if r and len(r.body) >= 40:
            return struct.unpack_from("<f", r.body, 36)[0]
        return None

    def get_chassis_aero(self) -> Dict[str, Optional[float]]:
        """
        Aerodynamic coefficients from the chassis file (tentative).

        Keys: cd, cl (values may require normalization; units not yet confirmed).
        """
        result: Dict[str, Optional[float]] = {}
        r = self.get_one("e0a125de")
        if r and len(r.body) >= 8:
            result["cd"] = struct.unpack_from("<f", r.body, 0)[0]
            result["cl"] = struct.unpack_from("<f", r.body, 4)[0]
        return result

    def get_suspension_spring_rates(self) -> List[float]:
        """
        Spring-rate values from hash 670b57ab in a suspension file.

        Returns all f32 values in that record that are in [5000, 300_000]
        (assumed to be in N/m — unit calibration pending).
        """
        rates = []
        for r in self.get_all("670b57ab"):
            for i in range(0, len(r.body) - 3, 4):
                v = struct.unpack_from("<f", r.body, i)[0]
                if 5_000 <= v <= 300_000:
                    rates.append(v)
        return sorted(set(rates))

    # ── Human-readable dump ───────────────────────────────────────────────────

    def dump(self, show_body: bool = False) -> str:
        """
        Return a human-readable summary of all records.

        Parameters
        ----------
        show_body : bool
            If True, also print raw body hex for each record.
        """
        lines = []
        lines.append(f"ShCB file: {Path(self.path).name}")
        lines.append(f"  Header : {self.header}")
        lines.append(f"  Records: {len(self.records)} total, "
                     f"{len(self.unique_hashes())} unique hashes")
        lines.append("")

        # Group by hash
        from collections import defaultdict
        by_hash: Dict[str, List[ShCBRecord]] = defaultdict(list)
        for r in self.records:
            by_hash[r.hash_hex].append(r)

        for h, recs in sorted(by_hash.items(), key=lambda kv: -len(kv[1])):
            name  = HASH_NAMES.get(h, f"<unknown_{h}>")
            types = sorted({f"0x{r.type_code:02x}" for r in recs})
            lines.append(f"  [{h}] {name}  count={len(recs)}  types={types}")

            # Show decoded values for known types
            for r in recs[:3]:   # at most 3 examples per hash
                self._append_decoded(lines, r)
            if len(recs) > 3:
                lines.append(f"    ... +{len(recs)-3} more records")

            if show_body:
                for r in recs[:2]:
                    lines.append(f"    body[{len(r.body)}] {r.body.hex()}")

            lines.append("")

        return "\n".join(lines)

    def _append_decoded(self, lines: List[str], r: ShCBRecord):
        """Append a decoded line for a single record to `lines`."""
        t = r.type_code

        if t == 0x21 and len(r.body) >= 4:
            v = struct.unpack_from("<f", r.body, 0)[0]
            lines.append(f"    f32 = {v:.6g}")

        elif t == 0x22 and len(r.body) >= 8:
            v0 = struct.unpack_from("<f", r.body, 0)[0]
            v1 = struct.unpack_from("<f", r.body, 4)[0]
            lines.append(f"    f32 = [{v0:.6g}, {v1:.6g}]")

        elif t == 0x52 and len(r.body) >= 8:
            p = r.decode_scalar_u32_pair()
            lines.append(f"    u32 pair = {p}")

        elif t == 0x93:
            pt = r.decode_curve_point()
            if pt:
                rpm, v1, v2 = pt
                lines.append(f"    curve_pt: RPM={rpm}  val1={v1:.3f}  val2={v2:.3f}")

        elif t == 0x96:
            pt = r.decode_multicol_curve_point()
            if pt:
                rpm, vals = pt
                vals_str = "  ".join(f"{v:.4f}" for v in vals)
                lines.append(f"    curve_pt: RPM={rpm}  [{vals_str}]")

        elif t == 0x82 and len(r.body) >= 5:
            v = struct.unpack_from("<f", r.body, 1)[0]
            lines.append(f"    f32(+1) = {v:.6g}")

        elif t in (0xa2, 0xa3, 0x83, 0x86, 0x13, 0x03, 0x7b):
            # Generic: show up to 6 f32 windows that look physically plausible
            phys = []
            for k in range(0, len(r.body) - 3, 4):
                v = struct.unpack_from("<f", r.body, k)[0]
                if 1e-4 < abs(v) < 1e7 and v == v:  # finite and non-trivial
                    phys.append(f"{v:.5g}")
            if phys:
                lines.append(f"    plausible f32s: {', '.join(phys[:8])}")
            else:
                u32s = r.as_u32_list()
                if any(0 < x < 1_000_000 for x in u32s):
                    lines.append(f"    u32s: {u32s[:6]}")

        # else: skip (empty or unparsed body)


# ─── CLI ─────────────────────────────────────────────────────────────────────

def _cli():
    import argparse, sys
    sys.stdout.reconfigure(encoding="utf-8")

    ap = argparse.ArgumentParser(
        prog="shcb_reader",
        description="Dump / explore a Project CARS 2 ShCB physics binary file.",
    )
    ap.add_argument("files", nargs="+", metavar="FILE",
                    help=".edfbin / .cdfbin / .sdfbin / .gdfbin / .hdtbin file(s)")
    ap.add_argument("--body", action="store_true",
                    help="Also print raw body hex for each record")
    ap.add_argument("--engine", action="store_true",
                    help="Print structured engine summary (torque curve + scalars)")
    ap.add_argument("--chassis", action="store_true",
                    help="Print structured chassis summary (mass + aero)")
    ap.add_argument("--suspension", action="store_true",
                    help="Print structured suspension summary (spring rates)")
    args = ap.parse_args()

    for path in args.files:
        if not os.path.isfile(path):
            print(f"[ERROR] not found: {path}")
            continue
        try:
            sf = ShCBFile(path)
        except Exception as exc:
            print(f"[ERROR] {path}: {exc}")
            continue

        if args.engine:
            print(f"=== ENGINE: {Path(path).name} ===")
            scalars = sf.get_engine_scalars()
            print(f"  Idle RPM         : {scalars.get('idle_rpm_min','?')}-"
                  f"{scalars.get('idle_rpm_max','?')}")
            print(f"  Torque peak RPM  : {scalars.get('torque_peak_rpm_low','?')}-"
                  f"{scalars.get('torque_peak_rpm_high','?')}")
            print(f"  Rev limiter      : {scalars.get('rev_limiter_rpm','?')} "
                  f"(hyst {scalars.get('rev_limiter_hysteresis','?')} RPM)")
            bp = scalars.get("boost_pressure_max_pa")
            if bp:
                print(f"  Max boost        : {bp/1000:.1f} kPa ({bp/100000:.3f} bar)")
            curve = sf.get_engine_torque_curve()
            if curve:
                print(f"  Torque curve     : {len(curve)} points")
                print(f"  {'RPM':>6}  {'Friction':>10}  {'Gross':>10}  {'Net':>10}")
                for rpm, fric, gross in curve:
                    net = gross + fric
                    print(f"  {rpm:6d}  {fric:10.1f}  {gross:10.1f}  {net:10.1f}")
            print()

        elif args.chassis:
            print(f"=== CHASSIS: {Path(path).name} ===")
            mass = sf.get_chassis_mass()
            if mass is not None:
                print(f"  Mass component   : {mass:.1f} (unit TBD, ~kg)")
            aero = sf.get_chassis_aero()
            if aero.get("cd") is not None:
                print(f"  Aero (Cd, Cl)    : {aero['cd']:.4f}, {aero.get('cl', '?')}")
            print()

        elif args.suspension:
            print(f"=== SUSPENSION: {Path(path).name} ===")
            rates = sf.get_suspension_spring_rates()
            print(f"  Spring-rate candidates ({len(rates)}):")
            for v in rates:
                print(f"    {v:.0f} N/m")
            print()

        else:
            print(sf.dump(show_body=args.body))


if __name__ == "__main__":
    _cli()
