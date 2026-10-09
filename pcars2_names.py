#!/usr/bin/env python3
"""
pcars2_names.py
===============
Shared name matching for pCARS2 car names vs. the various file-name schemes
the game uses (physics .edfbin/.sdfbin/... in snake_case, vehicle .bff
archives in PascalCase, .vdfm descriptors with their own variants).

Plain prefix matching ("first file starting with the first N chars") picks
the alphabetically first sibling, which silently pairs unrelated cars, e.g.
'aston_martin_db11' -> 'aston_martin_dbr1_1959' or
'lamborghini_huracan_lp610-4' -> 'Lamborghini_Aventador'. Token scoring
compares the model tokens too, so a shared brand prefix alone never wins.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable, List, Tuple


def name_tokens(name: str) -> List[str]:
    """Split a car/file name into lowercase tokens on '_' and '-'."""
    return [t for t in re.split(r"[_\-]", name.lower()) if t]


def norm_name(name: str) -> str:
    """Lowercase with separators stripped, for exact-equivalence checks."""
    return name.lower().replace("_", "").replace("-", "")


def score_candidate(car: str, candidate: str) -> Tuple[int, int]:
    """
    Return (matched, extra) for a candidate name against a car name.

    matched: car tokens that appear inside the candidate's normalized name.
    extra:   candidate tokens not covered by any car token (a candidate token
             is covered if it contains, or is contained by, a car token).
    """
    car_tokens = name_tokens(car)
    cand_norm = norm_name(candidate)

    def covers(a: str, b: str) -> bool:
        # Substring coverage, but a 1-char token (the 'r' in 'type-r', the
        # 'f' in 'f-type') must match exactly — otherwise it covers anything.
        if a == b:
            return True
        return (len(a) >= 2 and a in b) or (len(b) >= 2 and b in a)

    matched = sum(1 for t in car_tokens
                  if (t in name_tokens(candidate)) or (len(t) >= 2 and t in cand_norm))
    extra = sum(
        1 for st in name_tokens(candidate)
        if not any(covers(st, ct) for ct in car_tokens)
    )
    return matched, extra


def is_base_variant(car: str, candidate: str) -> bool:
    """
    True when `candidate` names a base version of `car`: its normalized name
    is a prefix of the car's, ending on a token boundary (optionally one
    trailing letter into the next token, for suffixes like 'dw12' -> 'dw12c'
    or 'lotus_49' -> 'lotus_49c'), and it covers more than the brand alone.

      ford_fusion_nascar13  -> ford_fusion_nascar13_daytona   yes
      lamborghini_aventador -> lamborghini_aventador_lp700-4  yes
      toyota_gt86           -> toyota_gt-one                  no
      ferrari_laferrari     -> ferrari_250_tr                 no
      acura_nsx_gt3         -> acura_nsx_2017                 no
    """
    car_tokens = name_tokens(car)
    car_norm = "".join(car_tokens)
    cand_norm = norm_name(candidate)
    if not car_norm.startswith(cand_norm) or len(cand_norm) <= len(car_tokens[0]):
        return False
    boundaries, pos = set(), 0
    for t in car_tokens:
        pos += len(t)
        boundaries.add(pos)
    n = len(cand_norm)
    if n in boundaries:
        return True
    return (n + 1) in boundaries and car_norm[n].isalpha()


def rank_candidates(car: str, paths: Iterable[Path],
                    strict: bool = False) -> List[Path]:
    """
    Rank candidate files for a car name, best first.

    An exact normalized-name match always wins on its own.

    strict=False (mesh archives): token scoring, matched - extra > 0.
    strict=True (physics files): only base variants (is_base_variant), longest
    first. A physics file from a different spec of car is worse than defaults
    — e.g. 'acura_nsx_2017' must not borrow the GT3's springs.
    """
    car_norm = norm_name(car)
    scored: List[Tuple[int, str, Path]] = []
    for p in paths:
        if norm_name(p.stem) == car_norm:
            return [p]
        if strict:
            if is_base_variant(car, p.stem):
                scored.append((len(norm_name(p.stem)), p.stem.lower(), p))
            continue
        matched, extra = score_candidate(car, p.stem)
        if matched == 0:
            continue
        score = matched - extra
        if score > 0:
            scored.append((score, p.stem.lower(), p))
    scored.sort(key=lambda x: (-x[0], x[1]))
    return [p for _, _, p in scored]
