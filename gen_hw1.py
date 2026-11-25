# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Your pair's robot and warm-up obstacle for HW1.

Usage
-----
  python gen_hw1.py --ids ID1 [ID2] --robot [--json]
      Print your robot R: a right triangle given by its vertices relative to its
      reference point, which is the right-angle vertex.
  python gen_hw1.py --ids ID1 [ID2] --warmup [--json]
      Print the obstacle of the Part A warm-up (compute its C-obstacle by hand).

The IDs are sorted internally, so their order does not matter; a student working
alone passes one ID. Pass each ID as its own argument (--ids 123456789 987654321),
not joined by a comma and not inside one pair of quotes. An ID has digits only, at
most nine; one shorter than nine digits gets its leading zeros back. Everything is
deterministic. The tool warns (and still runs) when a DO-NOT-EDIT file of the kit
differs from the original recorded in MANIFEST.json.

Importable: ``normalize_ids(ids)``, ``robot_for_ids(ids)``, ``warmup_obstacle(ids)``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import random
import re
import sys
from pathlib import Path

LEGS = (0.4, 0.5, 0.6, 0.7, 0.8)                        # leg lengths of the right triangle
ORIENTATIONS = ((1, 1), (-1, 1), (-1, -1), (1, -1))     # signs of the two legs
ID_DIGITS = 9                                           # an ID has at most nine digits


def normalize_id(i) -> str:
    """One ID as nine digits (a shorter all-digit ID gets its leading zeros back). Raises
    ValueError, saying what is wrong, for anything but one ID of at most nine digits 0-9."""
    s = str(i).strip()
    if not s:
        raise ValueError("an ID is empty: pass one or two IDs, e.g. --ids 123456789 987654321")
    if re.search(r"[,;\s]", s):
        raise ValueError(f"{s!r} contains a comma, a semicolon, or a space: pass each ID as its own argument, "
                         "separated by a space (--ids 123456789 987654321), without a comma and without quotes "
                         "around both")
    if s[0] in "'\"" or s[-1] in "'\"":
        raise ValueError(f"the ID {s} is quoted: pass the digits alone (--ids 123456789 987654321)")
    if not re.fullmatch(r"[0-9]+", s):
        raise ValueError(f"the ID {s!r} is not all digits: pass your ID, at most {ID_DIGITS} digits 0-9")
    if len(s) > ID_DIGITS:
        raise ValueError(f"the ID {s} has {len(s)} digits; an ID has at most {ID_DIGITS} digits (if you joined two "
                         "IDs, pass them as two arguments: --ids 123456789 987654321)")
    return s.zfill(ID_DIGITS)


def normalize_ids(ids) -> list[str]:
    """The sorted, nine-digit IDs of a student or a pair (one or two different IDs); raises
    ValueError."""
    if isinstance(ids, (str, int)):
        ids = [ids]
    ids = list(ids)
    if not 1 <= len(ids) <= 2:
        raise ValueError(f"pass one or two IDs, not {len(ids)}")
    out = sorted(normalize_id(i) for i in ids)
    if len(out) == 2 and out[0] == out[1]:
        raise ValueError(f"the two IDs are the same ({out[0]}): a student working alone passes one ID "
                         f"(--ids {out[0]})")
    return out


def rng_for(ids, purpose: str) -> random.Random:
    key = "|".join(normalize_ids(ids)) + "|" + purpose
    return random.Random(int(hashlib.sha256(key.encode()).hexdigest()[:16], 16))


def _ccw(pts):
    (ax, ay), (bx, by), (cx, cy) = pts
    return pts if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) > 0 else [pts[0], pts[2], pts[1]]


def robot_for_ids(ids) -> list[tuple[float, float]]:
    """The pair's robot: right triangle (0,0), (sx*a, 0), (0, sy*b), counter-clockwise."""
    rng = rng_for(ids, "robot")
    a, b = rng.choice(LEGS), rng.choice(LEGS)
    sx, sy = rng.choice(ORIENTATIONS)
    return _ccw([(0.0, 0.0), (sx * a, 0.0), (0.0, sy * b)])


def warmup_obstacle(ids) -> list[tuple[float, float]]:
    """Part A warm-up, counter-clockwise: an axis-aligned unit square, a square turned by 45 degrees
    whose diagonals are 1.5 long (side about 1.06), or a right triangle with legs 1.5 and 1."""
    rng = rng_for(ids, "warmup")
    x0, y0 = rng.randrange(8, 64) / 4, rng.randrange(8, 64) / 4
    kind = rng.randrange(3)
    if kind == 0:
        pts = [(x0, y0), (x0 + 1, y0), (x0 + 1, y0 + 1), (x0, y0 + 1)]
    elif kind == 1:
        pts = [(x0, y0), (x0 + 0.75, y0 + 0.75), (x0, y0 + 1.5), (x0 - 0.75, y0 + 0.75)]
    else:
        pts = [(x0, y0), (x0 + 1.5, y0), (x0, y0 + 1)]
    return [(float(x), float(y)) for x, y in pts]


def _fmt(p):
    return "(" + ", ".join(f"{v:g}" for v in p) + ")"


def _kit_warning():
    """check_ce.kit_warning() of this kit (imported here only: the module itself is stdlib only)."""
    here = str(Path(__file__).resolve().parent)
    if here not in sys.path:
        sys.path.insert(0, here)
    try:
        import check_ce
        return check_ce.kit_warning()
    except Exception as exc:
        return f"WARNING: the kit could not compare its files with MANIFEST.json ({type(exc).__name__}: {exc})"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="HW1: your robot and the Part A warm-up obstacle (236767).")
    ap.add_argument("--ids", nargs="+", required=True, metavar="ID")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--robot", action="store_true", help="print your robot")
    mode.add_argument("--warmup", action="store_true", help="print the Part A warm-up obstacle")
    ap.add_argument("--json", action="store_true", help="print JSON")
    a = ap.parse_args(argv)
    try:
        ids = normalize_ids(a.ids)
    except ValueError as exc:
        ap.error(str(exc))
    warning = _kit_warning()
    if warning:   # stdout stays pure JSON with --json
        print(warning, file=sys.stderr if a.json else sys.stdout, flush=True)
    robot = robot_for_ids(ids)
    if a.robot:
        if a.json:
            print(json.dumps({"ids": ids, "robot": [list(p) for p in robot]}))
        else:
            print(f"Robot of {' '.join(ids)}: triangle " + " ".join(_fmt(p) for p in robot)
                  + ", reference point (0, 0) at the right angle.")
        return 0
    obstacle = warmup_obstacle(ids)
    if a.json:
        print(json.dumps({"ids": ids, "robot": [list(p) for p in robot], "obstacle": [list(p) for p in obstacle]}))
    else:
        print(f"Part A warm-up for {' '.join(ids)}: obstacle O = " + " ".join(_fmt(p) for p in obstacle))
        print("Your robot R: " + " ".join(_fmt(p) for p in robot) + ". Compute the C-obstacle O (+) (-R) by hand,")
        print("then compare with: python draw.py --ids ... --warmup")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
