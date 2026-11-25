# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Instances for HW1: reading, writing, and checking the rules.

An instance (one counterexample) is a JSON file:

    {"obstacles": [[[x, y], [x, y], ...], ...],   # convex polygons, counter-clockwise
     "start": [x, y],
     "goal": [x, y]}

The robot is not in the file: it is your pair's robot (gen_hw1.py --robot).

Rules (validate_instance reports every rule an instance breaks):
* every point is a list [x, y] of two numbers (not strings, not true or false);
* every coordinate is a multiple of 0.001, written with at most three digits after
  the decimal point, and lies in the box [0, 20];
* at most 20 obstacles (zero is allowed);
* every obstacle is a convex polygon with 3 to 8 vertices in counter-clockwise order,
  without repeated vertices and without three consecutive collinear vertices;
* obstacles have pairwise-disjoint interiors (they may touch);
* start and goal are different points.
All checks use exact integer arithmetic on the 0.001 grid. A coordinate counts as a
multiple of 0.001 if it lies within 1e-12 (1e-9 grid units) of one, as every decimal
with at most three digits after the point does. parse_instance and load_instance snap
every coordinate to the grid, round(v * 1000) / 1000, so every tool sees the same exact
grid values.

The module also reads answers.txt of the submission, one line "KEY: text" per key of
ANSWER_KEYS (parse_answers).
"""
from __future__ import annotations

import json
import math
import numbers
import re
from pathlib import Path

GRID = 1000                # coordinates are multiples of 1 / GRID
GRID_TOL = 1e-9            # grid units: how far a float may lie from the grid (decimal-to-binary rounding)
BOX = (0.0, 20.0)
MAX_OBSTACLES = 20
MIN_VERTICES, MAX_VERTICES = 3, 8


def load_instance(path) -> dict:
    """Read an instance file; returns {"obstacles": [[(x, y), ...], ...], "start": (x, y), "goal": (x, y)},
    every coordinate snapped to the 0.001 grid."""
    raw = json.loads(Path(path).read_text(encoding="utf-8-sig"))   # -sig: accept a byte-order mark
    return parse_instance(raw)


def _is_number(v) -> bool:
    """A real number (int or float, numpy numbers included), but not a bool or a string."""
    return isinstance(v, numbers.Real) and not isinstance(v, bool)


def _is_point(p) -> bool:
    return isinstance(p, (list, tuple)) and len(p) == 2


def snap(v) -> float:
    """The coordinate v on the 0.001 grid: round(v * 1000) / 1000, the float of the nearest
    decimal with three digits after the point. Raises ValueError for anything but a finite number."""
    if not _is_number(v):
        raise ValueError(f"{_show(v)} is not a number")
    try:
        return round(float(v) * GRID) / GRID
    except (OverflowError, ValueError):
        raise ValueError(f"{_show(v)} is not a finite number") from None


def _snap_point(p, what) -> tuple[float, float]:
    if not _is_point(p):
        raise ValueError(f"{what} is not a point [x, y]")
    return snap(p[0]), snap(p[1])


def parse_instance(raw) -> dict:
    """The instance of a JSON object, as float tuples snapped to the 0.001 grid. Raises ValueError
    if raw does not have the shape of an instance (validate_instance says what is wrong)."""
    if not isinstance(raw, dict):
        raise ValueError("an instance is a JSON object with keys obstacles, start, goal")
    for key in ("obstacles", "start", "goal"):
        if key not in raw:
            raise ValueError(f"missing key {key!r}")
    if not isinstance(raw["obstacles"], (list, tuple)):
        raise ValueError("obstacles must be a list of polygons")
    obstacles = []
    for k, poly in enumerate(raw["obstacles"]):
        if not isinstance(poly, (list, tuple)):
            raise ValueError(f"obstacle {k} is not a list of points")
        obstacles.append([_snap_point(p, f"obstacle {k}, vertex {i}") for i, p in enumerate(poly)])
    return {"obstacles": obstacles,
            "start": _snap_point(raw["start"], "start"),
            "goal": _snap_point(raw["goal"], "goal")}


def save_instance(path, inst) -> None:
    """Write an instance file (UTF-8, one obstacle per line), creating its folder (for example
    counterexamples/) if it is missing."""
    out = {"obstacles": [[[fmt(x), fmt(y)] for x, y in poly] for poly in inst["obstacles"]],
           "start": [fmt(inst["start"][0]), fmt(inst["start"][1])],
           "goal": [fmt(inst["goal"][0]), fmt(inst["goal"][1])]}
    lines = ["{", ' "obstacles": [']
    lines += ["  " + json.dumps(poly) + ("," if k < len(out["obstacles"]) - 1 else "")
              for k, poly in enumerate(out["obstacles"])]
    lines += [" ],", ' "start": ' + json.dumps(out["start"]) + ",", ' "goal": ' + json.dumps(out["goal"]), "}"]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def fmt(v):
    """A grid coordinate as an int or a float with at most three decimals."""
    q = round(float(v) * GRID)
    return q // GRID if q % GRID == 0 else q / GRID


def _grid(v):
    """Integer grid coordinate of v, or None if v is not a finite real number within GRID_TOL
    grid units of a multiple of 0.001."""
    if not _is_number(v):
        return None
    try:
        v = float(v)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(v):
        return None
    q = round(v * GRID)
    return q if abs(v * GRID - q) <= GRID_TOL else None


def _show(v) -> str:
    text = repr(v)
    return text if len(text) <= 40 else text[:37] + "..."


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _interiors_disjoint(A, B) -> bool:
    """Convex polygons A, B (integer CCW vertex lists): True iff their interiors are disjoint.
    Separating-axis test with exact integers: some edge line of A or B has the other polygon
    entirely on its closed outer side."""
    for P, Q in ((A, B), (B, A)):
        n = len(P)
        for i in range(n):
            a, b = P[i], P[(i + 1) % n]
            if all(_cross(a, b, q) <= 0 for q in Q):   # q on or to the right of the CCW edge a->b
                return True
    return False


def _grid_point(p, what, problems):
    """The integer grid point of p, or None after appending what is wrong with it to problems."""
    if not _is_point(p):
        problems.append(f"{what}: not a point [x, y] (a list of two numbers), found {_show(p)}")
        return None
    odd = [v for v in p if not _is_number(v)]
    if odd:
        problems.append(f"{what}: coordinates must be numbers, found {_show(odd[0])}")
        return None
    x, y = _grid(p[0]), _grid(p[1])
    if x is None or y is None:
        problems.append(f"{what}: coordinates must be multiples of 0.001, with at most three digits after "
                        f"the decimal point, found {_show(p[0] if x is None else p[1])}")
        return None
    lo, hi = round(BOX[0] * GRID), round(BOX[1] * GRID)
    if not (lo <= x <= hi and lo <= y <= hi):
        problems.append(f"{what}: outside the box [0, 20] x [0, 20]")
    return (x, y)


def _grid_polygon(poly, tag, problems, max_vertices=MAX_VERTICES):
    """The integer vertices of a convex counter-clockwise polygon, or None after appending what is
    wrong with it to problems (the obstacle rules, with at most max_vertices vertices)."""
    if not isinstance(poly, (list, tuple)) or not MIN_VERTICES <= len(poly) <= max_vertices:
        problems.append(f"{tag}: needs {MIN_VERTICES} to {max_vertices} vertices")
        return None
    pts = [_grid_point(p, f"{tag}, vertex {i}", problems) for i, p in enumerate(poly)]
    if any(p is None for p in pts):
        return None
    n = len(pts)
    if len(set(pts)) != n:
        problems.append(f"{tag}: repeated vertex")
        return None
    turns = [_cross(pts[i], pts[(i + 1) % n], pts[(i + 2) % n]) for i in range(n)]
    if any(t == 0 for t in turns):
        problems.append(f"{tag}: three consecutive vertices are collinear")
        return None
    if any(t < 0 for t in turns):
        problems.append(f"{tag}: not convex, or not in counter-clockwise order")
        return None
    # a polygon whose every turn is a left turn can still wind around twice
    if _winding(pts) != 1:
        problems.append(f"{tag}: not a simple convex polygon")
        return None
    return pts


def validate_instance(inst) -> list[str]:
    """Every rule the instance breaks, as a list of messages; [] means valid."""
    problems = []
    try:
        if not isinstance(inst, dict):
            raise ValueError
        obstacles, start, goal = inst["obstacles"], inst["start"], inst["goal"]
    except Exception:
        return ["an instance needs the keys obstacles, start, goal"]
    s, g = _grid_point(start, "start", problems), _grid_point(goal, "goal", problems)
    if s is not None and s == g:
        problems.append("start and goal must be different points")
    if not isinstance(obstacles, (list, tuple)):
        return problems + ["obstacles must be a list of polygons"]
    if len(obstacles) > MAX_OBSTACLES:
        problems.append(f"at most {MAX_OBSTACLES} obstacles (found {len(obstacles)})")
    polys = []
    for k, poly in enumerate(obstacles):
        pts = _grid_polygon(poly, f"obstacle {k}", problems)
        if pts is not None:
            polys.append((k, pts))
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            (ki, A), (kj, B) = polys[i], polys[j]
            if not _interiors_disjoint(A, B):
                problems.append(f"obstacles {ki} and {kj} overlap (their interiors intersect)")
    return problems


def _dir(a, b):
    return (b[0] - a[0], b[1] - a[1])


def _winding(pts) -> int:
    """Number of full turns of the edge direction along the polygon (1 for a simple convex polygon)."""
    total = 0.0
    n = len(pts)
    for i in range(n):
        d1, d2 = _dir(pts[i - 1], pts[i]), _dir(pts[i], pts[(i + 1) % n])
        total += math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1[0] * d2[0] + d1[1] * d2[1])
    return round(total / (2 * math.pi))


# =============================================================================
# Part B and C: answers.txt (one line per key)
# =============================================================================
ANSWER_KEYS = tuple(f"P{k}.{w}" for k in range(1, 9) for w in ("principle", "guarantee")) + ("T1.planner",)
PLANNER_KEYS = ("T1.planner",)
SKIP_T1 = "-"              # T1.planner: - means that you skip T1 (it then scores 0)
MAX_ANSWER_CHARS = 400
_ANSWER_LINE = re.compile(r"^\s*([A-Za-z0-9_.()]+)\s*:(.*)$")
_CANON = {k.lower(): k for k in ANSWER_KEYS}


def answer_is_blank(value) -> bool:
    """A blank answer: missing, empty, or "-" (the value for a planner you judged correct)."""
    return value is None or str(value).strip() in ("", "-")


def parse_answers(data):
    """Read answers.txt (bytes or str). Returns ({key: value} for the keys found, the problems).

    Every line is a comment (its first non-blank character is #), blank, or "KEY: text", with
    KEY one of ANSWER_KEYS (P1.principle, P1.guarantee, ..., P8.guarantee, T1.planner; the case
    of a key does not matter). Every key appears exactly once; a value fits on its line and has
    at most 400 characters; "-" (or nothing) marks a blank answer, the value for a planner you
    judged correct; T1.planner names one planner, P1 ... P8, or is "-" if you skip T1 (the
    skeleton's placeholder P? is a problem)."""
    problems = []
    if isinstance(data, bytes):
        if data.startswith((b"\xff\xfe", b"\xfe\xff")) or (b"\x00" in data):
            return {}, ["answers.txt is not UTF-8 text (UTF-16?): save it as UTF-8 (in Windows PowerShell 5.1, "
                        "> and Out-File write UTF-16)"]
        try:
            text = data.decode("utf-8-sig")
        except UnicodeDecodeError:
            return {}, ["answers.txt is not UTF-8 text: save it as UTF-8"]
    else:
        text = str(data)
    values, where = {}, {}
    for n, line in enumerate(text.splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        m = _ANSWER_LINE.match(line)
        key = _CANON.get(m.group(1).lower()) if m else None
        if key is None:
            shown = line.strip() if len(line.strip()) <= 40 else line.strip()[:37] + "..."
            problems.append(f"answers.txt, line {n}: {shown!r} is not 'KEY: text' with a known key (an answer must "
                            "fit on one line; comment lines start with #)")
            continue
        if key in values:
            problems.append(f"answers.txt, line {n}: {key} appears twice (first on line {where[key]})")
            continue
        value = m.group(2).strip()
        if len(value) > MAX_ANSWER_CHARS:
            problems.append(f"answers.txt, line {n}: the answer to {key} has {len(value)} characters, "
                            f"more than {MAX_ANSWER_CHARS}")
        if key in PLANNER_KEYS:
            if re.fullmatch(r"[Pp][1-8]", value):
                value = value.upper()
            elif value == SKIP_T1:
                pass
            elif value.upper() == "P?":
                problems.append(f"answers.txt, line {n}: {key} is still the skeleton's placeholder {value!r}: name the "
                                "planner of your proof T1 (one of P1 ... P8), or write - if you skip T1")
            else:
                problems.append(f"answers.txt, line {n}: {key} must name one planner, P1 ... P8, or be - if you "
                                f"skip T1 (found {value!r})")
        values[key], where[key] = value, n
    missing = [k for k in ANSWER_KEYS if k not in values]
    if missing:
        problems.append("answers.txt lacks the key(s) " + ", ".join(missing) + ' (write "-" for a blank answer)')
    return values, problems
