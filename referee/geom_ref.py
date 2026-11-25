# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Geometry of the referee (236767 HW1).

Conventions:

* Points are (x, y) float tuples; polygons are lists of points, CCW, no repeated
  closing point.  The functions below also accept CW input and redundant
  (duplicate or collinear) vertices; they normalize internally.
* A point is in the *interior* of a polygon only if it lies inside it by more
  than ``eps`` (its distance to the boundary exceeds ``eps``).
* A segment is valid if none of its points is in the interior of any
  C-obstacle.  Touching a boundary and sliding along an edge are allowed.

C-obstacles are Minkowski sums of convex sets, hence convex.  ``segment_valid``,
``path_valid``, ``visibility_vertices``, and ``count_visibility_edges`` require
convex polygons and raise ``ValueError`` on a non-convex one;
``point_in_interior`` works for any simple polygon.

Segment test (Cyrus-Beck with a tolerance).  For a convex polygon with unit
inward normals n_i and offsets c_i, the signed distance of x to edge line i is
s_i(x) = n_i . x - c_i, and x is interior by more than eps iff min_i s_i(x) > eps.
Along x(t) = p + t (q - p), each s_i is affine in t, so the set
{t in [0, 1] : s_i(t) > eps for all i} is an (open) interval that we compute
exactly as in Cyrus-Beck clipping.  The segment is invalid for that polygon iff
the interval is non-empty.  No sampling, no special cases for vertices.
"""

from __future__ import annotations

import math

import numpy as np

EPS = 1e-9
COLLINEAR_TOL = 1e-9  # distance of a vertex from the line through its neighbours

__all__ = [
    "EPS",
    "canonical_corners",
    "point_in_interior",
    "segment_valid",
    "segments_valid",
    "visibility_vertices",
    "visibility_edges",
    "count_visibility_edges",
    "path_valid",
    "path_length",
]


# ---------------------------------------------------------------------------
# Polygon normalization
# ---------------------------------------------------------------------------

def _as_array(polygon) -> np.ndarray:
    pts = np.asarray([(float(p[0]), float(p[1])) for p in polygon], dtype=float)
    if pts.ndim != 2 or (pts.size and pts.shape[1] != 2):
        raise ValueError("polygon must be a list of (x, y) points")
    if pts.size and not np.all(np.isfinite(pts)):
        raise ValueError("polygon has non-finite coordinates")
    return pts.reshape(-1, 2)


def _signed_area(pts: np.ndarray) -> float:
    if len(pts) < 3:
        return 0.0
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def _strip(pts: np.ndarray, tol: float) -> np.ndarray:
    """Drop cyclically repeated points (within tol) and collinear points
    (distance from the line through the two neighbours at most tol)."""
    pts = [tuple(p) for p in pts]
    changed = True
    while changed and len(pts) >= 3:
        changed = False
        # repeated points
        out = []
        for p in pts:
            if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > tol:
                out.append(p)
        while len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= tol:
            out.pop()
        if len(out) != len(pts):
            changed = True
        pts = out
        if len(pts) < 3:
            break
        # collinear points: remove one at a time, then rescan
        n = len(pts)
        for i in range(n):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % n]
            ax, ay = c[0] - a[0], c[1] - a[1]
            L = math.hypot(ax, ay)
            if L <= tol:
                # a and c coincide: b is a spike tip; drop it
                pts.pop(i)
                changed = True
                break
            dist = abs(ax * (b[1] - a[1]) - ay * (b[0] - a[0])) / L
            if dist <= tol:
                pts.pop(i)
                changed = True
                break
    return np.asarray(pts, dtype=float).reshape(-1, 2)


def canonical_corners(polygon, tol: float = COLLINEAR_TOL) -> list[tuple[float, float]]:
    """Corners only (no duplicate or collinear vertices), CCW, starting from the
    lowest vertex (the leftmost of the lowest on ties within ``tol``)."""
    pts = _as_array(polygon)
    if len(pts) == 0:
        return []
    pts = _strip(pts, tol)
    if len(pts) >= 3 and _signed_area(pts) < 0:
        pts = pts[::-1]
    if len(pts) == 0:
        return []
    ymin = pts[:, 1].min()
    cand = np.nonzero(pts[:, 1] <= ymin + tol)[0]
    start = int(cand[np.argmin(pts[cand, 0])])
    pts = np.roll(pts, -start, axis=0)
    return [(float(x), float(y)) for x, y in pts]


def _halfplanes(polygon):
    """Unit inward normals (k, 2), offsets (k,), bbox (4,) of a convex polygon.
    Returns None for a polygon with empty interior (fewer than three corners)."""
    pts = np.asarray(canonical_corners(polygon), dtype=float).reshape(-1, 2)
    if len(pts) < 3 or abs(_signed_area(pts)) <= 0.0:
        return None
    e = np.roll(pts, -1, axis=0) - pts
    L = np.hypot(e[:, 0], e[:, 1])
    turn = e[:, 0] * np.roll(e[:, 1], -1) - e[:, 1] * np.roll(e[:, 0], -1)
    # a right turn larger than rounding noise means a reflex corner
    if np.any(turn < -1e-12 * (L * np.roll(L, -1))):
        raise ValueError("C-obstacle is not convex; the reference segment test needs convex polygons")
    n = np.stack([-e[:, 1], e[:, 0]], axis=1) / L[:, None]  # left normal = inward for CCW
    c = np.einsum("ij,ij->i", n, pts)
    bbox = np.array([pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()])
    return n, c, bbox


class _Obstacles:
    """Half-plane representation of a list of convex C-obstacles."""

    def __init__(self, c_obstacles, eps: float = EPS):
        self.eps = float(eps)
        self.polys = []
        for poly in c_obstacles:
            hp = _halfplanes(poly)
            if hp is not None:
                self.polys.append(hp)

    def interior_any(self, P: np.ndarray) -> np.ndarray:
        """Bool (n,): point P[k] is in the interior of some C-obstacle."""
        P = np.asarray(P, dtype=float).reshape(-1, 2)
        out = np.zeros(len(P), dtype=bool)
        for n, c, _ in self.polys:
            s = P @ n.T - c
            out |= s.min(axis=1) > self.eps
        return out

    def segments_valid(self, P: np.ndarray, Q: np.ndarray) -> np.ndarray:
        """Bool (n,): segment P[k]Q[k] has no point in the interior of any C-obstacle."""
        P = np.asarray(P, dtype=float).reshape(-1, 2)
        Q = np.asarray(Q, dtype=float).reshape(-1, 2)
        valid = np.ones(len(P), dtype=bool)
        if len(P) == 0:
            return valid
        xlo = np.minimum(P[:, 0], Q[:, 0])
        xhi = np.maximum(P[:, 0], Q[:, 0])
        ylo = np.minimum(P[:, 1], Q[:, 1])
        yhi = np.maximum(P[:, 1], Q[:, 1])
        eps = self.eps
        for n, c, (bx0, bx1, by0, by1) in self.polys:
            # an interior point lies strictly inside the bbox by more than eps
            cand = valid & (xhi > bx0) & (xlo < bx1) & (yhi > by0) & (ylo < by1)
            idx = np.nonzero(cand)[0]
            if idx.size == 0:
                continue
            s0 = P[idx] @ n.T - c
            s1 = Q[idx] @ n.T - c
            valid[idx[_interior_hit(s0, s1, eps)]] = False
        return valid


def _interior_hit(s0: np.ndarray, s1: np.ndarray, eps: float) -> np.ndarray:
    """For each row (one segment against one convex polygon, columns = edges):
    is there t in [0, 1] with s0 + t (s1 - s0) > eps in every column?"""
    ds = s1 - s0
    num = eps - s0
    with np.errstate(divide="ignore", invalid="ignore"):
        t = num / ds
    lo = np.where(ds > 0, t, -np.inf).max(axis=1)   # need t > lo
    hi = np.where(ds < 0, t, np.inf).min(axis=1)    # need t < hi
    blocked = ((ds == 0) & (s0 <= eps)).any(axis=1)  # parallel and never deep enough
    return (~blocked) & (lo < hi) & (lo < 1.0) & (hi > 0.0)


# ---------------------------------------------------------------------------
# Public predicates
# ---------------------------------------------------------------------------

def point_in_interior(p, poly, eps: float = EPS) -> bool:
    """True iff p lies inside the simple polygon ``poly`` and its distance to the
    boundary exceeds ``eps``.  Works for convex and non-convex polygons."""
    x, y = float(p[0]), float(p[1])
    pts = _strip(_as_array(poly), COLLINEAR_TOL)
    k = len(pts)
    if k < 3:
        return False
    inside = False
    dmin = math.inf
    for i in range(k):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % k]
        # distance to edge
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
        dmin = min(dmin, math.hypot(x - (ax + t * ex), y - (ay + t * ey)))
        # even-odd ray casting
        if (ay > y) != (by > y):
            xc = ax + (y - ay) * ex / ey
            if xc > x:
                inside = not inside
    return inside and dmin > eps


def segment_valid(p, q, c_obstacles, eps: float = EPS) -> bool:
    """True iff no point of segment pq is in the interior of any (convex) C-obstacle."""
    obs = _Obstacles(c_obstacles, eps)
    return bool(obs.segments_valid(np.array([p], float), np.array([q], float))[0])


def segments_valid(P, Q, c_obstacles, eps: float = EPS) -> np.ndarray:
    """Vectorized ``segment_valid`` for arrays of endpoints P, Q of shape (n, 2)."""
    return _Obstacles(c_obstacles, eps).segments_valid(P, Q)


# ---------------------------------------------------------------------------
# Visibility graph
# ---------------------------------------------------------------------------

def _merge_points(points: list[tuple[float, float]], tol: float):
    """Greedy clustering of points within ``tol``; the first point of each
    cluster is its representative.  Returns (representatives, labels)."""
    if not points:
        return [], np.zeros(0, dtype=int)
    A = np.asarray(points, dtype=float)
    labels = -np.ones(len(A), dtype=int)
    reps = []
    # sort by x so that only a window needs checking
    order = np.argsort(A[:, 0], kind="stable")
    xs = A[order, 0]
    for pos in range(len(order)):
        i = order[pos]
        if labels[i] >= 0:
            continue
        # keep the earliest index (source, dest first) as representative
        hi = np.searchsorted(xs, A[i, 0] + tol, side="right")
        lo = np.searchsorted(xs, A[i, 0] - tol, side="left")
        window = order[lo:hi]
        close = window[(labels[window] < 0) & (np.hypot(A[window, 0] - A[i, 0], A[window, 1] - A[i, 1]) <= tol)]
        r = int(close.min())
        lab = len(reps)
        reps.append((float(A[r, 0]), float(A[r, 1])))
        labels[close] = lab
    return reps, labels


def visibility_edges(c_obstacles, source=None, dest=None, eps: float = EPS):
    """Visibility graph of the C-obstacles, with source and dest if given.

    Vertices: every C-obstacle vertex (as given) that is not in the interior of
    a C-obstacle, plus ``source`` and ``dest`` when given and not interior;
    points within ``eps`` are one vertex (source and dest keep their exact
    coordinates as representatives).  Edges: every pair of distinct vertices
    whose segment is valid.  If source equals dest (within eps), is valid, and
    has no other incident edge, the self-loop (source, source) is returned so
    that the vertex is present in the edge list.

    Returns (edges, vertices).
    """
    obs = _Obstacles(c_obstacles, eps)
    pts: list[tuple[float, float]] = []
    s = d = None
    if source is not None:
        s = (float(source[0]), float(source[1]))
        pts.append(s)
    if dest is not None:
        d = (float(dest[0]), float(dest[1]))
        pts.append(d)
    for poly in c_obstacles:
        for v in poly:
            pts.append((float(v[0]), float(v[1])))
    reps, _ = _merge_points(pts, eps)
    if not reps:
        return [], []
    V = np.asarray(reps, dtype=float)
    keep = ~obs.interior_any(V)
    V = V[keep]
    verts = [reps[k] for k in np.nonzero(keep)[0]]
    nv = len(verts)
    edges: list[tuple[tuple[float, float], tuple[float, float]]] = []
    if nv >= 2:
        I, J = np.triu_indices(nv, k=1)
        ok = obs.segments_valid(V[I], V[J])
        edges = [(verts[i], verts[j]) for i, j in zip(I[ok].tolist(), J[ok].tolist())]
    if s is not None and d is not None and math.hypot(s[0] - d[0], s[1] - d[1]) <= eps:
        if s in verts and not any(s in e for e in edges):
            edges.append((s, s))
    return edges, verts


def visibility_vertices(c_obstacles, eps: float = EPS) -> list[tuple[float, float]]:
    """C-obstacle vertices not in the interior of any C-obstacle (duplicates merged)."""
    return visibility_edges(c_obstacles, None, None, eps)[1]


def count_visibility_edges(c_obstacles, eps: float = EPS) -> int:
    """Number of visibility-graph edges, start and goal excluded."""
    return len(visibility_edges(c_obstacles, None, None, eps)[0])


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

def path_length(path) -> float:
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])))


def path_valid(path, start, goal, c_obstacles, eps: float = EPS, *, endpoint_tol: float = 1e-6) -> bool:
    """The path starts at ``start`` and ends at ``goal`` (within ``endpoint_tol``),
    has finite coordinates, and every segment (and, for a one-point path, the
    point itself) is valid.  Never raises on malformed input; returns False."""
    obs = _Obstacles(c_obstacles, eps)  # malformed C-obstacles raise ValueError
    try:
        if path is None:
            return False
        P = np.asarray([(float(p[0]), float(p[1])) for p in path], dtype=float).reshape(-1, 2)
        if len(P) == 0 or not np.all(np.isfinite(P)):
            return False
        sx, sy = float(start[0]), float(start[1])
        gx, gy = float(goal[0]), float(goal[1])
        if math.hypot(P[0, 0] - sx, P[0, 1] - sy) > endpoint_tol:
            return False
        if math.hypot(P[-1, 0] - gx, P[-1, 1] - gy) > endpoint_tol:
            return False
        if len(P) == 1:
            return bool(obs.segments_valid(P, P)[0])
        return bool(np.all(obs.segments_valid(P[:-1], P[1:])))
    except (TypeError, ValueError, IndexError, OverflowError):
        return False
