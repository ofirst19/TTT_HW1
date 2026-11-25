# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P3: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: Shapely convex hull; pure-Python segment test; a hand-written binary heap.
"""
import math

from shapely.geometry import MultiPoint
from shapely.geometry.polygon import orient

GAP = 1e-9


def _region(obstacle, robot):
    cloud = MultiPoint([(ox - rx, oy - ry) for ox, oy in obstacle for rx, ry in robot])
    hull = orient(cloud.convex_hull, sign=1.0)
    return [(float(x), float(y)) for x, y in hull.exterior.coords[:-1]]


def _rows(corners):
    rows = []
    for (x1, y1), (x2, y2) in zip(corners, corners[1:] + corners[:1]):
        ux, uy = x2 - x1, y2 - y1
        n = math.hypot(ux, uy)
        rows.append((-uy / n, ux / n, (x1 * uy - y1 * ux) / n))
    return rows


def _depth(rows, p):
    return min(a * p[0] + b * p[1] + c for a, b, c in rows)


def _meets(rows, p, q, level):
    """Does segment pq contain a point whose depth in the C-obstacle reaches the given level?"""
    lo, hi = 0.0, 1.0
    for a, b, c in rows:
        f0 = a * p[0] + b * p[1] + c - level
        f1 = a * q[0] + b * q[1] + c - level
        if f1 > f0:
            lo = max(lo, -f0 / (f1 - f0))
        elif f1 < f0:
            hi = min(hi, -f0 / (f1 - f0))
        elif f0 < 0:
            return False
        if lo > hi:
            return False
    return True


def _leaves(rows, p, q):
    """Does the segment from p towards q move to the outer side of an edge line through p?"""
    dx, dy = q[0] - p[0], q[1] - p[1]
    tiny = 1e-12 * math.hypot(dx, dy)
    return any(abs(a * p[0] + b * p[1] + c) <= GAP and a * dx + b * dy < -tiny for a, b, c in rows)


def _clear(rows, p, q):
    if not _meets(rows, p, q, -GAP):
        return True
    return _leaves(rows, p, q) or _leaves(rows, q, p)


class _Heap:
    """Binary min-heap of (key, item) pairs."""

    def __init__(self):
        self.a = []

    def __len__(self):
        return len(self.a)

    def push(self, key, item):
        a = self.a
        a.append((key, item))
        i = len(a) - 1
        while i > 0:
            up = (i - 1) // 2
            if a[up][0] <= a[i][0]:
                break
            a[up], a[i] = a[i], a[up]
            i = up

    def pop(self):
        a = self.a
        top = a[0]
        last = a.pop()
        if a:
            a[0] = last
            i, n = 0, len(a)
            while True:
                kid = 2 * i + 1
                if kid >= n:
                    break
                if kid + 1 < n and a[kid + 1][0] < a[kid][0]:
                    kid += 1
                if a[i][0] <= a[kid][0]:
                    break
                a[i], a[kid] = a[kid], a[i]
                i = kid
        return top


def plan(robot, obstacles, start, goal):
    src = (float(start[0]), float(start[1]))
    dst = (float(goal[0]), float(goal[1]))
    robot = [(float(x), float(y)) for x, y in robot]
    shapes = []
    for obs in obstacles:
        corners = _region([(float(x), float(y)) for x, y in obs], robot)
        xs = [x for x, _ in corners]
        ys = [y for _, y in corners]
        shapes.append((corners, _rows(corners), (min(xs), max(xs), min(ys), max(ys))))

    def inside(p):
        return any(_depth(rows, p) > GAP for _, rows, _ in shapes)

    if inside(src) or inside(dst):
        return None, math.inf
    pts = [src, dst]
    for corners, _, _ in shapes:
        for v in corners:
            if v not in pts and not inside(v):
                pts.append(v)

    def visible(p, q):
        for _, rows, (x0, x1, y0, y1) in shapes:
            if max(p[0], q[0]) < x0 - GAP or min(p[0], q[0]) > x1 + GAP:
                continue
            if max(p[1], q[1]) < y0 - GAP or min(p[1], q[1]) > y1 + GAP:
                continue
            if not _clear(rows, p, q):
                return False
        return True

    n = len(pts)
    nbrs = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if visible(pts[i], pts[j]):
                d = math.dist(pts[i], pts[j])
                nbrs[i].append((j, d))
                nbrs[j].append((i, d))

    best = [math.inf] * n
    back = [-1] * n
    best[0] = 0.0
    heap = _Heap()
    heap.push(0.0, 0)
    while len(heap):
        d, u = heap.pop()
        if d > best[u]:
            continue
        if u == 1:
            break
        for v, w in nbrs[u]:
            if d + w < best[v]:
                best[v] = d + w
                back[v] = u
                heap.push(best[v], v)
    if best[1] == math.inf:
        return None, math.inf
    route = [1]
    while route[-1] != 0:
        route.append(back[route[-1]])
    path = [pts[k] for k in reversed(route)]
    return path, best[1]
