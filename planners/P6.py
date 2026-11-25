# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P6: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: Shapely convex hull; numpy segment test; an indexed heap with decrease-key.
"""
import math

import numpy as np
from shapely.affinity import translate
from shapely.geometry import GeometryCollection, Polygon
from shapely.geometry.polygon import orient

EPS = 1e-9


def configuration_polygon(obstacle, robot):
    body = Polygon(obstacle)
    copies = [translate(body, xoff=-rx, yoff=-ry) for rx, ry in robot]
    hull = orient(GeometryCollection(copies).convex_hull, sign=1.0)
    return np.asarray(hull.exterior.coords, dtype=float)[:-1]


class HalfPlanes:
    """All edges of all C-obstacles: rows n . x - c >= 0 inside, grouped by polygon."""

    def __init__(self, polys):
        normals, offsets, starts = [], [], []
        for P in polys:
            starts.append(sum(len(x) for x in normals))
            E = np.roll(P, -1, axis=0) - P
            N = np.column_stack((-E[:, 1], E[:, 0])) / np.hypot(E[:, 0], E[:, 1])[:, None]
            normals.append(N)
            offsets.append(np.einsum("ij,ij->i", N, P))
        self.count = len(polys)
        self.N = np.vstack(normals) if normals else np.zeros((0, 2))
        self.c = np.concatenate(offsets) if offsets else np.zeros(0)
        self.starts = np.asarray(starts, dtype=int)

    def deepest(self, X):
        """Depth of each point of X in each polygon (rows: points, columns: polygons)."""
        S = X @ self.N.T - self.c
        return np.minimum.reduceat(S, self.starts, axis=1)

    def clear_from(self, p, Q):
        """For each q in Q: no point of segment pq lies deeper than EPS in a polygon."""
        if self.count == 0 or len(Q) == 0:
            return np.ones(len(Q), dtype=bool)
        s0 = self.N @ p - self.c
        s1 = Q @ self.N.T - self.c
        ds = s1 - s0
        t = np.divide(EPS - s0, ds, out=np.zeros_like(ds), where=ds != 0)
        lower = np.maximum.reduceat(np.where(ds > 0, t, -np.inf), self.starts, axis=1)
        upper = np.minimum.reduceat(np.where(ds < 0, t, np.inf), self.starts, axis=1)
        stuck = np.logical_or.reduceat((ds == 0) & (s0 <= EPS), self.starts, axis=1)
        enters = ~stuck & (lower < upper) & (lower < 1.0) & (upper > 0.0)
        return ~enters.any(axis=1)


class IndexedHeap:
    """Min-heap over items 0..n-1 with decrease-key."""

    def __init__(self, n):
        self.key = [math.inf] * n
        self.where = [-1] * n
        self.items = []

    def _swap(self, i, j):
        it = self.items
        it[i], it[j] = it[j], it[i]
        self.where[it[i]] = i
        self.where[it[j]] = j

    def _up(self, i):
        while i > 0 and self.key[self.items[i]] < self.key[self.items[(i - 1) // 2]]:
            self._swap(i, (i - 1) // 2)
            i = (i - 1) // 2

    def _down(self, i):
        n = len(self.items)
        while True:
            small = i
            for kid in (2 * i + 1, 2 * i + 2):
                if kid < n and self.key[self.items[kid]] < self.key[self.items[small]]:
                    small = kid
            if small == i:
                return
            self._swap(i, small)
            i = small

    def offer(self, item, key):
        """Insert item, or lower its key; returns True if the key changed."""
        if key >= self.key[item]:
            return False
        self.key[item] = key
        if self.where[item] < 0:
            self.items.append(item)
            self.where[item] = len(self.items) - 1
        self._up(self.where[item])
        return True

    def pop(self):
        top = self.items[0]
        self._swap(0, len(self.items) - 1)
        self.items.pop()
        self.where[top] = -2
        if self.items:
            self._down(0)
        return top, self.key[top]

    def __bool__(self):
        return bool(self.items)


def plan(robot, obstacles, start, goal):
    robot = [(float(x), float(y)) for x, y in robot]
    polys = [configuration_polygon([(float(x), float(y)) for x, y in o], robot) for o in obstacles]
    planes = HalfPlanes(polys)
    ends = np.array([start, goal], dtype=float)
    if planes.count and (planes.deepest(ends) > EPS).any():
        return None, math.inf

    pts = ends
    if planes.count:
        corners = np.unique(np.vstack(polys), axis=0)
        corners = corners[~(planes.deepest(corners) > EPS).any(axis=1)]
        pts = np.vstack([ends, corners])
    n = len(pts)
    adjacency = [[] for _ in range(n)]
    for i in range(n - 1):
        ok = planes.clear_from(pts[i], pts[i + 1:])
        for j in np.flatnonzero(ok) + i + 1:
            d = float(np.hypot(*(pts[j] - pts[i])))
            adjacency[i].append((int(j), d))
            adjacency[int(j)].append((i, d))

    heap = IndexedHeap(n)
    via = [-1] * n
    heap.offer(0, 0.0)
    final = None
    while heap:
        u, du = heap.pop()
        if u == 1:
            final = du
            break
        for v, w in adjacency[u]:
            if heap.where[v] != -2 and heap.offer(v, du + w):
                via[v] = u
    if final is None:
        return None, math.inf
    chain = [1]
    while chain[-1] != 0:
        chain.append(via[chain[-1]])
    path = [(float(pts[k][0]), float(pts[k][1])) for k in reversed(chain)]
    return path, final
