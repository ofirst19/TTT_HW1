# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Visibility graph and shortest path of the referee (236767 HW1).

Both work on C-obstacles and do not depend on the robot.  Geometry and the
tolerance conventions live in geom_ref.py (EPS = 1e-9).
"""

from __future__ import annotations

import heapq
import math

try:
    from . import geom_ref
except ImportError:  # imported as a top-level module
    import geom_ref  # type: ignore

EPS = geom_ref.EPS

Point = tuple[float, float]
Edge = tuple[Point, Point]

def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _convex_hull(points: list[Point]) -> list[Point]:
    """Andrew's monotone chain; CCW, collinear points dropped (exact test;
    near-collinear ones are removed afterwards by canonical_corners)."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts
    lower: list[Point] = []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def get_visibility_graph(c_obstacles: list[list[tuple[float, float]]],
                         source: tuple[float, float] | None = None,
                         dest: tuple[float, float] | None = None
                         ) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Edges of the visibility graph, each as a pair of points (unordered, no duplicates)."""
    edges, _ = geom_ref.visibility_edges(c_obstacles, source, dest, EPS)
    return edges


def _find_node(index: dict, nodes: list[Point], p: Point):
    k = index.get(p)
    if k is not None:
        return k
    best, bestd = None, EPS
    for i, q in enumerate(nodes):
        dd = math.hypot(q[0] - p[0], q[1] - p[1])
        if dd <= bestd:
            best, bestd = i, dd
    return best


def shortest_path(edges: list[tuple[tuple[float, float], tuple[float, float]]],
                  source: tuple[float, float],
                  dest: tuple[float, float]
                  ) -> tuple[list[tuple[float, float]] | None, float]:
    """Dijkstra over the undirected edges, weighted by Euclidean length.
    Returns (path from source to dest as a list of points, cost) or (None, math.inf)."""
    source = (float(source[0]), float(source[1]))
    dest = (float(dest[0]), float(dest[1]))
    index: dict[Point, int] = {}
    nodes: list[Point] = []
    adj: list[dict[int, float]] = []

    def node(p) -> int:
        p = (float(p[0]), float(p[1]))
        k = index.get(p)
        if k is None:
            k = len(nodes)
            index[p] = k
            nodes.append(p)
            adj.append({})
        return k

    for a, b in edges:
        i, j = node(a), node(b)
        if i == j:
            continue
        w = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
        if w < adj[i].get(j, math.inf):
            adj[i][j] = w
            adj[j][i] = w

    s = _find_node(index, nodes, source)
    t = _find_node(index, nodes, dest)
    if s is None or t is None:
        return None, math.inf
    if s == t or math.hypot(source[0] - dest[0], source[1] - dest[1]) <= EPS:
        return [source], 0.0

    dist = [math.inf] * len(nodes)
    prev = [-1] * len(nodes)
    dist[s] = 0.0
    heap = [(0.0, s)]
    while heap:
        du, u = heapq.heappop(heap)
        if du > dist[u]:
            continue
        if u == t:
            break
        for v, w in adj[u].items():
            nd = du + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    if dist[t] == math.inf:
        return None, math.inf
    seq = [t]
    while seq[-1] != s:
        seq.append(prev[seq[-1]])
    seq.reverse()
    path = [nodes[k] for k in seq]
    path[0], path[-1] = source, dest
    return path, float(dist[t])
