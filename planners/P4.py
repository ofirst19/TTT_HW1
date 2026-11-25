# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P4: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: gift wrapping (Jarvis march); Shapely segment test; heapq.
"""
import heapq
import math

import numpy as np
import shapely
from shapely.geometry import Polygon

TOLERANCE = 1e-9
GRID = 1000                 # instance coordinates are multiples of 1 / GRID


def _orient(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _reach(o, a):
    return (a[0] - o[0]) ** 2 + (a[1] - o[1]) ** 2


def gift_wrap(points):
    """Convex hull of integer points, counter-clockwise, by the Jarvis march from the leftmost point."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts
    first = pts[0]
    hull = [first]
    current = first
    for _ in range(len(pts)):
        pick = pts[1] if current == pts[0] else pts[0]
        for r in pts:
            if r == current or r == pick:
                continue
            turn = _orient(current, pick, r)
            if turn < 0 or (turn == 0 and _reach(current, r) > _reach(current, pick)):
                pick = r
        if pick == first:
            break
        hull.append(pick)
        current = pick
    return hull


def _units(p):
    return round(GRID * float(p[0])), round(GRID * float(p[1]))


def configuration_obstacle(obstacle, robot):
    """Corners of the C-obstacle, counter-clockwise; the hull is exact in grid units."""
    cells = [_units(r) for r in robot]
    hull = gift_wrap([(ox - rx, oy - ry) for ox, oy in map(_units, obstacle) for rx, ry in cells])
    return [(x / GRID, y / GRID) for x, y in hull]


def _barriers(regions):
    shapes = shapely.make_valid([Polygon(r) for r in regions])
    pieces = shapely.get_parts(shapely.union_all(shapes, grid_size=TOLERANCE))
    return [piece.buffer(-TOLERANCE) for piece in pieces]


def plan(robot, obstacles, start, goal):
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    regions = [configuration_obstacle(o, robot) for o in obstacles]
    barriers = [b for b in _barriers(regions) if not b.is_empty]
    for b in barriers:
        shapely.prepare(b)

    def covered(xy):
        mask = np.zeros(len(xy), dtype=bool)
        if len(xy):
            dots = shapely.points(np.asarray(xy, dtype=float))
            for b in barriers:
                mask |= shapely.intersects(dots, b)
        return mask

    if covered([start, goal]).any():
        return None, math.inf
    corners = list(dict.fromkeys(v for r in regions for v in r if v != start and v != goal))
    keep = covered(corners)
    nodes = [start, goal] + [v for v, bad in zip(corners, keep) if not bad]

    xy = np.asarray(nodes, dtype=float)
    a, b = np.triu_indices(len(nodes), k=1)
    segs = shapely.linestrings(np.stack([xy[a], xy[b]], axis=1))
    lo, hi = np.minimum(xy[a], xy[b]), np.maximum(xy[a], xy[b])
    blocked = np.zeros(len(segs), dtype=bool)
    for bar in barriers:
        x0, y0, x1, y1 = bar.bounds
        near = np.flatnonzero(~blocked & (hi[:, 0] >= x0) & (lo[:, 0] <= x1) & (hi[:, 1] >= y0) & (lo[:, 1] <= y1))
        if near.size:
            blocked[near] = shapely.intersects(segs[near], bar)

    adjacent = {k: [] for k in range(len(nodes))}
    for i, j in zip(a[~blocked].tolist(), b[~blocked].tolist()):
        d = math.hypot(xy[j, 0] - xy[i, 0], xy[j, 1] - xy[i, 1])
        adjacent[i].append((j, d))
        adjacent[j].append((i, d))

    dist = {0: 0.0}
    parent = {0: None}
    done = set()
    queue = [(0.0, 0)]
    while queue:
        d, u = heapq.heappop(queue)
        if u in done:
            continue
        done.add(u)
        if u == 1:
            break
        for v, w in adjacent[u]:
            if v not in done and d + w < dist.get(v, math.inf):
                dist[v] = d + w
                parent[v] = u
                heapq.heappush(queue, (d + w, v))
    if 1 not in done:
        return None, math.inf
    seq = []
    u = 1
    while u is not None:
        seq.append(nodes[u])
        u = parent[u]
    seq.reverse()
    return seq, dist[1]
