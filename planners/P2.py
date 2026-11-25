# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P2: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: Andrew's monotone chain; pure-Python segment test; networkx.
"""
import itertools
import math

import networkx as nx

TOL = 1e-9


def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def monotone_chain(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def c_obstacle(obstacle, robot):
    diffs = [(float(ox) - float(rx), float(oy) - float(ry)) for ox, oy in obstacle for rx, ry in robot]
    return monotone_chain(diffs)


class Region:
    """A convex C-obstacle stored as half-planes a*x + b*y >= c (unit inward normals)."""

    def __init__(self, corners):
        self.corners = corners
        self.lines = []
        k = len(corners)
        for i in range(k):
            (x1, y1), (x2, y2) = corners[i], corners[(i + 1) % k]
            length = math.hypot(x2 - x1, y2 - y1)
            a, b = -(y2 - y1) / length, (x2 - x1) / length
            self.lines.append((a, b, a * x1 + b * y1))
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        self.box = (min(xs), max(xs), min(ys), max(ys))

    def depth(self, p):
        return min(a * p[0] + b * p[1] - c for a, b, c in self.lines)

    def cuts(self, p, q):
        """True if some point of segment pq lies deeper than TOL inside the region."""
        x0, x1, y0, y1 = self.box
        if max(p[0], q[0]) <= x0 or min(p[0], q[0]) >= x1 or max(p[1], q[1]) <= y0 or min(p[1], q[1]) >= y1:
            return False
        t_lo, t_hi = 0.0, 1.0
        for a, b, c in self.lines:
            f = a * p[0] + b * p[1] - c - TOL          # need f + t * df > 0
            df = a * (q[0] - p[0]) + b * (q[1] - p[1])
            if df > 0:
                t_lo = max(t_lo, -f / df)
            elif df < 0:
                t_hi = min(t_hi, -f / df)
            elif f <= 0:
                return False
            if t_lo >= t_hi:
                return False
        return True


def plan(robot, obstacles, start, goal):
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    regions = [Region(c_obstacle(obs, robot)) for obs in obstacles]

    def free_point(p):
        return all(r.depth(p) <= TOL for r in regions)

    if not (free_point(start) and free_point(goal)):
        return None, math.inf

    graph = nx.Graph()
    graph.add_node(start)
    graph.add_node(goal)
    for r in regions:
        for v in r.corners:
            if free_point(v):
                graph.add_node(v)

    for p, q in itertools.combinations(list(graph.nodes), 2):
        if not any(r.cuts(p, q) for r in regions):
            graph.add_edge(p, q, length=math.hypot(q[0] - p[0], q[1] - p[1]))

    try:
        cost, route = nx.single_source_dijkstra(graph, start, goal, weight="length")
    except nx.NetworkXNoPath:
        return None, math.inf
    return list(route), float(cost)
