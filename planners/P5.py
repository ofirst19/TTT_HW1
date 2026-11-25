# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P5: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: edge vectors sorted by angle; pure-Python segment test; an O(n^2) array scan.
"""
import math

SLACK = 1e-9
TWO_PI = 2.0 * math.pi


def ccw(poly):
    pts = [(float(x), float(y)) for x, y in poly]
    area = sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]))
    return pts if area > 0 else pts[::-1]


def bottom(poly):
    return min(poly, key=lambda p: (p[1], p[0]))


def trace_cobstacle(obstacle, robot):
    A = ccw(obstacle)
    B = ccw([(-x, -y) for x, y in robot])
    steps = []
    for poly in (A, B):
        for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
            steps.append((x2 - x1, y2 - y1))
    steps.sort(key=lambda v: math.atan2(v[1], v[0]) % TWO_PI)
    (ax, ay), (bx, by) = bottom(A), bottom(B)
    walk = [(ax + bx, ay + by)]
    for dx, dy in steps[:-1]:
        x, y = walk[-1]
        walk.append((x + dx, y + dy))
    corners = []
    for k, p in enumerate(walk):
        prev, nxt = walk[k - 1], walk[(k + 1) % len(walk)]
        bend = (p[0] - prev[0]) * (nxt[1] - p[1]) - (p[1] - prev[1]) * (nxt[0] - p[0])
        if abs(bend) > 1e-12:
            corners.append(p)
    return corners


class Body:
    def __init__(self, corners):
        self.corners = corners
        self.axes = []
        for (x1, y1), (x2, y2) in zip(corners, corners[1:] + corners[:1]):
            h = math.hypot(x2 - x1, y2 - y1)
            nx, ny = (y1 - y2) / h, (x2 - x1) / h
            self.axes.append((nx, ny, nx * x1 + ny * y1))
        self.lo = (min(p[0] for p in corners), min(p[1] for p in corners))
        self.hi = (max(p[0] for p in corners), max(p[1] for p in corners))

    def holds(self, p):
        """Is p deeper than SLACK inside the body?"""
        return all(nx * p[0] + ny * p[1] - c > SLACK for nx, ny, c in self.axes)

    def apart(self, p, q):
        """Is there a separating axis between segment pq and the interior of the body?"""
        if max(p[0], q[0]) <= self.lo[0] or min(p[0], q[0]) >= self.hi[0]:
            return True
        if max(p[1], q[1]) <= self.lo[1] or min(p[1], q[1]) >= self.hi[1]:
            return True
        for nx, ny, c in self.axes:
            if nx * p[0] + ny * p[1] - c <= SLACK and nx * q[0] + ny * q[1] - c <= SLACK:
                return True
        dx, dy = q[0] - p[0], q[1] - p[1]
        span = math.hypot(dx, dy)
        if span == 0.0:
            return not self.holds(p)
        sides = [(dx * (y - p[1]) - dy * (x - p[0])) / span for x, y in self.corners]
        return max(sides) <= SLACK or min(sides) >= -SLACK


def plan(robot, obstacles, start, goal):
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    bodies = [Body(trace_cobstacle(o, robot)) for o in obstacles]
    if any(b.holds(start) or b.holds(goal) for b in bodies):
        return None, math.inf

    corners = []
    for b in bodies:
        for v in b.corners:
            if v not in corners and not any(o.holds(v) for o in bodies):
                corners.append(v)
    nodes = [start, goal] + corners
    n = len(nodes)

    def sees(i, j):
        return all(b.apart(nodes[i], nodes[j]) for b in bodies)

    weight = [[math.inf] * n for _ in range(n)]
    for i in range(2, n):
        for j in range(i + 1, n):
            if sees(i, j):
                weight[i][j] = weight[j][i] = math.dist(nodes[i], nodes[j])
    for e in (0, 1):
        for j in range(2, n):
            if sees(e, j):
                weight[e][j] = weight[j][e] = math.dist(nodes[e], nodes[j])

    dist = [math.inf] * n
    prev = [None] * n
    settled = [False] * n
    dist[0] = 0.0
    for _ in range(n):
        u, du = -1, math.inf
        for k in range(n):
            if not settled[k] and dist[k] < du:
                u, du = k, dist[k]
        if u < 0 or u == 1:
            break
        settled[u] = True
        for v in range(n):
            if du + weight[u][v] < dist[v]:
                dist[v] = du + weight[u][v]
                prev[v] = u
    if dist[1] == math.inf:
        return None, math.inf
    order = [1]
    while order[-1] != 0:
        order.append(prev[order[-1]])
    return [nodes[k] for k in reversed(order)], dist[1]
