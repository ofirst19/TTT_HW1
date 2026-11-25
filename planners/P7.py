# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P7: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: edge vectors sorted by a comparator; pure-Python segment test; heapq.
"""
import functools
import heapq
import math

TOL = 1e-9


def _upper(v):
    return v[1] > 0 or (v[1] == 0 and v[0] > 0)


def _by_angle(u, v):
    if _upper(u) != _upper(v):
        return -1 if _upper(u) else 1
    turn = u[0] * v[1] - u[1] * v[0]
    return -1 if turn > 0 else (1 if turn < 0 else 0)


def _counter_clockwise(poly):
    twice_area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1]))
    return poly if twice_area > 0 else poly[::-1]


def c_obstacle(obstacle, robot):
    O = _counter_clockwise([(float(x), float(y)) for x, y in obstacle])
    M = _counter_clockwise([(-float(x), -float(y)) for x, y in robot])
    vectors = []
    for poly in (O, M):
        vectors += [(b[0] - a[0], b[1] - a[1]) for a, b in zip(poly, poly[1:] + poly[:1])]
    vectors.sort(key=functools.cmp_to_key(_by_angle))
    o = min(O, key=lambda p: (p[1], p[0]))
    m = min(M, key=lambda p: (p[1], p[0]))
    chain = [(o[0] + m[0], o[1] + m[1])]
    for dx, dy in vectors[:-1]:
        chain.append((chain[-1][0] + dx, chain[-1][1] + dy))
    out = []
    for k in range(len(chain)):
        a, b, c = chain[k - 1], chain[k], chain[(k + 1) % len(chain)]
        if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) > 1e-12:
            out.append(b)
    return out


def ring(poly):
    """Edges (a, b, ex, ey, length) of a counter-clockwise polygon."""
    out = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        ex, ey = b[0] - a[0], b[1] - a[1]
        out.append((a, b, ex, ey, math.hypot(ex, ey)))
    return out


def strictly_inside(x, edges):
    """x lies to the left of every edge, farther than TOL."""
    return all((ex * (x[1] - a[1]) - ey * (x[0] - a[0])) / ab > TOL for a, _, ex, ey, ab in edges)


def blocked(p, q, edges):
    dx, dy = q[0] - p[0], q[1] - p[1]
    span = math.hypot(dx, dy)
    if span == 0.0:
        return strictly_inside(p, edges)
    for a, b, ex, ey, ab in edges:
        s1 = (dx * (a[1] - p[1]) - dy * (a[0] - p[0])) / span
        s2 = (dx * (b[1] - p[1]) - dy * (b[0] - p[0])) / span
        if (s1 > TOL and s2 < -TOL) or (s1 < -TOL and s2 > TOL):
            s3 = (ex * (p[1] - a[1]) - ey * (p[0] - a[0])) / ab
            s4 = (ex * (q[1] - a[1]) - ey * (q[0] - a[0])) / ab
            if (s3 > TOL and s4 < -TOL) or (s3 < -TOL and s4 > TOL):
                return True
    return False


def plan(robot, obstacles, start, goal):
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    polys = [c_obstacle(o, robot) for o in obstacles]
    rings = [ring(P) for P in polys]
    boxes = [(min(x for x, _ in P), max(x for x, _ in P), min(y for _, y in P), max(y for _, y in P))
             for P in polys]

    def free(x):
        return not any(strictly_inside(x, E) for E in rings)

    if not (free(start) and free(goal)):
        return None, math.inf
    vertices = [start, goal]
    for P in polys:
        vertices += [v for v in P if v not in vertices and free(v)]

    def visible(p, q):
        for E, (x0, x1, y0, y1) in zip(rings, boxes):
            if max(p[0], q[0]) <= x0 or min(p[0], q[0]) >= x1:
                continue
            if max(p[1], q[1]) <= y0 or min(p[1], q[1]) >= y1:
                continue
            if blocked(p, q, E):
                return False
        return True

    n = len(vertices)
    graph = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if visible(vertices[i], vertices[j]):
                d = math.dist(vertices[i], vertices[j])
                graph[i].append((d, j))
                graph[j].append((d, i))

    best = [math.inf] * n
    came = [None] * n
    best[0] = 0.0
    frontier = [(0.0, 0)]
    closed = [False] * n
    while frontier:
        d, u = heapq.heappop(frontier)
        if closed[u]:
            continue
        closed[u] = True
        if u == 1:
            break
        for w, v in graph[u]:
            if d + w < best[v]:
                best[v] = d + w
                came[v] = u
                heapq.heappush(frontier, (best[v], v))
    if not closed[1]:
        return None, math.inf
    route = [1]
    while route[-1] != 0:
        route.append(came[route[-1]])
    return [vertices[k] for k in reversed(route)], best[1]
