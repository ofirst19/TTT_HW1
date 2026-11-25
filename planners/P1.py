# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P1: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: angular merge of edge vectors; numpy segment test; networkx.
"""
import math

import networkx as nx
import numpy as np

EPS = 1e-9


def _lowest_first(poly):
    """The polygon counter-clockwise, rotated to start at its lowest (then leftmost) vertex."""
    pts = [(float(x), float(y)) for x, y in poly]
    area2 = sum(pts[i - 1][0] * pts[i][1] - pts[i][0] * pts[i - 1][1] for i in range(len(pts)))
    if area2 < 0:
        pts.reverse()
    k = min(range(len(pts)), key=lambda i: (pts[i][1], pts[i][0]))
    return pts[k:] + pts[:k]


def edge_merge(P, Q):
    """Minkowski sum of two convex polygons: walk both edge lists in order of angle."""
    P, Q = _lowest_first(P), _lowest_first(Q)
    n, m = len(P), len(Q)
    P, Q = P + P[:2], Q + Q[:2]
    out = []
    i = j = 0
    while i < n or j < m:
        out.append((P[i][0] + Q[j][0], P[i][1] + Q[j][1]))
        ex, ey = P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1]
        fx, fy = Q[j + 1][0] - Q[j][0], Q[j + 1][1] - Q[j][1]
        turn = ex * fy - ey * fx
        if turn >= 0 and i < n:
            i += 1
        if turn <= 0 and j < m:
            j += 1
    return out


def _half_planes(poly):
    """Unit inward normals (k x 2), offsets (k,), and bounding box of a CCW convex polygon."""
    V = np.asarray(poly, dtype=float)
    E = np.roll(V, -1, axis=0) - V
    L = np.hypot(E[:, 0], E[:, 1])
    E, V, L = E[L > 0], V[L > 0], L[L > 0]
    N = np.column_stack((-E[:, 1], E[:, 0])) / L[:, None]
    return N, np.einsum("ij,ij->i", N, V), (V[:, 0].min(), V[:, 0].max(), V[:, 1].min(), V[:, 1].max())


def _covered(X, planes):
    """Mask of the points X (n x 2) lying deeper than EPS inside some C-obstacle."""
    out = np.zeros(len(X), dtype=bool)
    for N, c, _ in planes:
        out |= (X @ N.T - c).min(axis=1) > EPS
    return out


def _blocked(A, B, planes):
    """Mask over the segments A[k]B[k]: True where some point lies deeper than EPS in a C-obstacle."""
    hit = np.zeros(len(A), dtype=bool)
    x_lo, x_hi = np.minimum(A[:, 0], B[:, 0]), np.maximum(A[:, 0], B[:, 0])
    y_lo, y_hi = np.minimum(A[:, 1], B[:, 1]), np.maximum(A[:, 1], B[:, 1])
    for N, c, (bx0, bx1, by0, by1) in planes:
        k = np.flatnonzero(~hit & (x_hi > bx0) & (x_lo < bx1) & (y_hi > by0) & (y_lo < by1))
        if k.size == 0:
            continue
        da = A[k] @ N.T - c
        db = B[k] @ N.T - c
        slope = db - da
        cut = np.divide(EPS - da, slope, out=np.zeros_like(da), where=slope != 0)
        enter = np.where(slope > 0, cut, -np.inf).max(axis=1)
        leave = np.where(slope < 0, cut, np.inf).min(axis=1)
        never = ((slope == 0) & (da <= EPS)).any(axis=1)
        hit[k[~never & (enter < leave) & (enter < 1.0) & (leave > 0.0)]] = True
    return hit


def plan(robot, obstacles, start, goal):
    s = (float(start[0]), float(start[1]))
    g = (float(goal[0]), float(goal[1]))
    regions = [edge_merge(obs, robot) for obs in obstacles]
    planes = [_half_planes(r) for r in regions]
    if _covered(np.array([s, g]), planes).any():
        return None, math.inf

    nodes, seen = [s, g], {s, g}
    corners = [p for r in regions for p in r if p not in seen]
    if corners:
        C = np.array(corners, dtype=float)
        for p, inside in zip(corners, _covered(C, planes)):
            if not inside and p not in seen:
                seen.add(p)
                nodes.append(p)

    V = np.array(nodes, dtype=float)
    I, J = np.triu_indices(len(nodes), k=1)
    ok = ~_blocked(V[I], V[J], planes)
    w = np.hypot(V[J, 0] - V[I, 0], V[J, 1] - V[I, 1])

    G = nx.Graph()
    G.add_nodes_from(range(len(nodes)))
    G.add_weighted_edges_from(zip(I[ok].tolist(), J[ok].tolist(), w[ok].tolist()))
    try:
        route = nx.dijkstra_path(G, 0, 1, weight="weight")
    except nx.NetworkXNoPath:
        return None, math.inf
    path = [nodes[k] for k in route]
    cost = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    return path, cost
