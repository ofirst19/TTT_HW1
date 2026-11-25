# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Planner P8: shortest paths for the robot among the obstacles.

It computes the C-obstacles for the robot, builds a visibility graph on their vertices plus
the start and the goal, and returns a shortest path in that graph (Dijkstra's algorithm).
Implementation: scipy (Qhull) convex hull; Shapely STRtree segment test; networkx.
"""
import math

import networkx as nx
import numpy as np
import shapely
from scipy.spatial import ConvexHull
from shapely import STRtree
from shapely.geometry import Polygon

EPS = 1e-9


def qhull(points):
    """Counter-clockwise corner list of the convex hull of a point array."""
    pts = np.asarray(points, dtype=float)
    return pts[ConvexHull(pts).vertices]


def cspace_obstacle(obstacle, robot):
    O = np.asarray(obstacle, dtype=float)
    R = np.asarray(robot, dtype=float)
    return qhull((O[:, None, :] - R[None, :, :]).reshape(-1, 2))


def _separated(A, B):
    """True if an edge line of A or of B leaves the other polygon on its outer side (within EPS)."""
    for P, Q in ((A, B), (B, A)):
        E = np.roll(P, -1, axis=0) - P
        N = np.column_stack((-E[:, 1], E[:, 0])) / np.hypot(E[:, 0], E[:, 1])[:, None]
        depth = Q @ N.T - np.einsum("ij,ij->i", N, P)
        if (depth.max(axis=0) <= EPS).any():
            return True
    return False


def solid_blocks(blocks):
    root = list(range(len(blocks)))

    def find(i):
        while root[i] != i:
            root[i] = root[root[i]]
            i = root[i]
        return i

    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            if not _separated(blocks[i], blocks[j]):
                root[find(i)] = find(j)
    members = {}
    for i in range(len(blocks)):
        members.setdefault(find(i), []).append(blocks[i])
    return [c[0] if len(c) == 1 else qhull(np.vstack(c)) for c in members.values()]


def plan(robot, obstacles, start, goal):
    cobs = [cspace_obstacle(o, robot) for o in obstacles]
    blocks = solid_blocks(cobs)
    cores = [Polygon(b).buffer(-EPS) for b in blocks]
    tree = STRtree(cores)

    ends = np.array([start, goal], dtype=float)
    hits, _ = tree.query(shapely.points(ends), predicate="intersects")
    if len(hits):
        return None, math.inf
    pts = ends
    if blocks:
        corners = np.unique(np.vstack(blocks), axis=0)
        hits, _ = tree.query(shapely.points(corners), predicate="intersects")
        pts = np.vstack([ends, np.delete(corners, np.unique(hits), axis=0)])

    i, j = np.triu_indices(len(pts), k=1)
    lines = shapely.linestrings(np.stack([pts[i], pts[j]], axis=1))
    bad = np.zeros(len(lines), dtype=bool)
    hit_lines, _ = tree.query(lines, predicate="intersects")
    bad[hit_lines] = True
    lengths = np.hypot(*(pts[j] - pts[i]).T)

    G = nx.Graph()
    G.add_nodes_from(range(len(pts)))
    G.add_weighted_edges_from(zip(i[~bad].tolist(), j[~bad].tolist(), lengths[~bad].tolist()))
    try:
        cost, route = nx.bidirectional_dijkstra(G, 0, 1)
    except nx.NetworkXNoPath:
        return None, math.inf
    return [(float(pts[k][0]), float(pts[k][1])) for k in route], float(cost)
