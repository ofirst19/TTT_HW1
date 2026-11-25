# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Referee for HW1 (236767, Winter 2026-27): the reference answer for a triangle robot.

The robot R is a convex polygon given by its vertices relative to its reference point.
The C-obstacle of an obstacle O is  CO(O) = O (+) (-R) = {o - r : o in O, r in R}.
A path is valid if no point of it lies in the interior of any C-obstacle, tested per
C-obstacle with the EPS = 1e-9 rule of geom_ref (touching, sliding, and passing through
vertices are allowed). Cost is Euclidean length. The visibility graph and Dijkstra are
those of planner_ref, which work on C-obstacles and do not depend on the robot.
"""
from __future__ import annotations

try:
    from . import geom_ref, planner_ref
except ImportError:  # imported as a top-level module
    import geom_ref  # type: ignore
    import planner_ref  # type: ignore

EPS = geom_ref.EPS


def _pt(p):
    return (float(p[0]), float(p[1]))


def c_obstacle(obstacle, robot):
    """O (+) (-R): corners only, counter-clockwise, starting from the lowest vertex (leftmost on ties)."""
    sums = [(float(ox) - float(rx), float(oy) - float(ry)) for ox, oy in obstacle for rx, ry in robot]
    return geom_ref.canonical_corners(planner_ref._convex_hull(sums))


def c_obstacles(obstacles, robot):
    return [c_obstacle(o, robot) for o in obstacles]


def solve(robot, obstacles, start, goal) -> dict:
    """Reference answer: {"c_obstacles", "path", "cost", "path_exists"}; path None and cost inf if no path."""
    s, g = _pt(start), _pt(goal)
    cobs = c_obstacles(obstacles, robot)
    edges = planner_ref.get_visibility_graph(cobs, s, g)
    path, cost = planner_ref.shortest_path(edges, s, g)
    return {"c_obstacles": cobs, "path": path, "cost": cost, "path_exists": path is not None}


def path_valid(robot, obstacles, path, start, goal) -> bool:
    """True iff path is a polygonal chain from start to goal with no point in the interior of a C-obstacle."""
    return bool(geom_ref.path_valid(path, _pt(start), _pt(goal), c_obstacles(obstacles, robot)))
