# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""The referee of HW1: the correct answer for your pair's robot.

    from referee import referee
    answer = referee.solve(robot, obstacles, start, goal)   # c_obstacles, path, cost, path_exists
    referee.path_valid(robot, obstacles, path, start, goal)  # the validity test of check_ce.py
    referee.c_obstacle(obstacle, robot)                       # O (+) (-R), counter-clockwise

check_ce.py and draw.py use it. geom_ref holds the geometry (EPS = 1e-9) and
planner_ref the visibility graph and Dijkstra.
"""
from . import geom_ref, planner_ref, referee  # noqa: F401
from .referee import c_obstacle, c_obstacles, path_valid, solve  # noqa: F401

EPS = geom_ref.EPS
