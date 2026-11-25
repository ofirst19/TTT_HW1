# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).  Report figures come from this script.
# =============================================================================
"""Figures for HW1.

Usage
-----
  python draw.py --ids ID1 [ID2] counterexamples/P<k>.json [--planner P<k>] [--out figs/P<k>.pdf]
      Draws the instance for your pair's robot: the obstacles, numbered 0, 1, ... in
      the order of the file (the numbers of the messages of check_ce.py), their
      C-obstacles O (+) (-R), each with its own outline (so two C-obstacles that touch
      along an edge show that edge), the robot at the start and at the goal, and the
      referee's path. With --planner P<k> it also runs that planner, as check_ce.py
      does, and draws its path.
  python draw.py --ids ID1 [ID2] --warmup [--out figs/warmup.pdf]
      Draws the Part A warm-up: the obstacle O of gen_hw1.py --warmup, the C-obstacle
      O (+) (-R) with the coordinates of its vertices, and the robot placed with its
      reference point at each of these vertices.

The format follows the extension of --out (.pdf or .png). The default file is
figs/<name of the instance file>.pdf, or figs/warmup.pdf; --format png changes the
default extension. Figures are 3.4 inches wide, for a column of about half the text
width, as in report_template.tex. The tool warns (and still runs) when a DO-NOT-EDIT
file of the kit, a planner included, differs from the original recorded in
MANIFEST.json.

Importable: draw_instance(out, robot, obstacles, start, goal, ...) and
draw_warmup(out, robot, obstacle).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import to_rgba  # noqa: E402
from matplotlib.patches import Polygon as MplPolygon  # noqa: E402

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import check_ce  # noqa: E402
import gen_hw1  # noqa: E402
from referee import referee  # noqa: E402

OBSTACLE_COLOR = "#3b4a6b"
COBSTACLE_COLOR = "#d9534f"
COBSTACLE_FACE = to_rgba(COBSTACLE_COLOR, 0.32)   # see-through, so overlaps show
COBSTACLE_EDGE = "#a8322e"
REFEREE_COLOR = "#f0a500"
PLANNER_COLOR = "#1f5fa8"
START_COLOR = "#2e8b57"
GOAL_COLOR = "#6a3d9a"
ROBOT_COPY_COLOR = "#8a6d00"
FIGSIZE = (3.4, 3.4)
COBSTACLE_LABEL = r"C-obstacle $O \oplus (-R)$"


def placed(robot, p):
    """The robot with its reference point at p."""
    return [(p[0] + x, p[1] + y) for x, y in robot]


def _new_axes(figsize):
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    return fig, ax


def _finish(fig, ax, out, pts, ncol=2):
    """Fit the view to pts, put the legend below the axes, save, and close."""
    xs = [p[0] for p in pts if math.isfinite(p[0]) and math.isfinite(p[1])]
    ys = [p[1] for p in pts if math.isfinite(p[0]) and math.isfinite(p[1])]
    if xs:
        span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
        pad = 0.06 * span
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)
    handles, labels = ax.get_legend_handles_labels()
    fig.tight_layout()
    if handles:   # below the axes and their tick labels, so that it never hides the start or the goal
        fig.canvas.draw()
        box = ax.get_tightbbox(fig.canvas.get_renderer()).transformed(fig.transFigure.inverted())
        pos = ax.get_position()
        fig.legend(handles, labels, loc="upper center", bbox_to_anchor=((pos.x0 + pos.x1) / 2, box.y0),
                   bbox_transform=fig.transFigure, ncol=ncol, fontsize=7, frameon=False,
                   handlelength=1.6, columnspacing=1.0)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out


def _answer_parts(ans):
    """(status, path or None, cost) from a run_planner result, a referee.solve result, or the JSON form."""
    if ans is None:
        return "none", None, math.inf
    status = ans.get("status", "done")
    path = ans.get("path")
    path = [(float(x), float(y)) for x, y in path] if path else None
    cost = ans.get("cost")
    cost = math.inf if cost is None else float(cost)
    return status, path, cost


def _draw_path(ax, path, color, label, lw, ls, marker, ms, z):
    xs, ys = zip(*path)
    ax.plot(xs, ys, ls, color=color, lw=lw, zorder=z, label=label)
    ax.plot(xs, ys, marker, color=color, mec="black", mew=0.4, ms=ms, zorder=z + 0.1)


def draw_instance(out, robot, obstacles, start, goal, c_obstacles=None, referee_answer=None, planner=None,
                  planner_answer=None, figsize=FIGSIZE, title=None):
    """Draw obstacles, C-obstacles, the robot at start and goal, the referee's path, and,
    when planner is given, that planner's answer. Returns the path of the written file."""
    robot = [(float(x), float(y)) for x, y in robot]
    start, goal = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
    if referee_answer is None:
        referee_answer = referee.solve(robot, obstacles, start, goal)
    if c_obstacles is None:
        c_obstacles = referee_answer.get("c_obstacles") or referee.c_obstacles(obstacles, robot)
    fig, ax = _new_axes(figsize)
    pts = [start, goal]
    shown = [(k, c) for k, c in enumerate(c_obstacles) if len(c) >= 3]
    for k, c in shown:
        ax.add_patch(MplPolygon(c, closed=True, facecolor=COBSTACLE_FACE, edgecolor=COBSTACLE_EDGE, lw=0.8,
                                zorder=2, label=COBSTACLE_LABEL if k == shown[0][0] else None))
        pts += list(c)
    for k, c in shown:   # every outline above every fill: C-obstacles that touch along an edge show that edge
        ax.add_patch(MplPolygon(c, closed=True, fill=False, edgecolor=COBSTACLE_EDGE, lw=0.8, zorder=2.5,
                                gid=f"c-obstacle-outline-{k}"))
    n_obs = len(obstacles)
    obstacle_label = "obstacle 0" if n_obs == 1 else f"obstacles 0 to {n_obs - 1}"
    for k, o in enumerate(obstacles):
        ax.add_patch(MplPolygon(o, closed=True, facecolor=OBSTACLE_COLOR, edgecolor="black", lw=0.6, zorder=3,
                                label=obstacle_label if k == 0 else None))
        pts += list(o)
    for k, o in enumerate(obstacles):   # numbered as in the messages of check_ce.py and hw1_io.py
        cx, cy = sum(float(p[0]) for p in o) / len(o), sum(float(p[1]) for p in o) / len(o)
        ax.text(cx, cy, str(k), color="white", fontsize=6.5, fontweight="bold", ha="center", va="center",
                zorder=3.5, gid=f"obstacle-number-{k}",
                bbox=dict(boxstyle="round,pad=0.12", fc=OBSTACLE_COLOR, ec="none", alpha=0.85))
    for p, color, label in ((start, START_COLOR, "robot at start"), (goal, GOAL_COLOR, "robot at goal")):
        body = placed(robot, p)
        ax.add_patch(MplPolygon(body, closed=True, facecolor=color, edgecolor="black", alpha=0.85, lw=0.6,
                                zorder=5, label=label))
        ax.plot([p[0]], [p[1]], "o", color="white", mec="black", mew=0.5, ms=3, zorder=6)
        pts += body

    _, rpath, rcost = _answer_parts(referee_answer)
    if rpath:
        _draw_path(ax, rpath, REFEREE_COLOR, f"referee (cost {rcost:.4f})", 2.6, "-", "o", 3.5, 7)
        pts += rpath
    else:
        ax.plot([], [], " ", label="referee: no path")
    if planner:
        status, ppath, pcost = _answer_parts(planner_answer)
        if status == "done" and ppath:
            cost_txt = f"{pcost:.4f}" if math.isfinite(pcost) else str(pcost)
            _draw_path(ax, ppath, PLANNER_COLOR, f"{planner} (cost {cost_txt})", 1.3, "--", "s", 2.6, 8)
            pts += [p for p in ppath if math.isfinite(p[0]) and math.isfinite(p[1])]
        else:
            what = {"done": "no path", "timeout": "timeout", "crash": "crash"}.get(status, "not run")
            ax.plot([], [], " ", label=f"{planner}: {what}")
    if title:
        ax.set_title(title, fontsize=8)
    return _finish(fig, ax, out, pts)


def draw_warmup(out, robot, obstacle, figsize=FIGSIZE):
    """Draw O, the C-obstacle O (+) (-R) with its vertex coordinates, and the robot at each
    C-obstacle vertex. Returns (path of the written file, C-obstacle vertices)."""
    robot = [(float(x), float(y)) for x, y in robot]
    co = referee.c_obstacle(obstacle, robot)
    fig, ax = _new_axes(figsize)
    ax.add_patch(MplPolygon(co, closed=True, facecolor=COBSTACLE_FACE, edgecolor=COBSTACLE_EDGE, lw=0.8, zorder=2,
                            label=COBSTACLE_LABEL))
    ax.add_patch(MplPolygon(obstacle, closed=True, facecolor=OBSTACLE_COLOR, edgecolor="black", lw=0.6, zorder=3,
                            label="obstacle $O$"))
    pts = list(co) + list(obstacle)
    for k, v in enumerate(co):
        body = placed(robot, v)
        ax.add_patch(MplPolygon(body, closed=True, facecolor=ROBOT_COPY_COLOR, edgecolor=ROBOT_COPY_COLOR,
                                alpha=0.25, lw=0.8, zorder=4,
                                label="robot $R$ placed at a vertex" if k == 0 else None))
        pts += body
    xs, ys = zip(*co)
    ax.plot(xs, ys, "o", color="black", ms=2.5, zorder=6, label="reference point")
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    span = max(max(p[0] for p in pts) - min(p[0] for p in pts), max(p[1] for p in pts) - min(p[1] for p in pts))
    for x, y in co:
        dx, dy = x - cx, y - cy
        n = math.hypot(dx, dy) or 1.0
        ax.annotate(f"({round(x, 3):g}, {round(y, 3):g})", (x, y), xytext=(7 * dx / n, 7 * dy / n),
                    textcoords="offset points", fontsize=6.5, zorder=9,
                    ha="left" if dx > 0.3 * n else ("right" if dx < -0.3 * n else "center"),
                    va="bottom" if dy > 0.3 * n else ("top" if dy < -0.3 * n else "center"),
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
        pts.append((x + 0.12 * span * dx / n, y + 0.08 * span * dy / n))   # room for the label
    return _finish(fig, ax, out, pts), co


def _default_out(name, fmt):
    return Path("figs") / f"{name}.{fmt or 'pdf'}"


def main(argv=None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description="HW1 figures: an instance with its C-obstacles and paths, "
                                             "or the Part A warm-up (236767).")
    ap.add_argument("--ids", nargs="+", required=True, metavar="ID", help="one or two student IDs")
    ap.add_argument("instance", nargs="?", help="an instance file, e.g. counterexamples/P<k>.json")
    ap.add_argument("--warmup", action="store_true", help="draw the Part A warm-up instead of an instance")
    ap.add_argument("--planner", help="also run planner P<k> on the instance and draw its path")
    ap.add_argument("--planners-dir", help="folder with P1.py ... P8.py (default: planners/ next to this file)")
    ap.add_argument("--out", help="output file, .pdf or .png (default: figs/<instance name>.pdf)")
    ap.add_argument("--format", choices=["pdf", "png"], help="extension of the default output file")
    ap.add_argument("--timeout", type=float, default=check_ce.TIMEOUT_S, help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    a.ids, a.instance = check_ce.split_ids(a.ids, a.instance)
    if len(a.ids) > 2:
        ap.error("pass one or two IDs")
    if a.warmup == bool(a.instance):
        ap.error("give an instance file, or --warmup")
    if a.out and Path(a.out).suffix.lower() not in (".pdf", ".png"):
        ap.error("--out must end in .pdf or .png")
    try:
        ids = gen_hw1.normalize_ids(a.ids)
    except ValueError as exc:
        ap.error(str(exc))
    warning = check_ce.kit_warning(planners_dir=a.planners_dir)
    if warning:
        print(warning, flush=True)
    robot = gen_hw1.robot_for_ids(ids)
    rtxt = " ".join(check_ce.fmt_pt(p) for p in robot)

    if a.warmup:
        obstacle = gen_hw1.warmup_obstacle(ids)
        out, co = draw_warmup(a.out or _default_out("warmup", a.format), robot, obstacle)
        print(f"Part A warm-up for {' '.join(ids)}: robot R = {rtxt}")
        print("  obstacle O = " + " ".join(check_ce.fmt_pt(p) for p in obstacle))
        print(f"  C-obstacle O (+) (-R): {len(co)} vertices, counter-clockwise from the lowest: "
              + " ".join(check_ce.fmt_pt((round(x, 3), round(y, 3))) for x, y in co))
        print(f"Figure written to {out}")
        return 0

    try:
        data = Path(a.instance).read_bytes()
    except OSError as exc:
        print(f"draw.py: cannot read {a.instance}: {exc.strerror or exc}", file=sys.stderr)
        return 2
    raw, inst, problems = check_ce.read_instance_bytes(data)
    if inst is None:
        print(f"draw.py: {a.instance} cannot be drawn: " + "; ".join(problems), file=sys.stderr)
        return 2
    if problems:
        print(f"WARNING: {a.instance} breaks {len(problems)} rule(s) (drawn anyway): " + "; ".join(problems))
    obstacles, start, goal = inst["obstacles"], inst["start"], inst["goal"]
    try:
        ref = referee.solve(robot, obstacles, start, goal)
    except Exception as exc:
        print(f"draw.py: the referee failed on this instance ({type(exc).__name__}: {exc})", file=sys.stderr)
        return 2
    print(f"Robot of {' '.join(ids)}: {rtxt}; {len(obstacles)} obstacle(s)"
          + (f", numbered 0 to {len(obstacles) - 1} in the figure" if len(obstacles) > 1 else ""))
    print(f"  referee: {check_ce._fmt_answer(check_ce._answer(ref['path'], ref['cost']))}")
    name, ans = None, None
    if a.planner:
        try:
            name = check_ce.planner_for(a.instance, a.planner)
        except check_ce.CheckError as exc:
            ap.error(str(exc))
        if problems:
            print(f"  {name}: not run, since the instance breaks the rules")
            ans = {"status": "not run"}
        else:
            pdir = Path(a.planners_dir) if a.planners_dir else HERE / "planners"
            ans = check_ce.run_planner(pdir / f"{name}.py", robot, obstacles, start, goal, a.timeout)
            ans["name"] = name
            if ans["status"] == "error":
                print(f"  {name}: not run: {ans['message']}")
            else:
                if ans["status"] == "done":
                    print(f"  {name}: {check_ce._fmt_answer(check_ce._answer(ans['path'], ans['cost']))}")
                kind, detail = check_ce.classify(ans, ref, robot, obstacles, start, goal)
                print(f"  {name}: " + (f"COUNTEREXAMPLE ({kind}): {detail}" if kind else "NOT A COUNTEREXAMPLE"))
    out = draw_instance(a.out or _default_out(Path(a.instance).stem, a.format), robot, obstacles, start, goal,
                        c_obstacles=ref["c_obstacles"], referee_answer=ref, planner=name, planner_answer=ans)
    print(f"Figure written to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
