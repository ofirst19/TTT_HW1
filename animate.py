# =============================================================================
# Kit file (236767 HW1, kit version 2.4).  Animates a path for you to look at;
# nothing it writes is graded, and report figures come from draw.py.
# =============================================================================
"""Animate the robot along a path, in the workspace and in the configuration space.

Usage
-----
  python animate.py --ids ID1 [ID2] counterexamples/P<k>.json [--planner P<k>] [--referee]
                    [--path FILE] [--save FILE.html|FILE.gif] [--no-show] [--seconds S]

Opens a window with two panels side by side. The left panel (workspace) shows the
obstacles, numbered 0, 1, ... as in draw.py and check_ce.py, and your pair's robot moving
along the path. The right panel (configuration space) shows the C-obstacles O (+) (-R)
and the robot's reference point moving along the same path. The robot is
  green   when it is free,
  orange  when it touches an obstacle (allowed: touching, sliding along an edge, and
          passing a vertex),
  red     when it overlaps an obstacle, that is, when its reference point is in the
          interior of that obstacle's C-obstacle; the overlap is filled dark red in the
          left panel, and the C-obstacle is outlined red in the right one.
The overlap test is the referee's (the 1e-9 rule of check_ce.py), so the robot turns red
exactly where check_ce.py finds the path invalid. Every stretch of the path inside a
C-obstacle gets frames, however short it is, and the animation pauses for a second at the
first frame of each. The tool also prints these stretches.

Which path is animated:
  --planner P<k>   the path of P<k> (run as check_ce.py runs it), with the referee's path
                   drawn faintly; when P<k> returns no path, the referee's path;
  (nothing)        the referee's path;
  --referee        the referee's path, even when P<k> returns one;
  --path FILE      a path of your own: a JSON list of points [[x, y], ...], or a JSON
                   object with such a list under "path".

Window controls: space pauses and resumes (at the end, it starts again); the left and right
arrow keys step one frame back and forward; Home goes back to the start; the slider below
the panels scrubs.

Where no window can open (no display, or a Python without tkinter), the tool writes the
animation as an HTML page with its own play controls, figs/<instance name>_anim.html, and
opens it in the web browser. --save FILE also writes the animation, as .html or .gif;
--no-show skips the window. In a notebook:
    from animate import animate_instance
    anim = animate_instance(robot, obstacles, start, goal, path)
    from IPython.display import HTML; HTML(anim.to_jshtml())

Importable: animate_instance(...), inside_stretches(path, c_obstacles), and
make_frames(path, c_obstacles).
"""
from __future__ import annotations

import argparse
import bisect
import json
import logging
import math
import os
import sys
import webbrowser
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import check_ce  # noqa: E402
import draw  # noqa: E402  (selects matplotlib's file-only backend; see _open_window)
import gen_hw1  # noqa: E402
import matplotlib  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Polygon as MplPolygon  # noqa: E402
from referee import geom_ref, referee  # noqa: E402
from shapely.geometry import Polygon as ShPolygon  # noqa: E402

logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)   # font fallbacks are harmless here

FREE_COLOR = draw.START_COLOR
TOUCH_COLOR = "#e08a00"
HIT_COLOR = "#d62728"
OVERLAP_COLOR = "#5c0000"   # the overlap itself, darker than the red robot
TRACE_COLOR = "#555555"
FPS = 20
SECONDS = 6.0          # time for the motion itself, without the pauses
HOLD_HIT = FPS         # frames of pause at the first frame of each stretch inside a C-obstacle
HOLD_ENDS = FPS // 2   # frames of pause at the start and at the end
TOUCH_TOL = 1e-7       # display only: a placement this close to a C-obstacle counts as touching it
FIGSIZE = (10.0, 5.6)
HTML_DPI = 80


# ---------------------------------------------------------------------------
# Geometry: the referee's interior test, segment by segment
# ---------------------------------------------------------------------------

def _halfplanes(c_obstacles):
    """[(j, inward unit normals, offsets)] for every C-obstacle j with a nonempty interior."""
    out = []
    for j, c in enumerate(c_obstacles):
        hp = geom_ref._halfplanes(c)
        if hp is not None:
            out.append((j, hp[0], hp[1]))
    return out


def _segment_stretches(a, b, hps, eps=geom_ref.EPS):
    """[(j, t0, t1)]: the open interval of t in [0, 1] on which a + t (b - a) is in the interior
    of C-obstacle j by more than eps (the test of geom_ref.segments_valid), one per entered j."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    out = []
    for j, n, c in hps:
        s0 = n @ a - c
        ds = n @ b - c - s0
        if np.any((ds == 0) & (s0 <= eps)):   # parallel to an edge and never deep enough
            continue
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (eps - s0) / ds
        lo = max(0.0, float(t[ds > 0].max()) if np.any(ds > 0) else -math.inf)
        hi = min(1.0, float(t[ds < 0].min()) if np.any(ds < 0) else math.inf)
        if lo < hi:
            out.append((j, lo, hi))
    return out


def inside_stretches(path, c_obstacles, eps=geom_ref.EPS):
    """[(segment index i (0-based), C-obstacle j, t0, t1)]: the stretches of the path in the interior
    of a C-obstacle, where the robot overlaps obstacle j. Empty iff no segment fails the referee's test."""
    hps = _halfplanes(c_obstacles)
    segs = list(zip(path[:-1], path[1:])) or [(path[0], path[0])]
    return [(i, j, t0, t1) for i, (a, b) in enumerate(segs) for j, t0, t1 in _segment_stretches(a, b, hps, eps)]


def _status(p, hps, eps=geom_ref.EPS):
    """(obstacles overlapped, obstacles touched) by the robot with its reference point at p."""
    hits, touches = [], []
    q = np.asarray(p, float)
    for j, n, c in hps:
        m = float((n @ q - c).min())   # > 0 inside; the distance to the boundary for a convex polygon
        if m > eps:
            hits.append(j)
        elif m >= -TOUCH_TOL:
            touches.append(j)
    return hits, touches


def make_frames(path, c_obstacles, seconds=SECONDS, fps=FPS):
    """The frames of the animation: [(placement, segment index, obstacles overlapped, obstacles touched)].
    Evenly spaced along the path, plus every vertex, plus three placements inside every stretch of
    inside_stretches; each stretch's first frame is held for HOLD_HIT frames."""
    path = [(float(x), float(y)) for x, y in path]
    hps = _halfplanes(c_obstacles)
    lens = [math.dist(a, b) for a, b in zip(path[:-1], path[1:])]
    cum = [0.0]
    for d in lens:
        cum.append(cum[-1] + d)
    total = cum[-1]
    if total <= 0.0 or len(path) < 2:
        hits, touches = _status(path[0], hps)
        return [(path[0], 0, hits, touches)] * (2 * HOLD_ENDS + 1)
    n = max(2, int(round(seconds * fps)))
    marks = set()   # (arc length, segment index)
    for s in [total * k / (n - 1) for k in range(n)] + cum:
        i = min(max(bisect.bisect_right(cum, s) - 1, 0), len(lens) - 1)
        while lens[i] == 0.0 and i + 1 < len(lens):
            i += 1
        marks.add((s, i))
    marks.update((cum[i], i) for i, d in enumerate(lens) if d == 0.0)   # a repeated point is a segment too
    for i, _, t0, t1 in inside_stretches(path, c_obstacles):
        for t in (t0 + 0.02 * (t1 - t0), 0.5 * (t0 + t1), t1 - 0.02 * (t1 - t0)):
            marks.add((cum[i] + t * lens[i], i))
    frames = []
    for s, i in sorted(marks):
        t = 0.0 if lens[i] == 0.0 else min(max((s - cum[i]) / lens[i], 0.0), 1.0)
        (ax, ay), (bx, by) = path[i], path[i + 1]
        p = (ax + t * (bx - ax), ay + t * (by - ay))
        frames.append((p, i, *_status(p, hps)))
    out = [frames[0]] * HOLD_ENDS
    prev_hit = False
    for f in frames:
        out += [f] * (HOLD_HIT if f[2] and not prev_hit else 1)
        prev_hit = bool(f[2])
    return out + [frames[-1]] * HOLD_ENDS


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def _bounds(pts):
    xs = [p[0] for p in pts if math.isfinite(p[0]) and math.isfinite(p[1])]
    ys = [p[1] for p in pts if math.isfinite(p[0]) and math.isfinite(p[1])]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    pad = 0.06 * span
    return (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)


def _build(robot, obstacles, start, goal, path, label, c_obstacles, ghost_path=None, ghost_label=None,
           seconds=SECONDS, slider_room=False):
    """(figure, update(frame index), number of frames)."""
    robot = [(float(x), float(y)) for x, y in robot]
    frames = make_frames(path, c_obstacles, seconds)
    fig, (axw, axc) = plt.subplots(1, 2, figsize=FIGSIZE, sharex=True, sharey=True)
    fig.subplots_adjust(left=0.05, right=0.99, top=0.86, bottom=0.2 if slider_room else 0.12, wspace=0.08)
    for ax, title in ((axw, "workspace: obstacles and the robot"),
                      (axc, r"configuration space: C-obstacles $O \oplus (-R)$ and the reference point")):
        ax.set_aspect("equal")
        ax.tick_params(labelsize=7)
        ax.set_title(title, fontsize=9)

    obstacle_patches, cobstacle_patches = {}, {}
    for k, o in enumerate(obstacles):
        obstacle_patches[k] = axw.add_patch(MplPolygon(o, closed=True, facecolor=draw.OBSTACLE_COLOR,
                                                       edgecolor="black", lw=0.6, zorder=3))
        cx, cy = sum(float(p[0]) for p in o) / len(o), sum(float(p[1]) for p in o) / len(o)
        axw.text(cx, cy, str(k), color="white", fontsize=7, fontweight="bold", ha="center", va="center", zorder=3.5)
    for k, c in enumerate(c_obstacles):
        if len(c) < 3:
            continue
        cobstacle_patches[k] = axc.add_patch(MplPolygon(c, closed=True, facecolor=draw.COBSTACLE_FACE,
                                                        edgecolor=draw.COBSTACLE_EDGE, lw=0.8, zorder=2))
        cx, cy = sum(p[0] for p in c) / len(c), sum(p[1] for p in c) / len(c)
        axc.text(cx, cy, str(k), color=draw.COBSTACLE_EDGE, fontsize=7, fontweight="bold", ha="center",
                 va="center", zorder=2.5)
    for p, color in ((start, draw.START_COLOR), (goal, draw.GOAL_COLOR)):
        axw.add_patch(MplPolygon(draw.placed(robot, p), closed=True, fill=False, edgecolor=color, lw=1.0,
                                 ls="--", zorder=4))
        for ax in (axw, axc):
            ax.plot([p[0]], [p[1]], "o", color=color, mec="black", mew=0.4, ms=4, zorder=6)
    if ghost_path:
        gx, gy = zip(*ghost_path)
        axc.plot(gx, gy, "-", color=draw.REFEREE_COLOR, lw=2.4, alpha=0.45, zorder=3, label=ghost_label)
    px, py = zip(*path)
    for ax in (axw, axc):
        ax.plot(px, py, ":", color=TRACE_COLOR, lw=0.8, zorder=4, label=label if ax is axc else None)
    trace_w, = axw.plot([], [], "-", color=TRACE_COLOR, lw=1.2, zorder=4.5)
    trace_c, = axc.plot([], [], "-", color=TRACE_COLOR, lw=1.6, zorder=4.5)
    body = axw.add_patch(MplPolygon(draw.placed(robot, path[0]), closed=True, facecolor=FREE_COLOR,
                                    edgecolor="black", alpha=0.75, lw=0.7, zorder=5))
    overlaps = {k: axw.add_patch(MplPolygon([(0, 0)] * 3, closed=True, facecolor=OVERLAP_COLOR, edgecolor=OVERLAP_COLOR,
                                            lw=1.2, zorder=5.5, visible=False)) for k in obstacle_patches}
    dot, = axc.plot([], [], "o", ms=7, mec="black", mew=0.6, zorder=7)
    axc.legend(loc="upper left", fontsize=7, framealpha=0.85)

    pts = [p for o in obstacles for p in o] + [p for c in c_obstacles for p in c] + list(path)
    pts += [q for p in list(path) + [start, goal] for q in draw.placed(robot, p)]
    (x0, x1), (y0, y1) = _bounds(pts)
    axw.set_xlim(x0, x1)
    axw.set_ylim(y0, y1)
    fig.text(0.5, 0.975, label, ha="center", va="top", fontsize=10, fontweight="bold")
    status = fig.text(0.5, 0.925, "", ha="center", va="top", fontsize=9)
    fig.text(0.5, 0.02 if not slider_room else 0.085,
             "robot green: free    orange: touching an obstacle (allowed)    "
             "red: overlapping an obstacle (the path is invalid)", ha="center", va="bottom", fontsize=8)
    sh_obstacles = {k: ShPolygon(o) for k, o in enumerate(obstacles)}
    xs_all = [f[0][0] for f in frames]
    ys_all = [f[0][1] for f in frames]
    nseg = max(len(path) - 1, 1)

    def update(i):
        p, seg, hits, touches = frames[i]
        placed = draw.placed(robot, p)
        body.set_xy(placed)
        body.set_facecolor(HIT_COLOR if hits else (TOUCH_COLOR if touches else FREE_COLOR))
        dot.set_data([p[0]], [p[1]])
        dot.set_color(HIT_COLOR if hits else (TOUCH_COLOR if touches else FREE_COLOR))
        dot.set_markeredgecolor("black")
        trace_w.set_data(xs_all[:i + 1], ys_all[:i + 1])
        trace_c.set_data(xs_all[:i + 1], ys_all[:i + 1])
        shape = ShPolygon(placed)
        for k, patch in overlaps.items():
            inter = shape.intersection(sh_obstacles[k]) if k in hits else None
            if inter is not None and inter.geom_type == "Polygon" and not inter.is_empty:
                patch.set_xy(list(inter.exterior.coords))
                patch.set_visible(True)
            else:
                patch.set_visible(False)
            obstacle_patches[k].set_edgecolor(HIT_COLOR if k in hits else "black")
            obstacle_patches[k].set_linewidth(2.0 if k in hits else 0.6)
        for k, patch in cobstacle_patches.items():
            patch.set_edgecolor(HIT_COLOR if k in hits else draw.COBSTACLE_EDGE)
            patch.set_linewidth(2.2 if k in hits else 0.8)
        where = f"segment {seg + 1} of {nseg}, reference point at {check_ce.fmt_pt((round(p[0], 4), round(p[1], 4)))}"
        if hits:
            what = "OVERLAPS obstacle " + ", ".join(map(str, hits)) + " (inside its C-obstacle)"
        elif touches:
            what = "touches obstacle " + ", ".join(map(str, touches)) + " (allowed)"
        else:
            what = "free"
        status.set_text(f"{where}: {what}")
        status.set_color(HIT_COLOR if hits else "black")
        return []

    update(0)
    return fig, update, len(frames)


def animate_instance(robot, obstacles, start, goal, path, label="path", c_obstacles=None, ghost_path=None,
                     ghost_label=None, seconds=SECONDS):
    """A matplotlib FuncAnimation of the robot along path (for .to_jshtml() or .save())."""
    if c_obstacles is None:
        c_obstacles = referee.c_obstacles(obstacles, robot)
    fig, update, n = _build(robot, obstacles, start, goal, path, label, c_obstacles, ghost_path, ghost_label,
                            seconds)
    return FuncAnimation(fig, update, frames=n, interval=1000 / FPS, repeat=False)


def save_animation(anim, out, title="HW1 animation"):
    """Write anim as .html (a page with play controls) or .gif. Returns the path."""
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim._fig.set_dpi(HTML_DPI)
    if out.suffix.lower() == ".gif":
        anim.save(out, writer=PillowWriter(fps=FPS), dpi=HTML_DPI)
    else:
        matplotlib.rcParams["animation.embed_limit"] = 200   # MB
        page = anim.to_jshtml(fps=FPS, default_mode="once")
        out.write_text(f'<!doctype html>\n<html><head><meta charset="utf-8"><title>{title}</title></head>\n'
                       f'<body style="font-family:sans-serif">\n{page}\n</body></html>\n', encoding="utf-8")
    return out


def _window_backends():
    """Interactive backends to try, best first; none where no display can exist."""
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return []
    return (["macosx", "TkAgg", "QtAgg"] if sys.platform == "darwin" else ["TkAgg", "QtAgg"])


def _open_window(make_figure):
    """Show the animation in a window with play controls. False if no window could be opened."""
    for name in _window_backends():
        try:
            plt.switch_backend(name)
            for key, drop in (("keymap.back", ("left",)), ("keymap.forward", ("right",)), ("keymap.home", ("home",))):
                matplotlib.rcParams[key] = [k for k in matplotlib.rcParams[key] if k not in drop]
            fig, update, n = make_figure(slider_room=True)
        except Exception:
            continue
        break
    else:
        plt.switch_backend("Agg")
        return False

    from matplotlib.widgets import Slider
    state = {"i": 0, "playing": True}
    slider = Slider(fig.add_axes([0.2, 0.13, 0.6, 0.03]), "frame", 0, n - 1, valinit=0, valstep=1)

    def go(i):
        state["i"] = i = min(max(int(i), 0), n - 1)
        update(i)
        if int(slider.val) != i:
            slider.eventson = False
            slider.set_val(i)
            slider.eventson = True
        fig.canvas.draw_idle()

    def tick():
        if state["playing"]:
            if state["i"] < n - 1:
                go(state["i"] + 1)
            else:
                state["playing"] = False

    def on_key(event):
        if event.key == " ":
            if not state["playing"] and state["i"] >= n - 1:
                go(0)
            state["playing"] = not state["playing"]
        elif event.key in ("left", "right"):
            state["playing"] = False
            go(state["i"] + (1 if event.key == "right" else -1))
        elif event.key == "home":
            go(0)

    def on_slide(val):
        state["playing"] = False
        go(val)

    slider.on_changed(on_slide)
    fig.canvas.mpl_connect("key_press_event", on_key)
    timer = fig.canvas.new_timer(interval=int(1000 / FPS))
    timer.add_callback(tick)
    fig._hw1_keep = (slider, timer)   # keep the widgets alive while the window is open
    timer.start()
    print("Window: space pauses and resumes, the left and right arrows step, Home restarts; close it to end.",
          flush=True)
    plt.show()
    return True


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def _read_path(fname):
    data = json.loads(Path(fname).read_text(encoding="utf-8-sig"))
    if isinstance(data, dict):
        data = data.get("path")
    if not isinstance(data, list) or not data:
        raise ValueError('expected a JSON list of points [[x, y], ...], or an object with such a list under "path"')
    pts = []
    for q in data:
        if not (isinstance(q, (list, tuple)) and len(q) == 2):
            raise ValueError(f"{q!r} is not a point [x, y]")
        x, y = float(q[0]), float(q[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"{q!r} has a coordinate that is not a finite number")
        pts.append((x, y))
    return pts


def _report(path, label, c_obstacles):
    st = inside_stretches(path, c_obstacles)
    print(f"Animating {label}: {max(len(path) - 1, 0)} segment(s), length {check_ce.path_length(path):.6g}")
    if not st:
        print("  the robot never overlaps an obstacle (touching, sliding, and passing vertices are allowed)")
    for i, j, t0, t1 in st:
        (ax, ay), (bx, by) = path[i], path[i + 1] if len(path) > 1 else path[i]
        p0 = (round(ax + t0 * (bx - ax), 4), round(ay + t0 * (by - ay), 4))
        p1 = (round(ax + t1 * (bx - ax), 4), round(ay + t1 * (by - ay), 4))
        print(f"  segment {i + 1}: the robot overlaps obstacle {j} from {check_ce.fmt_pt(p0)} to {check_ce.fmt_pt(p1)}"
              f" (inside its C-obstacle)")
    return st


def main(argv=None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description="HW1: animate the robot along a path, in the workspace and in the "
                                             "configuration space (236767).")
    ap.add_argument("--ids", nargs="+", required=True, metavar="ID", help="one or two student IDs")
    ap.add_argument("instance", nargs="?", help="an instance file, e.g. counterexamples/P<k>.json")
    ap.add_argument("--planner", help="run planner P<k> on the instance and animate its path")
    ap.add_argument("--referee", action="store_true", help="animate the referee's path, even with --planner")
    ap.add_argument("--path", metavar="FILE", help="animate your own path: a JSON list of points [[x, y], ...]")
    ap.add_argument("--save", metavar="FILE", help="also write the animation, .html or .gif")
    ap.add_argument("--no-show", action="store_true", help="open no window (with --save: only write the file)")
    ap.add_argument("--seconds", type=float, default=SECONDS, help=f"duration of the motion (default {SECONDS:g})")
    ap.add_argument("--planners-dir", help="folder with P1.py ... P8.py (default: planners/ next to this file)")
    ap.add_argument("--timeout", type=float, default=check_ce.TIMEOUT_S, help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    a.ids, a.instance = check_ce.split_ids(a.ids, a.instance)
    if len(a.ids) > 2:
        ap.error("pass one or two IDs")
    if not a.instance:
        ap.error("give an instance file")
    if a.save and Path(a.save).suffix.lower() not in (".html", ".gif"):
        ap.error("--save must end in .html or .gif")
    if a.path and a.planner:
        ap.error("give --planner or --path, not both")
    if not (a.seconds > 0):
        ap.error("--seconds must be positive")
    try:
        ids = gen_hw1.normalize_ids(a.ids)
    except ValueError as exc:
        ap.error(str(exc))
    warning = check_ce.kit_warning(planners_dir=a.planners_dir)
    if warning:
        print(warning, flush=True)
    robot = gen_hw1.robot_for_ids(ids)

    try:
        data = Path(a.instance).read_bytes()
    except OSError as exc:
        print(f"animate.py: cannot read {a.instance}: {exc.strerror or exc}", file=sys.stderr)
        return 2
    raw, inst, problems = check_ce.read_instance_bytes(data)
    if inst is None:
        print(f"animate.py: {a.instance} cannot be animated: " + "; ".join(problems), file=sys.stderr)
        return 2
    if problems:
        print(f"WARNING: {a.instance} breaks {len(problems)} rule(s) (animated anyway): " + "; ".join(problems))
    obstacles, start, goal = inst["obstacles"], inst["start"], inst["goal"]
    try:
        ref = referee.solve(robot, obstacles, start, goal)
    except Exception as exc:
        print(f"animate.py: the referee failed on this instance ({type(exc).__name__}: {exc})", file=sys.stderr)
        return 2
    rtxt = " ".join(check_ce.fmt_pt(p) for p in robot)
    print(f"Robot of {' '.join(ids)}: {rtxt}; {len(obstacles)} obstacle(s)")
    print(f"  referee: {check_ce._fmt_answer(check_ce._answer(ref['path'], ref['cost']))}")

    path, label, ghost, ghost_label = None, None, None, None
    if a.path:
        try:
            path = _read_path(a.path)
        except (OSError, ValueError) as exc:
            print(f"animate.py: cannot read the path in {a.path}: {exc}", file=sys.stderr)
            return 2
        label = f"your path ({Path(a.path).name})"
        ghost, ghost_label = ref["path"], "referee's path"
    elif a.planner:
        try:
            name = check_ce.planner_for(a.instance, a.planner)
        except check_ce.CheckError as exc:
            ap.error(str(exc))
        if problems:
            print(f"  {name}: not run, since the instance breaks the rules")
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
                ppath = ans.get("path") if ans["status"] == "done" else None
                if ppath and all(math.isfinite(x) and math.isfinite(y) for x, y in ppath):
                    if a.referee:
                        print(f"  --referee: animating the referee's path, not {name}'s")
                    else:
                        path = [(float(x), float(y)) for x, y in ppath]
                        label = f"{name}'s path"
                        ghost, ghost_label = ref["path"], "referee's path"
                elif ppath:
                    print(f"  {name}'s path has a coordinate that is not a finite number; animating the referee's path")
                else:
                    print(f"  {name} returns no path; animating the referee's path")
    if path is None:
        if not ref["path"]:
            print("No path to animate: the referee finds no path and no other path was given "
                  "(--planner P<k> or --path FILE).")
            return 0
        path, label = [(float(x), float(y)) for x, y in ref["path"]], "referee's path"
    _report(path, label, ref["c_obstacles"])

    def make_figure(slider_room=False):
        return _build(robot, obstacles, start, goal, path, label, ref["c_obstacles"], ghost, ghost_label,
                      a.seconds, slider_room)

    def write(out):
        fig, update, n = make_figure()
        anim = FuncAnimation(fig, update, frames=n, interval=1000 / FPS, repeat=False)
        out = save_animation(anim, out, title=f"HW1: {label} on {Path(a.instance).name}")
        plt.close(fig)
        print(f"Animation written to {out}")
        return out

    if a.save:
        write(a.save)
    if a.no_show:
        return 0
    if _open_window(make_figure):
        return 0
    if a.save and Path(a.save).suffix.lower() == ".html":
        out = Path(a.save)
    else:
        print("No window can be opened here (no display, or no tkinter); writing an HTML page instead.")
        out = write(Path("figs") / f"{Path(a.instance).stem}_anim.html")
    try:
        opened = webbrowser.open(out.resolve().as_uri())
    except Exception:
        opened = False
    if not opened:
        print(f"Open {out} in a web browser to play it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
