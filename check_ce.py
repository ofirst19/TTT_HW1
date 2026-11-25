# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Check a counterexample: run one planner and the referee on one instance.

Usage
-----
  python check_ce.py --ids ID1 [ID2] counterexamples/P<k>.json [--draw fig.pdf] [--json]

The file name picks the planner: P<k>.json (or a draft such as P<k>_try2.json) is
checked against planners/P<k>.py; --planner P<k> picks the planner explicitly. The
robot is your pair's robot (python gen_hw1.py --ids ID1 ID2 --robot).

Steps:
  1. The instance must follow the rules of hw1_io.validate_instance; if it breaks
     one, the tool prints INVALID INSTANCE with every broken rule and runs nothing.
     The coordinates are snapped to the 0.001 grid (hw1_io.parse_instance).
  2. The planner's plan(robot, obstacles, start, goal) runs in a separate Python
     process that receives the robot and the instance and nothing else, with a
     limit of 30 s on the call.
  3. The referee (referee/referee.py) solves the same instance.
  4. The tool prints the size (the number of obstacles), both answers, and
     COUNTEREXAMPLE (kind) or NOT A COUNTEREXAMPLE.

The planner is wrong on the instance (COUNTEREXAMPLE) if
  missed_path   it returns no path, but a path exists;
  false_path    it returns a path, but no path exists;
  invalid_path  its path has a point in the interior of a C-obstacle, or does not
                start at the start and end at the goal (within 1e-6);
  suboptimal    its path is longer than the optimum by more than 1e-6;
  wrong_cost    the cost it reports differs from the length of its path by more
                than 1e-6 (when it returns no path, the cost must be inf);
  crash         it raises an exception, or returns something that is not a pair
                (path, cost);
  timeout       the call takes more than 30 s.
When several apply, the first of timeout, crash, missed_path, false_path,
invalid_path, suboptimal, wrong_cost is reported.

Options:
  --draw FILE        also draw the instance, the C-obstacles, and both paths
                     (draw.py; FILE ends in .pdf or .png)
  --json             print one JSON object instead of the text report
  --planner P<k>     the planner to run (default: from the file name)
  --planners-dir DIR folder with P1.py ... P8.py (default: planners/ of this kit)

Exit code: 0 for COUNTEREXAMPLE, 1 for NOT A COUNTEREXAMPLE, 2 for an invalid
instance or a check that could not be completed.

The tool warns (and still runs) when a DO-NOT-EDIT file of the kit, a planner
included, differs from the original recorded in MANIFEST.json: the grader uses the
original files.

The planner's process is an isolated Python (python -I): it ignores PYTHONPATH and
the user site-packages, so numpy, scipy, shapely, networkx, and matplotlib must be
installed in the interpreter itself (a virtual environment, README.md). If the
planner cannot import them, the check is not completed and the tool says that the
Python environment is broken; this is no verdict on the planner or the referee.

Importable: check(path, ids, ...) returns the record that --json prints, and
run_planner(...) runs a planner in its own process.
"""
from __future__ import annotations

# Only the standard library at module level: this file also runs as the planner's
# process (--child), which must not import the referee or any other kit module.
import argparse
import hashlib
import importlib.util
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
KIT_VERSION = "2.4"
TIMEOUT_S = 30.0         # limit on one call of plan()
STARTUP_S = 180.0        # wall time allowed for starting Python and importing the planner (not part of the limit)
PLAN_GRACE_S = 5.0       # the parent stops a call that its own watchdog failed to stop after the limit + this
COST_TOL = 1e-6          # suboptimal and wrong_cost
ENDPOINT_TOL = 1e-6      # the path must start at the start and end at the goal within this distance
KINDS = ("missed_path", "false_path", "invalid_path", "suboptimal", "wrong_cost", "crash", "timeout")
PLANNERS = tuple(f"P{k}" for k in range(1, 9))
VERDICTS = ("counterexample", "not_a_counterexample")
RUNNER_FAILED = 97       # exit code of the planner's process when the runner itself failed
NAME_RE = re.compile(r"^[Pp]([1-8])(?:[_\-. ].*)?$")
MANIFEST = "MANIFEST.json"   # SHA-256 of the original DO-NOT-EDIT files of the kit
SANDBOX_PACKAGES = ("numpy", "scipy", "shapely", "networkx", "matplotlib")   # requirements.txt


class CheckError(Exception):
    """The check could not be completed (not a verdict on the planner)."""


class EnvironmentBroken(CheckError):
    """The Python environment cannot import the kit's packages (not a verdict on anything)."""


def env_trouble(exc=None):
    """Why a failure looks like a broken Python environment (one phrase), or None: an ImportError,
    or a package of requirements.txt that was imported without its __init__.py (an empty folder, or
    a broken installation)."""
    if isinstance(exc, ImportError):
        return f"{type(exc).__name__}: {exc}"
    for name in SANDBOX_PACKAGES:
        mod = sys.modules.get(name)
        if mod is not None and getattr(mod, "__file__", None) is None:
            return f"the package {name} has no __init__.py (an empty folder, or a broken installation)"
    return None


def environment_message(detail, who="the planner's process") -> str:
    """The message for a broken Python environment (no verdict)."""
    return (f"the Python environment is broken: {who} could not use the kit's packages ({detail}). This is no "
            "verdict on the planner or the referee. The planner runs in an isolated Python (python -I), which "
            "ignores PYTHONPATH and the user site-packages: install numpy, scipy, shapely, networkx, and matplotlib "
            f"into the interpreter itself, {sys.executable} -m pip install -r requirements.txt, preferably in a "
            "fresh virtual environment (README.md, Setup)")


# =============================================================================
# The planner's process (python -I -B check_ce.py --child PLANNER IN OUT TIMEOUT)
# =============================================================================
def _write_state(out_path: str, text: str) -> None:
    """Write text to out_path through out_path.tmp. On Windows the move can fail while
    another process (a virus scanner, an indexer) holds out_path open: it is retried,
    and then out_path is written in place. The reader takes the larger "seq" of the two."""
    tmp = out_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
    for k in range(10):
        try:
            os.replace(tmp, out_path)
            return
        except OSError:
            time.sleep(0.05 * (k + 1))
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(text)


def _short_tb(limit: int = 6) -> str:
    lines = traceback.format_exc(limit=limit).strip().splitlines()
    return "\n".join(lines[-limit:])


def _to_point(p):
    if hasattr(p, "coords"):                 # a shapely Point
        p = list(p.coords)[0]
    p = list(p)
    if len(p) != 2:
        raise TypeError(f"a point has {len(p)} coordinates")
    return [float(p[0]), float(p[1])]


def _convert_answer(result):
    """(path, cost) as returned by plan() -> ([[x, y], ...] or None, float); raises only ValueError."""
    try:
        path, cost = result
    except Exception:
        raise ValueError(f"plan returned {type(result).__name__}, not a pair (path, cost)") from None
    try:
        cost = float(cost)
    except Exception:
        raise ValueError(f"the cost {cost!r:.80} is not a number") from None
    if path is None:
        return None, cost
    try:
        if type(path).__name__ == "LineString":   # shapely
            path = list(path.coords)
        pts = [_to_point(p) for p in path]
    except Exception:
        raise ValueError("the path is neither None nor a list of (x, y) points") from None
    return pts, cost


def _child(planner_file: str, in_json: str, out_json: str, timeout: str, flag: str) -> int:
    state = {"stage": "start", "seq": 0}
    lock = threading.Lock()
    flags = {"over": False}

    def dump(**kw):                          # call with the lock held
        state.update(kw)
        state["seq"] += 1
        _write_state(out_json, json.dumps(state))

    with open(in_json, encoding="utf-8") as f:
        inst = json.load(f)
    robot = [tuple(p) for p in inst["robot"]]
    obstacles = [[tuple(p) for p in poly] for poly in inst["obstacles"]]
    start, goal = tuple(inst["start"]), tuple(inst["goal"])
    limit = float(timeout)
    with lock:
        dump(stage="loading")

    sys.dont_write_bytecode = True
    name = Path(planner_file).stem
    try:
        spec = importlib.util.spec_from_file_location(name, planner_file)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        plan = getattr(module, "plan")
        if not callable(plan):
            raise TypeError("plan is not a function")
    except BaseException as exc:
        with lock:
            dump(stage="load_failed", error=_short_tb(), env_error=env_trouble(exc))
        return 0

    clock = {"t0": time.perf_counter()}

    def watchdog():
        with lock:
            if flags["over"]:
                return
            flags["over"] = True
            dump(stage="timeout", plan_s=time.perf_counter() - clock["t0"])
        os._exit(0)

    with open(flag, "w", encoding="utf-8"):   # tells the parent that the limit starts now
        pass
    with lock:
        dump(stage="planning")
    timer = threading.Timer(limit, watchdog)
    timer.daemon = True
    clock["t0"] = t0 = time.perf_counter()
    timer.start()
    env_error = None
    try:
        result = plan(robot, obstacles, start, goal)
        failure = None
    except BaseException as exc:             # SystemExit and KeyboardInterrupt included
        result, failure = None, _short_tb()
        env_error = env_trouble(exc)
    elapsed = time.perf_counter() - t0
    with lock:
        if flags["over"]:                    # the watchdog has fired and is ending the process
            time.sleep(60)
        flags["over"] = True
    timer.cancel()
    with lock:
        if failure is not None:
            dump(stage="crashed", plan_s=elapsed, error=failure, env_error=env_error)
            return 0
        try:
            path, cost = _convert_answer(result)
        except ValueError as exc:
            dump(stage="bad_return", plan_s=elapsed, error=str(exc))
            return 0
        dump(stage="done", plan_s=elapsed, path=path, cost=cost)
    return 0


# =============================================================================
# Running a planner (parent side)
# =============================================================================
def _kill_tree(proc) -> None:
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True, timeout=30)
        else:
            os.killpg(proc.pid, 9)
    except Exception:
        pass
    try:
        proc.kill()
    except Exception:
        pass


def _read_state(out_path: Path):
    """The runner's last state: out_path or out_path.tmp, whichever has the larger seq."""
    best = None
    for f in (out_path, out_path.with_name(out_path.name + ".tmp")):
        try:
            st = json.loads(f.read_text(encoding="utf-8"))
        except Exception:
            continue
        if isinstance(st, dict) and (best is None or int(st.get("seq", 0)) > int(best.get("seq", 0))):
            best = st
    return best


def _tail(path: Path, n: int = 1500) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")[-n:]
    except Exception:
        return ""


def _last_line(text) -> str:
    lines = [ln for ln in str(text or "").strip().splitlines() if ln.strip()]
    return lines[-1].strip() if lines else ""


def _run_once(planner_file: Path, payload: dict, timeout: float, startup: float) -> dict:
    work = Path(tempfile.mkdtemp(prefix="hw1ce_"))
    try:
        in_json, out_json = work / "instance.json", work / "answer.json"
        in_json.write_text(json.dumps(payload), encoding="utf-8")
        flag = work / "planning.flag"
        env = {k: v for k, v in os.environ.items() if not k.upper().startswith("PYTHON")}
        env.update(MPLBACKEND="Agg")
        kw = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
        cmd = [sys.executable, "-I", "-B", str(Path(__file__).resolve()), "--child", str(planner_file),
               str(in_json), str(out_json), repr(float(timeout)), str(flag)]
        killed = None    # None, or the phase in which the process was stopped: "loading" or "planning"
        t0 = time.perf_counter()
        with open(work / "stdout.txt", "wb") as fo, open(work / "stderr.txt", "wb") as fe:
            proc = subprocess.Popen(cmd, cwd=str(work), stdout=fo, stderr=fe, stdin=subprocess.DEVNULL,
                                    env=env, **kw)
            try:
                plan_deadline = None
                while True:
                    try:
                        proc.wait(timeout=0.1)
                        break
                    except subprocess.TimeoutExpired:
                        pass
                    now = time.perf_counter()
                    if plan_deadline is None and flag.exists():
                        plan_deadline = now + timeout + PLAN_GRACE_S
                    if plan_deadline is not None and now > plan_deadline:
                        killed = "planning"
                    elif plan_deadline is None and now > t0 + startup:
                        killed = "loading"
                    if killed:
                        _kill_tree(proc)
                        try:
                            proc.wait(timeout=30)
                        except Exception:
                            pass
                        break
            except BaseException:            # Ctrl-C in the parent: do not leave the planner running
                _kill_tree(proc)
                raise
        state = _read_state(out_json) or {"stage": "no_output"}
        state.update(killed=killed, returncode=proc.returncode, wall_s=time.perf_counter() - t0,
                     stderr_tail=_tail(work / "stderr.txt"))
        return state
    finally:
        shutil.rmtree(work, ignore_errors=True)


def run_planner(planner_file, robot, obstacles, start, goal, timeout: float = TIMEOUT_S,
                startup: float = STARTUP_S) -> dict:
    """Run plan(robot, obstacles, start, goal) of planner_file in its own process.

    The process gets the robot and the instance only. Returns a dict with
      status   "done", "timeout", "crash" (an exception, an unusable return value, or a
               process that ended during the call), or "error" (the planner could not
               be loaded or run; not a verdict on the planner);
      path     list of (x, y) or None, and cost (float), when status is "done";
      plan_s   seconds spent in plan(), when known;
      message  what went wrong, for every status except "done".
    """
    planner_file = Path(planner_file).resolve()   # the planner's process runs in a temporary folder
    if not planner_file.is_file():
        return {"status": "error", "path": None, "cost": None, "plan_s": None,
                "message": f"planner file {planner_file} not found"}
    payload = {"robot": [[float(x), float(y)] for x, y in robot],
               "obstacles": [[[float(x), float(y)] for x, y in poly] for poly in obstacles],
               "start": [float(start[0]), float(start[1])], "goal": [float(goal[0]), float(goal[1])]}
    st = _run_once(planner_file, payload, timeout, startup)
    if st.get("returncode") == RUNNER_FAILED and not st.get("killed"):
        st = _run_once(planner_file, payload, timeout, startup)   # a failure of the runner: retry once
    stage, plan_s = st.get("stage"), st.get("plan_s")
    out = {"status": None, "path": None, "cost": None, "plan_s": plan_s, "message": None}
    limit = f"{timeout:g} s"
    tail = (st.get("stderr_tail") or "").strip()
    if stage in ("load_failed", "crashed") and st.get("env_error"):
        out.update(status="error", environment_broken=True,
                   message=environment_message(st["env_error"]) + ":\n" + (st.get("error") or "").strip())
    elif (stage not in ("timeout", "done", "crashed", "bad_return", "planning", "load_failed")
          and not st.get("killed") and re.search(r"\b(ModuleNotFoundError|ImportError)\b", tail)):
        out.update(status="error", environment_broken=True,
                   message=environment_message(_last_line(tail), "the Python that runs the planner") + ":\n" + tail)
    elif stage == "timeout" or (stage == "done" and plan_s is not None and plan_s > timeout):
        out.update(status="timeout", message=f"plan() did not return within {limit}")
    elif stage == "done":
        path = st.get("path")
        out.update(status="done", path=None if path is None else [(float(x), float(y)) for x, y in path],
                   cost=float(st.get("cost")))
    elif st.get("killed") == "planning":
        out.update(status="timeout", message=f"plan() did not return within {limit} (stopped)")
    elif st.get("killed") == "loading":
        out.update(status="error", message=f"the planner's process did not reach plan() within {startup:g} s "
                                           "(starting Python and importing the planner; the computer may be "
                                           "busy): try again")
    elif stage == "crashed":
        out.update(status="crash", message="plan() raised an exception:\n" + (st.get("error") or "").strip())
    elif stage == "bad_return":
        out.update(status="crash", message="plan() returned an unusable value: " + (st.get("error") or ""))
    elif stage == "planning":
        out.update(status="crash", message=f"the planner's process ended during plan() without returning "
                                           f"(exit code {st.get('returncode')})")
    elif stage == "load_failed":
        out.update(status="error", message=f"could not load {planner_file.name}:\n" + (st.get("error") or "").strip())
    else:
        out.update(status="error", message=f"the planner's process stopped at stage {stage!r} "
                                           f"(exit code {st.get('returncode')})" + (f":\n{tail}" if tail else ""))
    return out


# =============================================================================
# The check
# =============================================================================
def _kit():
    """The kit modules (parent process only). Raises EnvironmentBroken if they cannot be imported."""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    try:
        import gen_hw1
        import hw1_io
        from referee import geom_ref, referee
    except ImportError as exc:
        raise EnvironmentBroken(environment_message(env_trouble(exc), "this Python")) from None
    return gen_hw1, hw1_io, geom_ref, referee


def planner_for(path, planner=None) -> str:
    """The planner a file is checked against: --planner, else the file name (P<k>.json, P<k>_try.json)."""
    if planner:
        m = re.fullmatch(r"[Pp]([1-8])", str(planner).strip())
        if not m:
            raise CheckError(f"--planner must be one of P1 ... P8, not {planner!r}")
        return f"P{m.group(1)}"
    m = NAME_RE.match(Path(path).stem)
    if not m:
        raise CheckError(f"the file name {Path(path).name!r} does not name a planner: call it P1.json ... P8.json "
                         "(a draft may be P<k>_try.json), or pass --planner P<k>")
    return f"P{m.group(1)}"


def file_sha256(data: bytes) -> str:
    """SHA-256 of a kit file as MANIFEST.json records it (CRLF line ends count as LF)."""
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


def load_manifest(kit=None):
    """{path in the kit: SHA-256} of the original DO-NOT-EDIT files, from MANIFEST.json of the
    kit folder (default: this kit), or None if it is missing or unreadable."""
    try:
        files = json.loads((Path(kit or HERE) / MANIFEST).read_text(encoding="utf-8"))["files"]
    except Exception:
        return None
    if not isinstance(files, dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in files.items()):
        return None
    return files


def edited_kit_files(kit=None, planners_dir=None):
    """The DO-NOT-EDIT files of the kit that differ from MANIFEST.json (a missing file counts as
    different), or None if there is no usable MANIFEST.json. With planners_dir, the planners
    are those of that folder, as check_ce.py --planners-dir runs them."""
    kit = Path(kit or HERE)
    files = load_manifest(kit)
    if files is None:
        return None
    edited = []
    for rel, digest in sorted(files.items()):
        f = kit / rel
        if planners_dir is not None and rel.startswith("planners/"):
            f = Path(planners_dir) / rel.split("/", 1)[1]
        try:
            same = file_sha256(f.read_bytes()) == digest
        except OSError:
            same = False
        if not same:
            edited.append(rel)
    return edited


def kit_warning(kit=None, planners_dir=None):
    """One warning line if a DO-NOT-EDIT file differs from MANIFEST.json (or MANIFEST.json is
    missing), else None. The tools still run; the grader uses the original files."""
    edited = edited_kit_files(kit, planners_dir)
    if edited is None:
        return (f"WARNING: {MANIFEST} is missing or unreadable, so the kit cannot tell whether its DO-NOT-EDIT "
                "files are the originals (copy it from the kit zip).")
    if not edited:
        return None
    names = ", ".join(edited) if len(edited) <= 4 else ", ".join(edited[:3]) + f", and {len(edited) - 3} more"
    return (f"WARNING: {names} {'differs' if len(edited) == 1 else 'differ'} from the original kit ({MANIFEST}). "
            "DO-NOT-EDIT files are replaced by the originals when your submission is graded, so this result may "
            "differ from your grade: restore the original files from the kit zip.")


def instance_digest(data: bytes) -> str:
    """Fingerprint of an instance file: sha256 of its coordinates as floats snapped to the 0.001
    grid (hw1_io.parse_instance), so it does not change with spacing, line ends, or 1 vs 1.0. A
    file that cannot be parsed is hashed as text."""
    _, hw1_io, _, _ = _kit()
    try:
        inst = hw1_io.parse_instance(json.loads(data.decode("utf-8-sig")))
        canon = json.dumps({"obstacles": [[[x, y] for x, y in poly] for poly in inst["obstacles"]],
                            "start": list(inst["start"]), "goal": list(inst["goal"])}, separators=(",", ":"))
        return hashlib.sha256(canon.encode("ascii")).hexdigest()
    except Exception:
        return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


_SAVE_AS_UTF8 = ("save it as UTF-8 (in Windows PowerShell 5.1, > and Out-File write UTF-16: use "
                 "Out-File -Encoding utf8, or save the file as UTF-8 in your editor)")


def encoding_problem(data: bytes):
    """What is wrong with the encoding of an instance file (one sentence), or None if it is UTF-8
    text (a UTF-8 byte-order mark is fine)."""
    boms = ((b"\xff\xfe\x00\x00", "UTF-32 (little-endian)"), (b"\x00\x00\xfe\xff", "UTF-32 (big-endian)"),
            (b"\xff\xfe", "UTF-16 (little-endian)"), (b"\xfe\xff", "UTF-16 (big-endian)"))
    found = next((name for bom, name in boms if data.startswith(bom)), None)
    if found is None and b"\x00" in data:
        even, odd = data[0::2], data[1::2]
        if odd.count(0) > len(odd) // 3 and even.count(0) <= len(even) // 10:
            found = "UTF-16 (little-endian, without a byte-order mark)"
        elif even.count(0) > len(even) // 3 and odd.count(0) <= len(odd) // 10:
            found = "UTF-16 (big-endian, without a byte-order mark)"
        else:
            return "the file contains zero bytes, so it is not a text file in UTF-8: " + _SAVE_AS_UTF8
    if found:
        return f"the file is {found} text, not UTF-8: {_SAVE_AS_UTF8}"
    try:
        data.decode("utf-8")
    except UnicodeDecodeError as exc:
        return (f"the file is not UTF-8 text (byte 0x{data[exc.start]:02x} at position {exc.start}; it may be in a "
                f"Windows code page such as 1252 or 1255): {_SAVE_AS_UTF8}")
    return None


def read_instance_bytes(data: bytes):
    """(raw JSON object or None, instance with float tuples snapped to the grid or None, problems)
    for a file's bytes."""
    _, hw1_io, _, _ = _kit()
    bad_text = encoding_problem(data)
    if bad_text:
        return None, None, [bad_text]
    try:
        raw = json.loads(data.decode("utf-8-sig"))
    except UnicodeDecodeError:   # not reached: encoding_problem has caught it
        return None, None, ["the file is not UTF-8 text: " + _SAVE_AS_UTF8]
    except ValueError as exc:
        return None, None, [f"not valid JSON: {exc}"]
    try:
        problems = list(hw1_io.validate_instance(raw))
    except Exception as exc:  # validate_instance should not raise; be safe with odd input
        problems = [f"cannot be checked: {type(exc).__name__}: {exc}"]
    try:
        inst = hw1_io.parse_instance(raw)
    except Exception as exc:
        return raw, None, problems or [f"cannot be read as an instance: {exc}"]
    return raw, inst, problems


def path_length(path) -> float:
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])))


def _num(v):
    """A float for strict JSON: finite numbers as they are, others as "inf", "-inf", "nan"."""
    if v is None:
        return None
    v = float(v)
    if math.isfinite(v):
        return v
    return "nan" if math.isnan(v) else ("inf" if v > 0 else "-inf")


def _answer(path, cost):
    return {"path": None if path is None else [[_num(x), _num(y)] for x, y in path], "cost": _num(cost)}


def fmt_pt(p) -> str:
    return "(" + ", ".join(f"{float(v):.6g}" for v in p) + ")"


def fmt_cost(c) -> str:
    c = float(c)
    return f"{c:.6f}" if math.isfinite(c) else str(c)


def _why_no_path(start, goal, cobs, geom_ref) -> str:
    for what, p in (("start", start), ("goal", goal)):
        for j, c in enumerate(cobs):
            if geom_ref.point_in_interior(p, c):
                return f"the {what} lies in the interior of the C-obstacle of obstacle {j}"
    return "the start and the goal are in different parts of the free space"


def _why_invalid(name, path, start, goal, cobs, geom_ref) -> str:
    if not path or any(not (math.isfinite(x) and math.isfinite(y)) for x, y in path):
        return f"{name}'s path has a coordinate that is not a finite number"
    if math.dist(path[0], start) > ENDPOINT_TOL:
        return f"{name}'s path starts at {fmt_pt(path[0])}, not at the start {fmt_pt(start)}"
    if math.dist(path[-1], goal) > ENDPOINT_TOL:
        return f"{name}'s path ends at {fmt_pt(path[-1])}, not at the goal {fmt_pt(goal)}"
    segs = list(zip(path[:-1], path[1:])) or [(path[0], path[0])]
    for i, (a, b) in enumerate(segs, 1):
        for j, c in enumerate(cobs):
            if not geom_ref.segment_valid(a, b, [c]):
                return (f"segment {i} of {name}'s path, from {fmt_pt(a)} to {fmt_pt(b)}, has points in the "
                        f"interior of the C-obstacle of obstacle {j}")
    return f"{name}'s path is invalid under the referee"


def classify(ans: dict, ref: dict, robot, obstacles, start, goal):
    """(kind or None, explanation) for a planner answer (from run_planner) and the referee's answer."""
    _, _, geom_ref, referee = _kit()
    name = ans.get("name", "the planner")
    if ans["status"] == "timeout":
        return "timeout", f"{name}: {ans['message']}"
    if ans["status"] == "crash":
        return "crash", f"{name}: {ans['message']}"
    path, cost = ans["path"], ans["cost"]
    has_path = bool(path)
    cobs = ref["c_obstacles"]
    if ref["path_exists"] and not has_path:
        return "missed_path", f"{name} returns no path, but a path of cost {fmt_cost(ref['cost'])} exists"
    if not ref["path_exists"] and has_path:
        return "false_path", (f"{name} returns a path, but no path exists: "
                              + _why_no_path(start, goal, cobs, geom_ref))
    if not has_path:
        if not (math.isinf(cost) and cost > 0):
            return "wrong_cost", f"{name} returns no path with cost {fmt_cost(cost)}; with no path the cost must be inf"
        return None, f"{name} and the referee both report that no path exists"
    if not referee.path_valid(robot, obstacles, path, start, goal):
        return "invalid_path", _why_invalid(name, path, start, goal, cobs, geom_ref)
    length = path_length(path)
    if length > ref["cost"] + COST_TOL:
        return "suboptimal", (f"{name}'s path has length {fmt_cost(length)}, longer than the optimum "
                              f"{fmt_cost(ref['cost'])} by {length - ref['cost']:.3g}")
    if not abs(cost - length) <= COST_TOL:
        return "wrong_cost", f"{name} reports cost {fmt_cost(cost)}, but its path has length {fmt_cost(length)}"
    if length < ref["cost"] - COST_TOL:   # cannot happen with a correct referee; say so if it does
        return None, (f"{name}'s valid path is shorter than the referee's ({fmt_cost(length)} < "
                      f"{fmt_cost(ref['cost'])}); please report this instance to the course staff")
    return None, f"{name}'s path is valid and optimal (cost {fmt_cost(cost)})"


def check(path, ids, planner=None, planners_dir=None, timeout: float = TIMEOUT_S, startup: float = STARTUP_S,
          draw=None) -> dict:
    """Check one counterexample file; returns the record that --json prints:
      kit_version, ids, robot, planner, file, digest, valid_instance, problems, size,
      verdict ("counterexample", "not_a_counterexample", or None if the instance is
      invalid or the check could not be completed), kind, detail, planner_answer and
      referee_answer ({"path": [[x, y], ...] or None, "cost": number or "inf"}; None
      when there is no answer), planner_time_s, and error (why the check could not
      be completed, else None)."""
    gen_hw1, hw1_io, geom_ref, referee = _kit()
    ids = gen_hw1.normalize_ids(ids)
    name = planner_for(path, planner)
    robot = gen_hw1.robot_for_ids(ids)
    rec = {"kit_version": KIT_VERSION, "ids": ids, "robot": [[x, y] for x, y in robot], "planner": name,
           "file": str(path), "digest": None, "valid_instance": False, "problems": [], "size": None,
           "verdict": None, "kind": None, "detail": None, "planner_answer": None, "referee_answer": None,
           "planner_time_s": None, "error": None}
    try:
        data = Path(path).read_bytes()
    except OSError as exc:
        rec["error"] = f"cannot read {path}: {exc.strerror or exc}"
        return rec
    rec["digest"] = instance_digest(data)
    raw, inst, problems = read_instance_bytes(data)
    if isinstance(raw, dict) and isinstance(raw.get("obstacles"), list):
        rec["size"] = len(raw["obstacles"])
    rec["problems"] = problems
    rec["valid_instance"] = not problems and inst is not None
    if not rec["valid_instance"]:
        rec["detail"] = "the instance breaks the rules; the planner was not run"
        return rec
    obstacles, start, goal = inst["obstacles"], inst["start"], inst["goal"]

    pdir = Path(planners_dir) if planners_dir else HERE / "planners"
    ans = run_planner(pdir / f"{name}.py", robot, obstacles, start, goal, timeout, startup)
    ans["name"] = name
    rec["planner_time_s"] = None if ans.get("plan_s") is None else round(float(ans["plan_s"]), 4)
    if ans["status"] == "error":
        rec["error"] = f"{name}: {ans['message']}"
        rec["detail"] = "the planner could not be run; this is not a verdict on the planner"
        return rec
    if ans["status"] == "done":
        rec["planner_answer"] = _answer(ans["path"], ans["cost"])

    try:
        ref = referee.solve(robot, obstacles, start, goal)
    except Exception as exc:
        trouble = env_trouble(exc)
        if trouble:
            rec["error"] = environment_message(trouble, "the referee") + ":\n" + traceback.format_exc(limit=3)
        else:
            rec["error"] = ("the referee failed on this instance; please report it to the course staff:\n"
                            + traceback.format_exc(limit=3))
        return rec
    rec["referee_answer"] = _answer(ref["path"], ref["cost"])
    kind, detail = classify(ans, ref, robot, obstacles, start, goal)
    rec["verdict"] = "counterexample" if kind else "not_a_counterexample"
    rec["kind"], rec["detail"] = kind, detail

    if draw:
        try:
            import draw as draw_mod
            draw_mod.draw_instance(draw, robot, obstacles, start, goal, c_obstacles=ref["c_obstacles"],
                                   referee_answer=ref, planner=name, planner_answer=ans)
            rec["figure"] = str(draw)
        except Exception as exc:
            rec["figure_error"] = f"{type(exc).__name__}: {exc}"
    return rec


# =============================================================================
# Text report
# =============================================================================
def _fmt_path(path, max_points=10) -> str:
    pts = [fmt_pt(p) for p in path]
    if len(pts) > max_points:
        pts = pts[: max_points - 2] + ["...", pts[-1]]
    return " -> ".join(pts)


def _fmt_answer(ans) -> str:
    if ans is None:
        return "no answer"
    path = ans["path"]
    cost = float(ans["cost"])
    if not path:
        return f"no path (cost {fmt_cost(cost)})"
    pts = [(float(x), float(y)) for x, y in path]
    return (f"cost {fmt_cost(cost)}; path of {len(pts)} points, length {fmt_cost(path_length(pts))}: "
            + _fmt_path(pts))


def report_text(rec) -> str:
    L = [f"check_ce.py {rec['planner']} on {rec['file']}",
         f"  robot of {' '.join(rec['ids'])}: " + " ".join(fmt_pt(p) for p in rec["robot"])]
    if rec["size"] is not None:
        L.append(f"  size: {rec['size']} (number of obstacles)")
    if not rec["valid_instance"] and rec["problems"]:
        if rec["size"] is None:   # not even a JSON object with a list of obstacles
            L.append("INVALID INSTANCE: the file cannot be read as an instance; the planner was not run.")
        else:
            L.append(f"INVALID INSTANCE: {len(rec['problems'])} rule(s) broken; the planner was not run.")
        L += [f"  - {p}" for p in rec["problems"]]
        return "\n".join(L)
    planner_label = f"{rec['planner']} answer:"
    if rec["error"]:
        if rec["planner_answer"] is not None:
            L.append(f"  {planner_label:<16} {_fmt_answer(rec['planner_answer'])}")
        L.append("CHECK NOT COMPLETED: " + rec["error"])
        return "\n".join(L)
    t = rec["planner_time_s"]
    L.append(f"  {planner_label:<16} {_fmt_answer(rec['planner_answer'])}"
             + (f"   [plan() took {t:.2f} s]" if t is not None else ""))
    L.append(f"  {'referee answer:':<16} {_fmt_answer(rec['referee_answer'])}")
    if rec["verdict"] == "counterexample":
        L.append(f"COUNTEREXAMPLE ({rec['kind']}): {rec['detail']}")
    else:
        L.append(f"NOT A COUNTEREXAMPLE: {rec['detail']}")
    if rec.get("figure"):
        L.append(f"Figure written to {rec['figure']}")
    if rec.get("figure_error"):
        L.append(f"Figure not drawn: {rec['figure_error']}")
    return "\n".join(L)


def exit_code(rec) -> int:
    return {"counterexample": 0, "not_a_counterexample": 1}.get(rec.get("verdict"), 2)


def split_ids(ids, instance):
    """--ids takes one or more values, so in `--ids ID1 ID2 counterexamples/P<k>.json` argparse
    reads the file as a third ID. Move a trailing value that names a file back to `instance`."""
    ids = list(ids)
    if instance is None and len(ids) >= 2:
        last = ids[-1]
        if last.lower().endswith(".json") or "/" in last or "\\" in last or Path(last).is_file():
            instance = ids.pop()
    return ids, instance


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description="Check a counterexample against one planner (236767 HW1).",
                                 epilog="Exit code: 0 COUNTEREXAMPLE, 1 NOT A COUNTEREXAMPLE, 2 invalid instance "
                                        "or check not completed.")
    ap.add_argument("--ids", nargs="+", required=True, metavar="ID", help="one or two student IDs")
    ap.add_argument("instance", nargs="?", help="the counterexample file, e.g. counterexamples/P<k>.json")
    ap.add_argument("--planner", help="P1 ... P8 (default: from the file name)")
    ap.add_argument("--planners-dir", help="folder with P1.py ... P8.py (default: planners/ next to this file)")
    ap.add_argument("--draw", metavar="FILE", help="also draw the instance and both paths to FILE (.pdf or .png)")
    ap.add_argument("--json", action="store_true", help="print one JSON object instead of the text report")
    ap.add_argument("--timeout", type=float, default=TIMEOUT_S, help=argparse.SUPPRESS)
    ap.add_argument("--startup-allowance", type=float, default=STARTUP_S, help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    a.ids, a.instance = split_ids(a.ids, a.instance)
    if a.instance is None:
        ap.error("give the counterexample file, e.g. counterexamples/P<k>.json")
    if len(a.ids) > 2:
        ap.error("pass one or two IDs")
    if a.draw and Path(a.draw).suffix.lower() not in (".pdf", ".png"):
        ap.error("--draw must end in .pdf or .png")
    warning = kit_warning(planners_dir=a.planners_dir)
    if warning:   # stdout stays pure JSON with --json
        print(warning, file=sys.stderr if a.json else sys.stdout, flush=True)
    try:
        rec = check(a.instance, a.ids, planner=a.planner, planners_dir=a.planners_dir, timeout=a.timeout,
                    startup=a.startup_allowance, draw=a.draw)
    except (CheckError, ValueError) as exc:
        if a.json:
            print(json.dumps({"kit_version": KIT_VERSION, "file": a.instance, "verdict": None,
                              "error": str(exc)}, indent=1))
        else:
            print(f"check_ce.py: {exc}", file=sys.stderr)
        return 2
    if a.json:
        print(json.dumps(rec, indent=1, allow_nan=False))
    else:
        print(report_text(rec))
    return exit_code(rec)


if __name__ == "__main__":
    if sys.argv[1:2] == ["--child"]:
        code = 0
        try:
            code = _child(*sys.argv[2:7])
        except BaseException:               # a failure of the runner, not of the planner
            code = RUNNER_FAILED
            try:
                traceback.print_exc()
            except Exception:
                pass
        finally:
            # End the planner's process at once: threads or atexit hooks left behind by
            # the planner must not keep a finished run alive until the time limit.
            for stream in (sys.stdout, sys.stderr):
                try:
                    stream.flush()
                except Exception:
                    pass
            os._exit(code)
    sys.exit(main())
