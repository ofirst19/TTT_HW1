# =============================================================================
# Kit file (236767 HW1, kit version 2.4).  results.json must be written by this
# script and never edited by hand.
# =============================================================================
"""Check every counterexample of your pair and write results.json.

  python HW1.py --ids ID1 [ID2] [--counterexamples DIR] [--out results.json] [--figs DIR]

For each planner P<k>, k = 1, ..., 8:
  * if DIR/P<k>.json exists, you claim that P<k> is flawed, and check_ce.py checks the
    file against planners/P<k>.py (the planner runs in its own process, 30 s limit);
  * otherwise you claim that P<k> is correct.
DIR is counterexamples/ in the folder you run the command in (or next to this file).
Only the exact names P1.json ... P8.json count; drafts such as P<k>_try.json are ignored.

results.json holds the IDs, the kit version, your robot, one record per planner (the
claim and the check), and the errors of the run; README.md describes every field.
With --figs DIR the run also draws DIR/P<k>.pdf for every file it checks: the figure of
python draw.py --ids ... counterexamples/P<k>.json --planner P<k>.
The run warns when a DO-NOT-EDIT file of the kit, a planner included, differs from the
original recorded in MANIFEST.json: the grader reruns your files with the originals.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import check_ce  # noqa: E402
import gen_hw1  # noqa: E402

KIT_VERSION = check_ce.KIT_VERSION
RECORD_KEYS = ("claimed", "file", "valid_instance", "problems", "verdict", "kind", "size",
               "planner_answer", "referee_answer", "detail", "digest")


def empty_record(claimed: str, file=None) -> dict:
    rec = dict.fromkeys(RECORD_KEYS)
    rec.update(claimed=claimed, file=file, problems=[])
    return rec


def find_files(cdir: Path):
    """({k: path} for the files named exactly P<k>.json, notes on other files, errors)."""
    files, ignored, errors = {}, [], []
    for name in sorted(os.listdir(cdir)):
        p = cdir / name
        if not p.is_file():
            continue
        m = re.fullmatch(r"P([1-8])\.json", name)
        if m:
            files[int(m.group(1))] = p
            continue
        m = re.fullmatch(r"p([1-8])\.json", name, flags=re.IGNORECASE)
        if m:
            errors.append(f"{cdir.name}/{name}: rename it to P{m.group(1)}.json (file names are case-sensitive); "
                          "it was not checked")
        else:
            ignored.append(name)
    return files, ignored, errors


def run(ids, cdir: Path, out: Path, planners_dir=None, timeout=check_ce.TIMEOUT_S, figs=None) -> dict:
    ids = gen_hw1.normalize_ids(ids)
    robot = gen_hw1.robot_for_ids(ids)
    results = {"ids": ids, "kit_version": KIT_VERSION, "robot": [[x, y] for x, y in robot], "planners": {},
               "errors": []}
    files, ignored = {}, []
    if cdir.is_dir():
        files, ignored, errs = find_files(cdir)
        results["errors"] += errs
    else:
        print(f"WARNING: folder {cdir} not found, so every planner is claimed correct")
    if ignored:
        print(f"Ignored in {cdir}: {', '.join(ignored)} (only P1.json ... P8.json count)")
    print(f"Robot of {' '.join(ids)}: " + " ".join(check_ce.fmt_pt(p) for p in robot))
    for k in range(1, 9):
        name = f"P{k}"
        if k not in files:
            results["planners"][name] = empty_record("correct")
            continue
        rec = empty_record("flawed", f"counterexamples/{name}.json")
        t0 = time.perf_counter()
        try:
            r = check_ce.check(files[k], ids, planner=name, planners_dir=planners_dir, timeout=timeout,
                               draw=(Path(figs) / f"{name}.pdf") if figs else None)
        except Exception as exc:  # never stop the run
            results["errors"].append(f"{name}: the check failed: {type(exc).__name__}: {exc}")
        else:
            for key in RECORD_KEYS[2:]:
                rec[key] = r[key]
            if r["error"]:
                results["errors"].append(f"{name}: {r['error']}")
            if r.get("figure_error"):
                results["errors"].append(f"{name}: figure not drawn: {r['figure_error']}")
        results["planners"][name] = rec
        print(f"  {name}: checked {files[k]} in {time.perf_counter() - t0:.1f} s")
    out.write_text(json.dumps(results, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    return results


def summary(results) -> str:
    L = [f"{'Planner':<8} {'Claimed':<8} {'Size':>4}  Verdict"]
    for name, rec in results["planners"].items():
        if rec["claimed"] == "correct":
            L.append(f"{name:<8} {'correct':<8} {'-':>4}  -")
            continue
        size = "-" if rec["size"] is None else str(rec["size"])
        if rec["valid_instance"] is False and rec["size"] is None:
            v = "INVALID INSTANCE (the file cannot be read as an instance; see results.json)"
        elif rec["valid_instance"] is False:
            v = f"INVALID INSTANCE ({len(rec['problems'])} rule(s) broken; see results.json)"
        elif rec["verdict"] == "counterexample":
            v = f"COUNTEREXAMPLE ({rec['kind']})"
        elif rec["verdict"] == "not_a_counterexample":
            v = "NOT A COUNTEREXAMPLE"
        else:
            v = "CHECK NOT COMPLETED (see errors)"
        L.append(f"{name:<8} {'flawed':<8} {size:>4}  {v}")
    invalid = [name for name, rec in results["planners"].items()
               if rec["claimed"] == "flawed" and rec["valid_instance"] is False]
    if invalid:
        L.append(f"WARNING: the counterexample file of {', '.join(invalid)} breaks the instance rules: "
                 "check_submission.py rejects the whole zip (it would not be graded) until you fix or remove "
                 + ("that file" if len(invalid) == 1 else "those files"))
    return "\n".join(L)


def main(argv=None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description="HW1: check every counterexample and write results.json (236767).")
    ap.add_argument("--ids", nargs="+", required=True, metavar="ID", help="one or two student IDs")
    ap.add_argument("--counterexamples", help="folder of P1.json ... P8.json (default: counterexamples)")
    ap.add_argument("--out", default="results.json", help="output file (default: results.json)")
    ap.add_argument("--figs", metavar="DIR", help="also draw DIR/P<k>.pdf for every checked file")
    ap.add_argument("--planners-dir", help="folder with P1.py ... P8.py (default: planners/ next to this file)")
    ap.add_argument("--timeout", type=float, default=check_ce.TIMEOUT_S, help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    try:
        gen_hw1.normalize_ids(a.ids)
    except ValueError as exc:
        ap.error(str(exc))
    if a.counterexamples:
        cdir = Path(a.counterexamples)
    else:
        cdir = Path("counterexamples")
        if not cdir.is_dir() and (HERE / "counterexamples").is_dir():
            cdir = HERE / "counterexamples"
            print(f"Using {cdir} (there is no counterexamples/ in the current folder)")
    warning = check_ce.kit_warning(planners_dir=a.planners_dir)
    if warning:
        print(warning, flush=True)
    t0 = time.perf_counter()
    results = run(a.ids, cdir, Path(a.out), a.planners_dir, a.timeout, a.figs)
    print(summary(results))
    n_flawed = sum(1 for r in results["planners"].values() if r["claimed"] == "flawed")
    print(f"Wrote {a.out} in {time.perf_counter() - t0:.1f} s: {n_flawed} planner(s) claimed flawed, "
          f"{8 - n_flawed} claimed correct; {len(results['errors'])} error(s).")
    for e in results["errors"]:
        print("  ERROR " + e.splitlines()[0])
    if warning:
        print("WARNING: results.json was written with kit files that differ from the original kit (see the "
              "first line); the grader reruns your counterexamples with the original files.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
