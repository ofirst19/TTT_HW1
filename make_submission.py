# =============================================================================
# Kit file (236767 HW1, kit version 2.4).  Builds your submission zip.
# =============================================================================
"""Build HW1_<ID1>_<ID2>.zip (or HW1_<ID>.zip) and run check_submission.py on it.

  python make_submission.py --ids ID1 [ID2] [--report report.pdf] [--results results.json]
                            [--answers answers.txt] [--counterexamples counterexamples] [--out .]

Run it from the folder where you ran python HW1.py --ids ... The zip gets, at its root:
  report.pdf                 from --report; without --report, ./report.pdf, or else the PDF
                             of the template, ./report_template.pdf (then the same two files
                             next to this file), with a note saying which file it used
  results.json               from --results (default ./results.json), as HW1.py wrote it
  answers.txt                from --answers; without it, ./answers.txt, or else the one next
                             to this file (the kit ships a skeleton there: fill it in)
  counterexamples/P<k>.json  the files P1.json ... P8.json of --counterexamples (default
                             ./counterexamples, or the one next to this file, as in HW1.py)
  code/                      every .py and .ipynb file of this kit folder and its subfolders
                             (the kit, planners/, referee/, and your own scripts), without
                             virtual environments, caches, and the folders counterexamples/,
                             example/, figs/, plots/, code/, and submission/; the run lists
                             the scripts it leaves out of those folders
Paths in the zip use forward slashes, so the zip reads the same on every system. Exits
with the exit code of check_submission.py (0 only if the zip passes; a zip that fails
the check is not graded).
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import gen_hw1  # noqa: E402

TOOL_DIRS = {"__pycache__", ".git", ".venv", "venv", "env", ".env", "site-packages", ".ipynb_checkpoints"}
OTHER_DIRS = {"counterexamples", "example", "figs", "plots", "code", "submission"}   # not code; listed when skipped
SKIP_DIRS = TOOL_DIRS | OTHER_DIRS
CODE_SUFFIXES = (".py", ".ipynb")
REPORT_NAMES = ("report.pdf", "report_template.pdf")


def code_files(kit: Path):
    """(files for code/ as (path, name in the zip), code files left out of OTHER_DIRS as names)."""
    out, left_out = [], []
    for f in sorted(kit.rglob("*")):
        if not f.is_file() or f.suffix.lower() not in CODE_SUFFIXES:
            continue
        rel = f.relative_to(kit)
        dirs = rel.parts[:-1]
        if any(part in TOOL_DIRS or part.startswith(".") for part in dirs):
            continue
        if any(part in OTHER_DIRS for part in dirs):
            left_out.append(rel.as_posix())
            continue
        out.append((f, "code/" + rel.as_posix()))
    return out, left_out


def counterexample_files(cdir: Path):
    """(files named exactly P<k>.json, other file names) in cdir."""
    if not cdir.is_dir():
        return [], []
    keep, other = [], []
    for name in sorted(os.listdir(cdir)):
        if (cdir / name).is_file():
            (keep if re.fullmatch(r"P[1-8]\.json", name) else other).append(name)
    return [cdir / n for n in keep], other


def find_report(given):
    """(the PDF to submit as report.pdf or None, the places looked at)."""
    if given:
        return (Path(given) if Path(given).is_file() else None), [Path(given)]
    places = [Path(n) for n in REPORT_NAMES]
    if Path.cwd().resolve() != HERE:
        places += [HERE / n for n in REPORT_NAMES]
    return next((p for p in places if p.is_file()), None), places


def find_file(given, name):
    """(the file to submit as `name` or None, the places looked at): --option, else ./name, else
    the one next to this file."""
    if given:
        return (Path(given) if Path(given).is_file() else None), [Path(given)]
    places = [Path(name)]
    if Path.cwd().resolve() != HERE:
        places.append(HERE / name)
    return next((p for p in places if p.is_file()), None), places


def main(argv=None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", nargs="+", required=True, metavar="ID", help="one or two student IDs")
    ap.add_argument("--report", help="the report PDF (default: report.pdf, else report_template.pdf)")
    ap.add_argument("--results", default="results.json")
    ap.add_argument("--answers", help="answers.txt (default: ./answers.txt, else the one next to this file)")
    ap.add_argument("--counterexamples", help="folder of P1.json ... P8.json (default: counterexamples, as in HW1.py)")
    ap.add_argument("--out", default=".", help="folder for the zip (default: the current folder)")
    a = ap.parse_args(argv)
    try:
        ids = gen_hw1.normalize_ids(a.ids)
    except ValueError as exc:
        ap.error(str(exc))
    results = Path(a.results)
    if a.counterexamples:
        cdir = Path(a.counterexamples)
    else:   # the same folder that HW1.py uses
        cdir = Path("counterexamples")
        if not cdir.is_dir() and (HERE / "counterexamples").is_dir():
            cdir = HERE / "counterexamples"
            print(f"Using {cdir} (there is no counterexamples/ in the current folder)")
    report, looked = find_report(a.report)
    missing = []
    if report is None:
        missing.append("report.pdf (looked for " + ", ".join(str(p.resolve()) for p in looked) + ")")
    elif not a.report and report != Path("report.pdf"):
        print(f"Using {report} as report.pdf (there is no report.pdf in the current folder)")
    if not results.is_file():
        missing.append(f"{results} (looked for {results.resolve()})")
    answers, looked_a = find_file(a.answers, "answers.txt")
    if answers is None:
        missing.append("answers.txt (looked for " + ", ".join(str(p.resolve()) for p in looked_a) + ")")
    elif answers.resolve() != Path("answers.txt").resolve() and not a.answers:
        print(f"Using {answers} as answers.txt (there is no answers.txt in the current folder)")
    if missing:
        print("Missing: " + "; ".join(missing))
        if not results.is_file():
            print("Write results.json first: python HW1.py --ids " + " ".join(ids))
        if report is None:
            print("Compile your report (report_template.tex) to report.pdf, or pass --report FILE")
        return 1
    ces, other = counterexample_files(cdir)
    if other:
        print(f"Left out of the zip (only P1.json ... P8.json are submitted): {', '.join(other)}")
    newer = [p.name for p in ces if p.stat().st_mtime > results.stat().st_mtime]
    if newer:
        print(f"WARNING: {', '.join(newer)} changed after {results} was written; "
              "rerun python HW1.py --ids " + " ".join(ids) + " so that results.json matches them")
    zpath = Path(a.out) / ("HW1_" + "_".join(ids) + ".zip")
    zpath.parent.mkdir(parents=True, exist_ok=True)
    code, left_out = code_files(HERE)
    if left_out:
        print("Left out of code/ (scripts in the folders " + ", ".join(f"{d}/" for d in sorted(OTHER_DIRS))
              + " are not submitted; move a script you need into the kit folder): " + ", ".join(left_out))
    files = [(report, "report.pdf"), (results, "results.json"), (answers, "answers.txt")]
    files += [(p, f"counterexamples/{p.name}") for p in ces]
    files += code
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for src, arc in files:
            z.write(src, arc)
    print(f"Wrote {zpath} ({len(files)} files, {len(ces)} counterexample(s); the IDs in its name are in increasing "
          f"order); running: python check_submission.py {zpath}", flush=True)
    r = subprocess.run([sys.executable, str(HERE / "check_submission.py"), str(zpath)])
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
