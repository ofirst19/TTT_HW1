# =============================================================================
# DO NOT EDIT. This kit file is replaced by the original when your submission
# is graded (236767 HW1, kit version 2.4).
# =============================================================================
"""Check an HW1 submission zip before you upload it.

  python check_submission.py HW1_<ID1>_<ID2>.zip     (or HW1_<ID>.zip alone)

The zip holds, at its root (not inside a folder):
  report.pdf
  results.json                 written by python HW1.py --ids ..., never edited by hand
  answers.txt                  one line "KEY: text" per key (principles, guarantee lines,
                               and T1.planner, the planner of your proof T1)
  counterexamples/P<k>.json    one file for each planner that results.json claims flawed
  code/                        this kit (with planners/ and referee/) and any scripts of yours

The check verifies the file name; the required files; the results.json schema; that
results.json belongs to the IDs in the file name and to their robot; that every
counterexample file follows the instance rules (hw1_io.validate_instance); that
results.json matches the counterexample files: the same planners, and the same
contents (rerun HW1.py after you change a file); and that answers.txt has every key
exactly once, each answer on one line of at most 400 characters, and T1.planner naming
a planner P1 ... P8, or - if you skip T1 (hw1_io.parse_answers; the skeleton's P? is a
problem). It prints every problem and exits with 0 only if there are none. Warnings
(for example, a check that results.json records as NOT A COUNTEREXAMPLE, a blank answer
for a planner claimed flawed, a principle or guarantee line for a planner without a
counterexample file, which scores 0, a T1.planner that is - or is not a planner you
claim flawed with a counterexample file of two or more obstacles, or a DO-NOT-EDIT file
in code/ that differs from the original kit, as recorded in MANIFEST.json) do not fail
the check. A zip that fails this check will not be checked: it is not graded and earns
0 points.
"""
from __future__ import annotations

import json
import math
import re
import sys
import zipfile
from pathlib import Path, PurePosixPath

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import check_ce  # noqa: E402
import gen_hw1  # noqa: E402
import hw1_io  # noqa: E402

KIT_VERSION = "2.4"
NAME_RE = re.compile(r"^HW1_(\d+)(?:_(\d+))?\.zip$")
PLANNERS = tuple(f"P{k}" for k in range(1, 9))
CE_FILES = {p: f"counterexamples/{p}.json" for p in PLANNERS}
REFEREE_FILES = ("referee/__init__.py", "referee/geom_ref.py", "referee/planner_ref.py", "referee/referee.py")
PLANNER_FILES = tuple(f"planners/{p}.py" for p in PLANNERS)
DO_NOT_EDIT = (("hw1_io.py", "gen_hw1.py", "check_ce.py", "draw.py", "check_submission.py")
               + REFEREE_FILES + PLANNER_FILES)
KIT_CODE_FILES = ("HW1.py",) + DO_NOT_EDIT
REQUIRED = ("report.pdf", "results.json", "answers.txt") + tuple(f"code/{f}" for f in KIT_CODE_FILES)
TOP_KEYS = ("ids", "kit_version", "robot", "planners", "errors")
RECORD_KEYS = ("claimed", "file", "valid_instance", "problems", "verdict", "kind", "size",
               "planner_answer", "referee_answer", "detail", "digest")
JUNK = re.compile(r"(^|/)(__MACOSX|\.DS_Store|Thumbs\.db|desktop\.ini)(/|$)", re.IGNORECASE)
SIZE_WARN_MB = 20


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _is_cost(v):
    """A cost in results.json: a finite number, or "inf", "-inf", "nan" (JSON has no infinity)."""
    return _is_num(v) or v in ("inf", "-inf", "nan")


def _is_point(p):
    return isinstance(p, list) and len(p) == 2 and all(_is_cost(c) for c in p)


def _norm_ids(ids):
    try:
        return gen_hw1.normalize_ids(ids)
    except Exception:
        return sorted(str(i) for i in ids)


class Checker:
    def __init__(self):
        self.problems: list[str] = []
        self.warnings: list[str] = []

    def bad(self, msg):
        self.problems.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)

    def keys(self, where, rec, keys):
        if not isinstance(rec, dict):
            self.bad(f"results.json: {where} should be an object, got {type(rec).__name__}")
            return False
        missing = [k for k in keys if k not in rec]
        extra = [k for k in rec if k not in keys]
        if missing:
            self.bad(f"results.json: {where} is missing the key(s) {', '.join(missing)}")
        if extra:
            self.bad(f"results.json: {where} has unexpected key(s) {', '.join(map(str, extra))}")
        return not missing


def _check_answer(ck, where, ans):
    if ans is None:
        return
    if not isinstance(ans, dict) or set(ans) != {"path", "cost"}:
        ck.bad(f"results.json: {where} should be null or an object with the keys path and cost")
        return
    path = ans["path"]
    if path is not None and not (isinstance(path, list) and all(_is_point(p) for p in path)):
        ck.bad(f"results.json: {where}.path should be null or a list of [x, y]")
    if not _is_cost(ans["cost"]):
        ck.bad(f"results.json: {where}.cost should be a number or \"inf\"")


def check_record(ck, name, rec):
    """Schema of one planner record; returns True if it can be compared with the files."""
    where = f"planners.{name}"
    if not ck.keys(where, rec, RECORD_KEYS):
        return False
    claimed = rec["claimed"]
    if claimed not in ("flawed", "correct"):
        ck.bad(f"results.json: {where}.claimed should be \"flawed\" or \"correct\", got {claimed!r}")
        return False
    if claimed == "correct":
        others = {k: rec[k] for k in RECORD_KEYS if k not in ("claimed", "problems")}
        if any(v is not None for v in others.values()) or rec["problems"] not in ([], None):
            ck.bad(f"results.json: {where} claims correct, so its other fields should be null")
        return True
    if rec["file"] != CE_FILES[name]:
        ck.bad(f"results.json: {where}.file should be {CE_FILES[name]!r}, got {rec['file']!r}")
    if not isinstance(rec["valid_instance"], bool):
        ck.bad(f"results.json: {where}.valid_instance should be true or false")
    if not (isinstance(rec["problems"], list) and all(isinstance(p, str) for p in rec["problems"])):
        ck.bad(f"results.json: {where}.problems should be a list of strings")
    if rec["verdict"] not in ("counterexample", "not_a_counterexample", None):
        ck.bad(f"results.json: {where}.verdict should be \"counterexample\", \"not_a_counterexample\", or null")
    if rec["verdict"] == "counterexample":
        if rec["kind"] not in check_ce.KINDS:
            ck.bad(f"results.json: {where}.kind should be one of {', '.join(check_ce.KINDS)}")
    elif rec["kind"] is not None:
        ck.bad(f"results.json: {where}.kind should be null unless the verdict is \"counterexample\"")
    size = rec["size"]
    if size is not None and not (isinstance(size, int) and not isinstance(size, bool) and size >= 0):
        ck.bad(f"results.json: {where}.size should be a non-negative integer or null")
    _check_answer(ck, f"{where}.planner_answer", rec["planner_answer"])
    _check_answer(ck, f"{where}.referee_answer", rec["referee_answer"])
    if rec["detail"] is not None and not isinstance(rec["detail"], str):
        ck.bad(f"results.json: {where}.detail should be a string or null")
    if not (isinstance(rec["digest"], str) and re.fullmatch(r"[0-9a-f]{64}", rec["digest"])):
        ck.bad(f"results.json: {where}.digest should be the 64-digit hex digest that HW1.py writes")
    return True


def check_results(ck, res, ids_from_name):
    """Schema of results.json; returns the per-planner records that can be compared with the files."""
    if not ck.keys("the top level", res, TOP_KEYS) and not isinstance(res, dict):
        return {}
    ids = res.get("ids")
    if not (isinstance(ids, list) and 1 <= len(ids) <= 2 and all(isinstance(i, str) and i for i in ids)):
        ck.bad("results.json: 'ids' should be a list of one or two ID strings")
        ids = None
    elif ids_from_name and _norm_ids(ids) != _norm_ids(ids_from_name):
        ck.bad(f"results.json: ids {ids} do not match the IDs in the file name {ids_from_name}")
    if res.get("kit_version") != KIT_VERSION:
        ck.bad(f"results.json: kit_version is {res.get('kit_version')!r}, expected {KIT_VERSION!r} "
               "(run this kit's HW1.py)")
    robot = res.get("robot")
    if ids:
        want = [[x, y] for x, y in gen_hw1.robot_for_ids(ids)]
        same = (isinstance(robot, list) and len(robot) == len(want)
                and all(isinstance(p, list) and len(p) == 2 and all(_is_num(c) for c in p)
                        and abs(p[0] - q[0]) < 1e-9 and abs(p[1] - q[1]) < 1e-9 for p, q in zip(robot, want)))
        if not same:
            ck.bad(f"results.json: robot {robot} is not the robot of {ids} ({want})")
    errs = res.get("errors")
    if not (isinstance(errs, list) and all(isinstance(e, str) for e in errs)):
        ck.bad("results.json: 'errors' should be a list of strings")
    elif errs:
        ck.warn(f"results.json records {len(errs)} error(s) in its 'errors' list, e.g. {errs[0].splitlines()[0]}")
    planners = res.get("planners")
    if not isinstance(planners, dict):
        ck.bad("results.json: 'planners' should be an object with the keys P1 ... P8")
        return {}
    missing = [p for p in PLANNERS if p not in planners]
    extra = [p for p in planners if p not in PLANNERS]
    if missing:
        ck.bad(f"results.json: 'planners' lacks {', '.join(missing)}")
    if extra:
        ck.bad(f"results.json: 'planners' has unexpected key(s) {', '.join(map(str, extra))}")
    return {p: planners[p] for p in PLANNERS if p in planners and check_record(ck, p, planners[p])}


def check_counterexamples(ck, names, read, records, hw1_cmd):
    """Instance rules for every counterexample file, and agreement with results.json."""
    for n in sorted(names):
        if n.startswith("counterexamples/") and n not in CE_FILES.values():
            if JUNK.search(n):
                ck.warn(f"{n} is not needed in the zip (system file); it is ignored")
            else:
                ck.bad(f"unexpected file {n}: counterexamples/ holds only P1.json ... P8.json "
                       "(names are case-sensitive)")
    for name in PLANNERS:
        zn = CE_FILES[name]
        present = zn in names
        rec = records.get(name)
        if present:
            data = read(zn)
            raw, inst, probs = check_ce.read_instance_bytes(data)
            for p in probs:
                ck.bad(f"{zn}: {p}")
            size = len(raw["obstacles"]) if isinstance(raw, dict) and isinstance(raw.get("obstacles"), list) else None
        if rec is None:
            continue
        claimed = rec.get("claimed")
        if claimed == "flawed" and not present:
            ck.bad(f"results.json claims {name} flawed, but {zn} is not in the zip")
            continue
        if claimed == "correct" and present:
            ck.bad(f"{zn} is in the zip, but results.json claims {name} correct; rerun {hw1_cmd}")
            continue
        if not present:
            continue
        if rec.get("digest") != check_ce.instance_digest(data):
            ck.bad(f"results.json does not match {zn}: the file changed after HW1.py wrote results.json; "
                   f"rerun {hw1_cmd}")
            continue
        if rec.get("size") != size or rec.get("valid_instance") != (not probs and inst is not None):
            ck.bad(f"results.json: the size or validity recorded for {name} does not match {zn}; rerun {hw1_cmd}")
        if not probs:
            if rec.get("verdict") == "not_a_counterexample":
                ck.warn(f"results.json records {zn} as NOT A COUNTEREXAMPLE for {name}")
            elif rec.get("verdict") is None:
                ck.warn(f"results.json has no verdict for {name} (the check was not completed; see 'errors')")
    n_correct = sum(1 for r in records.values() if r.get("claimed") == "correct")
    if records and len(records) == len(PLANNERS) and n_correct != 2:
        ck.warn(f"results.json claims {n_correct} planner(s) correct; exactly two of the eight are correct")


def check_answers(ck, data, records, present=None):
    """answers.txt: the rules of hw1_io.parse_answers are problems. These warn: a blank answer for a
    planner that results.json claims flawed; a principle or guarantee line for a planner without a
    counterexample file in the zip (present: the planners with a file), which scores 0; T1.planner -
    (T1 skipped); and a T1 planner that does not fit the question (a planner you judged flawed
    whose counterexample file has two or more obstacles)."""
    values, problems = hw1_io.parse_answers(data)
    for p in problems:
        ck.bad(p)
    blank = [p for p in PLANNERS if (records.get(p) or {}).get("claimed") == "flawed"
             and any(hw1_io.answer_is_blank(values.get(f"{p}.{w}")) for w in ("principle", "guarantee"))
             and f"{p}.principle" in values and f"{p}.guarantee" in values]
    if blank:
        ck.warn(f"answers.txt has a blank principle or guarantee line for {', '.join(blank)}, which results.json "
                "claims flawed; a blank answer scores 0 and, if that planner is drawn, counts as 0 in the fraction "
                "f that your other answers inherit (README, Points and grading)")
    if present is not None:
        orphans = [p for p in PLANNERS if p not in present
                   and any(not hw1_io.answer_is_blank(values.get(f"{p}.{w}")) for w in ("principle", "guarantee"))]
        if orphans:
            ck.warn(f"answers.txt has a principle or guarantee line for {', '.join(orphans)}, but the zip has no "
                    f"counterexample file for {'it' if len(orphans) == 1 else 'them'} "
                    f"({', '.join(CE_FILES[p] for p in orphans)}): these answers score 0 without a counterexample "
                    "file")
    p = values.get("T1.planner")
    if p == hw1_io.SKIP_T1:
        ck.warn("answers.txt: T1.planner is -, so you skip T1 (it scores 0)")
        return
    rec = (records.get(p) or {}) if p in PLANNERS else {}
    what = "a planner you judged flawed whose counterexample file has two or more obstacles"
    if rec.get("claimed") == "correct":
        ck.warn(f"answers.txt: T1.planner is {p}, which results.json claims correct; T1 asks for {what}")
    elif rec.get("claimed") == "flawed" and isinstance(rec.get("size"), int) and rec["size"] < 2:
        ck.warn(f"answers.txt: T1.planner is {p}, whose counterexample file has {rec['size']} obstacle(s); T1 asks "
                f"for {what}")


def check_kit_files(ck, names, read):
    """Warn about every DO-NOT-EDIT file in code/ that differs from the original kit: the SHA-256
    recorded in MANIFEST.json, or, without a manifest, this kit's own copy."""
    manifest = check_ce.load_manifest(HERE)
    if manifest is None:
        ck.warn(f"{check_ce.MANIFEST} is missing from this kit, so the DO-NOT-EDIT files in code/ were compared "
                "with this kit's own copies (copy MANIFEST.json from the kit zip)")
    edited = []
    for f in sorted(set(DO_NOT_EDIT) | set(manifest or ())):
        zn = f"code/{f}"
        if zn not in names:
            continue
        if manifest is not None and f in manifest:
            want = manifest[f]
        elif (HERE / f).is_file():
            want = check_ce.file_sha256((HERE / f).read_bytes())
        else:
            continue
        try:
            data = read(zn)
        except Exception:   # an unreadable member: not this check's business
            continue
        if check_ce.file_sha256(data) != want:
            edited.append(zn)
    if edited:
        ck.warn(f"{', '.join(edited)} {'differs' if len(edited) == 1 else 'differ'} from the original kit "
                f"({check_ce.MANIFEST}): DO-NOT-EDIT files are replaced by the originals when your submission is "
                "graded, so the grade may not match results.json (restore the original files and rerun HW1.py)")


def check_zip(path: Path) -> Checker:
    ck = Checker()
    m = NAME_RE.match(path.name)
    if not m:
        ck.bad(f"file name {path.name!r} should look like HW1_<ID1>_<ID2>.zip (or HW1_<ID>.zip), IDs in digits")
        ids_from_name = []
    else:
        ids_from_name = [g for g in m.groups() if g]
    hw1_cmd = "python HW1.py --ids " + (" ".join(_norm_ids(ids_from_name)) if ids_from_name else "ID1 ID2")
    if not path.is_file():
        ck.bad(f"{path} not found")
        return ck
    try:
        zf = zipfile.ZipFile(path)
    except zipfile.BadZipFile:
        ck.bad(f"{path} is not a valid zip file")
        return ck
    with zf:
        # Normalized name -> name stored in the zip. Windows PowerShell 5.1 Compress-Archive
        # stores backslashes (code\HW1.py); zipfile converts them only on Windows.
        raw = {}
        for n in zf.namelist():
            if not n.replace("\\", "/").endswith("/"):
                raw.setdefault(str(PurePosixPath(n.replace("\\", "/"))), n)
        names = set(raw)
        read = lambda name: zf.read(raw[name])  # noqa: E731
        stored = [getattr(i, "orig_filename", i.filename) for i in zf.infolist()]
        if any("\\" in n for n in stored):
            ck.warn("the zip stores paths with backslashes (Windows PowerShell Compress-Archive); it is "
                    "accepted, but python make_submission.py --ids ... writes a portable zip")
        size_mb = sum(i.file_size for i in zf.infolist()) / 2 ** 20
        heavy = sorted({p for n in names for p in n.split("/")[:-1]
                        if p in ("site-packages", ".venv", "venv", "__pycache__", ".git")})
        if size_mb > SIZE_WARN_MB or heavy:
            ck.warn(f"the zip holds {size_mb:.0f} MB" + (f" and the folder(s) {', '.join(heavy)}" if heavy else "")
                    + "; leave virtual environments and caches out (make_submission.py does)")
        missing = [r for r in REQUIRED if r not in names]
        if missing:
            tops = {n.split("/", 1)[0] for n in names if "/" in n}
            wrapped = sorted(t for t in tops if f"{t}/report.pdf" in names or f"{t}/results.json" in names)
            if wrapped:
                ck.bad(f"the files are inside the folder '{wrapped[0]}/'; zip the contents so that report.pdf, "
                       "results.json, counterexamples/, and code/ are at the root of the zip")
                return ck
            for r in missing:
                ck.bad(f"missing required file: {r}")

        if "report.pdf" in names and read("report.pdf")[:5] != b"%PDF-":
            ck.bad("report.pdf is not a PDF file")

        check_kit_files(ck, names, read)

        records = {}
        if "results.json" in names:
            try:
                res = json.loads(read("results.json").decode("utf-8-sig"))
            except (ValueError, UnicodeDecodeError) as exc:
                ck.bad(f"results.json is not valid JSON: {exc}")
            else:
                records = check_results(ck, res, ids_from_name)
        check_counterexamples(ck, names, read, records, hw1_cmd)
        if "answers.txt" in names:
            check_answers(ck, read("answers.txt"), records, present={p for p in PLANNERS if CE_FILES[p] in names})
    return ck


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except Exception:
            pass
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: python check_submission.py HW1_<ID1>_<ID2>.zip")
        return 2
    try:
        ck = check_zip(Path(argv[0]))
    except check_ce.EnvironmentBroken as exc:   # no verdict on the zip: the Python environment is broken
        print(f"ERROR: {exc}")
        print("check_submission.py could not check the zip: fix the Python environment, then run it again.")
        return 2
    for w in ck.warnings:
        print(f"WARNING: {w}")
    for p in ck.problems:
        print(f"PROBLEM: {p}")
    if ck.problems:
        print(f"FAILED: {len(ck.problems)} problem(s). Fix them before you submit: a zip that fails this check "
              "is not graded (0 points).")
        return 1
    print("OK: the submission passes check_submission.py" + (f" ({len(ck.warnings)} warning(s))" if ck.warnings else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
