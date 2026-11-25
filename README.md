# HW1 kit: Translational Motion Planning (236767, Winter 2026-27)

In this homework you do not write a planner. Eight planners, `planners/P1.py` to `planners/P8.py`, each claim to solve the problem below, and you referee them (Part B): for each one you decide whether it is correct, and for a flawed one you find a counterexample, an instance on which it gives a wrong answer. Part C asks for one short proof, T1. The handout (HW1.pdf) is the authority on the items, points, and deadlines; this file explains how to run the kit (version 2.4).

## What is in the kit

| File | What it does | May you edit it? |
| --- | --- | --- |
| `planners/P1.py` ... `P8.py` | The eight planners. Each defines `plan(robot, obstacles, start, goal)`. Read them. | No (DO NOT EDIT) |
| `referee/` | The referee: the correct answer (`referee.solve`) and the validity test (`referee.path_valid`). | No (DO NOT EDIT) |
| `check_ce.py` | Checks one counterexample file against one planner. | No (DO NOT EDIT) |
| `draw.py` | Figures: an instance with its C-obstacles and paths, and the Part A warm-up. | No (DO NOT EDIT) |
| `animate.py` | An animation of the robot moving along a path, which shows where it overlaps an obstacle. | No need |
| `HW1.py` | Checks all your counterexample files and writes `results.json`. | No need |
| `gen_hw1.py` | Your robot and your Part A warm-up obstacle, from your IDs. | No (DO NOT EDIT) |
| `hw1_io.py` | Reads and writes instance files; `validate_instance` lists the rules an instance breaks. | No (DO NOT EDIT) |
| `check_submission.py` | Checks your zip before you upload it. | No (DO NOT EDIT) |
| `make_submission.py` | Builds your zip and runs `check_submission.py` on it. | No need |
| `report_template.tex` | The report template: the AI-use box, one section for Part B (one subsection per planner), and one section for Part C. | Yes: write your report in it |
| `answers.txt` | A skeleton of your one-line answers: principles, guarantee lines, and the planner of T1. | Yes: fill it in |
| `counterexamples/` | Your counterexample files, `P1.json` ... `P8.json`. | Yes: this is where your work goes |
| `example/` | A solved instance (see below). | No |
| `requirements.txt` | The pinned packages of the grading sandbox. | No |
| `MANIFEST.json` | The SHA-256 fingerprints of the original DO-NOT-EDIT files. | No (DO NOT EDIT) |

Files marked DO NOT EDIT are replaced by the originals when your submission is graded, the planners included, so changes to them are lost. You may of course copy a planner or the referee into a script of your own and experiment there. `check_ce.py`, `HW1.py`, `draw.py`, `gen_hw1.py`, and `check_submission.py` compare the DO-NOT-EDIT files with `MANIFEST.json` and print a warning (and still run) when one differs, for example a planner with a debugging edit left in it: your results would then come from the edited file, while the grader uses the original.

## Setup

Use Python 3.12, the version of the grading sandbox.

```
python -m venv .venv                                     (Windows: py -3.12 -m venv .venv)
source .venv/bin/activate                                (Linux, macOS)
pip install -r requirements.txt
```

On Windows you do not need to activate the environment (PowerShell blocks `Activate.ps1` by default): call its Python directly, as in `.venv\Scripts\python.exe -m pip install -r requirements.txt` and `.venv\Scripts\python.exe check_ce.py ...`, or use `.venv\Scripts\activate.bat` in cmd. `make_submission.py` leaves `.venv` out of your zip, so it may live in this folder. On Windows, keep the path of the kit folder short (for example `C:\hw1\kit`): pip cannot install numpy into a virtual environment whose path is longer than about 150 characters, as deep folders under OneDrive or Downloads can be.

The sandbox has exactly these packages: numpy, scipy, shapely, networkx, and matplotlib, at the versions in `requirements.txt`. Install them into the Python you run the tools with (the virtual environment above), not with `pip install --user` and not through `PYTHONPATH`: `check_ce.py` runs each planner in an isolated Python process (`python -I`), which ignores both. If that process cannot import the packages, `check_ce.py` reports that the Python environment is broken; this is no verdict on the planner, so fix the environment and check again.

In all commands, `ID1 ID2` are the IDs of the pair; a student working alone passes one ID (the same ID twice is refused). Pass each ID as its own argument, separated by a space (`--ids 123456789 987654321`), not joined by a comma and not inside one pair of quotes; the tools refuse anything else. The IDs are sorted internally, so their order does not matter. Use your full nine-digit IDs, leading zeros included (the kit pads an all-digit ID shorter than nine digits with leading zeros).

## The setting in brief

- **Robot.** Your pair's robot R is a right triangle that translates without rotating. Its reference point is the right-angle vertex, and its vertices relative to that point are `(0, 0)`, `(±a, 0)`, and `(0, ±b)`, with a, b in {0.4, ..., 0.8}. `python gen_hw1.py --ids ID1 ID2 --robot` prints it. A placement is a position of the reference point.
- **C-obstacle.** The C-obstacle of an obstacle O is O ⊕ (−R) = {o − r : o ∈ O, r ∈ R}, the placements at which the robot meets O. Mind the minus sign: your triangle is not centrally symmetric, so −R ≠ R.
- **Paths.** A path is a polygonal chain of placements from the start to the goal. It is valid if no point of it lies in the interior of a C-obstacle, tested per C-obstacle with a tolerance of 1e-9 (a point is in the interior only if it is inside by more than 1e-9). Touching a C-obstacle, sliding along its edge, and passing through its vertex are allowed. The cost of a path is its Euclidean length.
- **The answer.** A planner must return a shortest valid path, or no path when none exists. If the start or the goal lies in the interior of a C-obstacle, no path exists.

Every planner defines `plan(robot, obstacles, start, goal) -> (path, cost)`. The robot and every obstacle are lists of `(x, y)` tuples in counter-clockwise order; `path` is a list of `(x, y)` tuples from the start to the goal, or `None`; `cost` is a float, and `math.inf` when `path` is `None`.

## Instance files

A counterexample is an instance, stored as a JSON file. The robot is not in the file: it is always your pair's robot.

```json
{"obstacles": [
  [[6, 5], [7, 6], [6, 7], [5, 6]]
 ],
 "start": [5.4, 5.4],
 "goal": [6, 6]}
```

The rules (`hw1_io.validate_instance(inst)` returns the list of rules an instance breaks; an empty list means it is valid):

- every point is a list `[x, y]` of two numbers (not strings, and not `true` or `false`);
- every coordinate is a multiple of 0.001, written with at most three digits after the decimal point, and lies in [0, 20];
- there are 0 to 20 obstacles;
- each obstacle is a convex polygon with 3 to 8 vertices in counter-clockwise order, without repeated vertices and without three consecutive collinear vertices;
- obstacles have pairwise-disjoint interiors (they may touch);
- the start and the goal are different points.

The start or the goal may lie in the interior of a C-obstacle; then no path exists. The size of an instance is its number of obstacles. `hw1_io.load_instance(path)` and `hw1_io.save_instance(path, inst)` read and write these files; every tool snaps the coordinates it reads to the 0.001 grid (`round(v * 1000) / 1000`), so a planner and the referee always see exact grid values. Save files that you write by hand as UTF-8 text: in Windows PowerShell 5.1, `>` writes UTF-16, which the kit does not read (`check_ce.py` then says which encoding it found).

## Commands

### Your robot and the Part A warm-up

```
python gen_hw1.py --ids ID1 ID2 --robot           (add --json for machine-readable output)
python gen_hw1.py --ids ID1 ID2 --warmup
python draw.py --ids ID1 ID2 --warmup             (writes figs/warmup.pdf)
```

`--warmup` prints the obstacle O of the Part A warm-up. Compute the vertices of O ⊕ (−R) by hand, then compare them with `draw.py --warmup`, which prints them and draws O, the C-obstacle with the coordinates of its vertices, and the robot placed with its reference point at each of these vertices.

### Checking a counterexample: `check_ce.py`

```
python check_ce.py --ids ID1 ID2 counterexamples/P<k>.json [--draw fig.pdf] [--json]
```

Here and below, `P<k>` stands for one planner, `P1` ... `P8`. The file name picks the planner: `P<k>.json` is checked against `planners/P<k>.py`. A draft may be called `P<k>_try2.json` (also checked against P<k>), and `--planner P<k>` picks the planner for a file with any name. The tool

1. checks the instance rules; an instance that breaks one is reported as `INVALID INSTANCE`, with every broken rule, and nothing is run;
2. runs the planner's `plan` in a separate, isolated Python process (`python -I`) that receives only the robot and the instance, with a limit of 30 s on the call (the time to start Python and import the planner does not count);
3. runs the referee on the same instance;
4. prints the size of the instance, both answers, and `COUNTEREXAMPLE (kind)` or `NOT A COUNTEREXAMPLE`, with a one-line explanation.

A planner is wrong on an instance, and the instance is a counterexample, in exactly these cases (the kind is printed in parentheses):

| Kind | The planner ... |
| --- | --- |
| `missed_path` | returns no path, but a path exists |
| `false_path` | returns a path, but no path exists |
| `invalid_path` | returns a path with a point in the interior of a C-obstacle, or a path that does not start at the start and end at the goal (within 1e-6) |
| `suboptimal` | returns a valid path that is longer than the optimum by more than 1e-6 |
| `wrong_cost` | reports a cost that differs from the length of its path by more than 1e-6 (with no path, the cost must be inf) |
| `crash` | raises an exception, or returns something that is not a pair (path, cost) |
| `timeout` | takes more than 30 s |

When several apply, the first of timeout, crash, missed_path, false_path, invalid_path, suboptimal, and wrong_cost is reported. The exit code is 0 for COUNTEREXAMPLE, 1 for NOT A COUNTEREXAMPLE, and 2 for an invalid instance or a check that could not be completed (for example, a missing planner file, or a broken Python environment). Messages number the obstacles 0, 1, ... in the order of the file, as `draw.py` does. If a DO-NOT-EDIT file differs from `MANIFEST.json`, the tool prints a warning first and still runs. `--draw fig.pdf` (or `.png`) also draws the instance with both paths. `--json` prints one JSON object instead of the text report: the fields of a `results.json` record (below) except `claimed`, plus `kit_version`, `ids`, `robot`, `planner`, `planner_time_s` (the seconds spent in `plan`), and `error` (why the check could not be completed, or `null`).

### Figures: `draw.py`

```
python draw.py --ids ID1 ID2 counterexamples/P<k>.json --planner P<k> [--out figs/P<k>.pdf]
```

draws the instance for your robot: the obstacles, numbered 0, 1, ... in the order of the file (the numbers in the messages of `check_ce.py`), their C-obstacles O ⊕ (−R), each with its own outline (so two C-obstacles that touch along an edge show that edge), the robot at the start and at the goal, the referee's path, and, with `--planner P<k>`, the path of P<k> (run as `check_ce.py` runs it). The legend gives both costs. The default output is `figs/<name of the instance file>.pdf`; `--out` with `.pdf` or `.png` chooses the file. The figures in your report must come from this script.

### Watching the motion: `animate.py`

```
python animate.py --ids ID1 ID2 counterexamples/P<k>.json --planner P<k>
```

opens a window with two panels side by side: on the left, the obstacles and your robot moving along P<k>'s path; on the right, the C-obstacles and the reference point moving along the same path. The robot is green when it is free, orange when it touches an obstacle (allowed), and red when it overlaps one, with the overlap filled dark red and the C-obstacle outlined red. The overlap test is the referee's, so the robot turns red exactly where `check_ce.py` finds the path invalid; every such stretch gets frames, however short, the animation pauses at its first frame, and the tool prints each stretch. Space pauses and resumes, the arrow keys step one frame, Home restarts, and the slider scrubs.

Without `--planner`, or when P<k> returns no path, the referee's path is animated (`--referee` forces it); `--path FILE` animates a path of your own, a JSON list of points `[[x, y], ...]`. Where no window can open (no display, or a Python without tkinter), the tool writes `figs/<name of the instance file>_anim.html`, a page with its own play controls, and opens it in your web browser. `--save FILE` also writes the animation as `.html` or `.gif`, and `--no-show` skips the window. Nothing `animate.py` writes is graded, and the figures in your report come from `draw.py`.

### All planners: `HW1.py` and `results.json`

```
python HW1.py --ids ID1 ID2 [--figs figs]
```

For each planner P<k>: if `counterexamples/P<k>.json` exists, you claim that P<k> is flawed, and the file is checked as by `check_ce.py`; otherwise you claim that P<k> is correct. The run prints a summary table and writes `results.json`. With `--figs figs` it also draws `figs/P<k>.pdf` for every file it checks, the figures that `report_template.tex` includes. Only the exact names `P1.json` ... `P8.json` count (names are case-sensitive); drafts such as `P<k>_try2.json` are ignored. Run `HW1.py` again after every change to a counterexample file: `check_submission.py` compares `results.json` with the files. Submit the `results.json` that this command writes; never edit it by hand.

```json
{"ids": ["ID1", "ID2"], "kit_version": "2.4", "robot": [[0, 0], [0.6, 0], [0, 0.5]],
 "planners": {"P1": {...}, "P2": {...}, ..., "P8": {...}},
 "errors": []}
```

The record of a planner P<k> with a file `counterexamples/P<k>.json`, and that of a planner without one:

```json
{"claimed": "flawed", "file": "counterexamples/P<k>.json", "valid_instance": true, "problems": [],
 "verdict": "counterexample", "kind": "<kind>", "size": <number of obstacles>,
 "planner_answer": {"path": [[x, y], ...], "cost": <cost>},
 "referee_answer": {"path": [[x, y], ...], "cost": <cost>},
 "detail": "...", "digest": "..."}

{"claimed": "correct", "file": null, "valid_instance": null, "problems": [],
 "verdict": null, "kind": null, "size": null, "planner_answer": null,
 "referee_answer": null, "detail": null, "digest": null}
```

| Key | Content |
| --- | --- |
| `ids`, `robot` | The sorted IDs and their robot. |
| `claimed` | `"flawed"` if `counterexamples/P<k>.json` exists, otherwise `"correct"`. |
| `file` | The counterexample file, or `null`. |
| `valid_instance`, `problems` | Whether the file follows the instance rules, and the rules it breaks. |
| `verdict` | `"counterexample"`, `"not_a_counterexample"`, or `null` (invalid instance, or the check could not be completed; see `errors`). |
| `kind` | The kind of counterexample (table above), or `null`. |
| `size` | The number of obstacles in the file. |
| `planner_answer`, `referee_answer` | `{"path", "cost"}` of the planner and of the referee; `path` is `null` when there is no path, and a cost that is not a finite number is written as `"inf"` (JSON has no infinity). `planner_answer` is `null` after a crash or a timeout. |
| `detail` | The one-line explanation that `check_ce.py` prints. |
| `digest` | A fingerprint of the file's contents, with which `check_submission.py` confirms that `results.json` was written from the files you submit. |
| `errors` | Problems of the run, such as a check that could not be completed or a file name with the wrong case. |

### Your one-line answers: `answers.txt`

The principle and the guarantee line of every planner, and the planner of your proof T1, go into `answers.txt`, one line per key; the TA grades the principles and guarantee lines from this file. The kit ships a skeleton with every key. Filled in, its lines look like this (`P<k>` and `P<j>` stand for planners you judged flawed and correct, respectively):

```
# comment lines start with #
P<k>.principle: <one sentence>
P<k>.guarantee: <one sentence>
P<j>.principle: -
P<j>.guarantee: -
...
T1.planner: P<k>
```

The keys are `P1.principle`, `P1.guarantee`, ..., `P8.principle`, `P8.guarantee`, and `T1.planner`, each exactly once, written `KEY: text`. Every answer fits on its line and has at most 400 characters. Write `-` for both keys of a planner you judged correct; a blank answer (`-` or nothing) earns 0. The principle and the guarantee line of a planner for which you submit no counterexample file earn 0, whatever they say; a file `counterexamples/P<k>.json` counts even if `check_ce.py` does not confirm it (it must still follow the instance rules). `T1.planner` names one planner, `P1` ... `P8`: a planner you judged flawed whose counterexample file has two or more obstacles; the proof itself goes into your report. Write `T1.planner: -` if you skip T1; T1 then earns 0. Save the file as UTF-8 text. `check_submission.py` rejects a missing file, a missing, repeated, or unknown key, an answer that spills onto a second line, an answer longer than 400 characters, and a `T1.planner` that is neither one of `P1` ... `P8` nor `-` (the skeleton's placeholder `P?` included).

### Building and checking the submission

```
python make_submission.py --ids ID1 ID2          (writes HW1_<IDs>.zip, then runs check_submission.py on it)
python check_submission.py HW1_<IDs>.zip
```

The name of the zip lists the IDs in increasing order, whatever order you pass them in; `make_submission.py` prints the exact `check_submission.py` command for the zip it wrote.

`make_submission.py` takes `report.pdf`, `results.json`, `answers.txt`, and `counterexamples/P<k>.json` from the current folder (`answers.txt`, if it is not there, from the kit folder; `--answers FILE` picks another file), and every `.py` and `.ipynb` file of this kit folder and its subfolders as `code/`, leaving out `.venv`, caches, and the folders `counterexamples/`, `example/`, `figs/`, `plots/`, `code/`, and `submission/` (it lists the scripts it leaves out of these folders). Without `report.pdf` it takes `report_template.pdf`, the PDF of the template, and says so; `--report FILE` picks another PDF. If you zip by hand, zip the contents, not the folder, and prefer `make_submission.py` to Windows PowerShell's `Compress-Archive`, which stores paths with backslashes.

`check_submission.py` checks the file name, the required files, the `results.json` schema, that `results.json` belongs to the IDs in the file name and to their robot, that every counterexample file follows the instance rules, that `results.json` matches the counterexample files (the same planners and the same contents), and the rules of `answers.txt` (above). It prints every problem and exits with 0 only if there are none. It also warns, without failing, among others: when a DO-NOT-EDIT file in `code/` differs from `MANIFEST.json`; when `results.json` records a counterexample file as NOT A COUNTEREXAMPLE, or has no verdict for it; when `results.json` claims a number of correct planners other than two (exactly two of the eight are correct); when the `errors` list of `results.json` is not empty; when a planner that `results.json` claims flawed has a blank answer (it earns 0 and, if its planner is drawn, counts as 0 in the fraction f of Points and grading); when a planner without a counterexample file has a principle or a guarantee line (these answers score 0 without a counterexample file); when `T1.planner` is `-` (you skip T1); when `T1.planner` is not a planner you claim flawed with a counterexample file of two or more obstacles; and when the zip stores paths with backslashes, holds system files, or is very large. A zip that fails it will not be checked: it is not graded, and earns 0 points.

## The example folder

`example/` holds a solved instance for the robot of the made-up ID 123456789, R = (0, 0), (0, 0.7), (−0.5, 0): `instance.json` (one diamond-shaped obstacle; the start (5.4, 5.4); the goal (6, 6), the center of the obstacle), `robot.json` (the ID and its robot), and `answer.json` (the referee's answer and the C-obstacle O ⊕ (−R), whose vertices are (6, 4.3), (7, 5.3), (7.5, 6), (6.5, 7), (6, 7), (5, 6), and (5, 5.3)).

The start lies outside the obstacle, but the robot placed there overlaps it: its vertex (5.4, 6.1) is inside the obstacle. So the start lies in the interior of the C-obstacle, and no path exists, even before we look at the goal, which lies inside the obstacle itself (so that no robot has a path on this instance). Use the ID 123456789 to try the tools on it:

```
python gen_hw1.py --ids 123456789 --robot
python draw.py --ids 123456789 example/instance.json --out figs/example.pdf
python check_ce.py --ids 123456789 example/instance.json --planner P<k>
```

`draw.py` shows the robot at the start overlapping the obstacle and writes "referee: no path" in the legend. Every one of the eight planners also reports that no path exists, so for each `P<k>`, `check_ce.py` prints

```
check_ce.py P<k> on example/instance.json
  robot of 123456789: (0, 0) (0, 0.7) (-0.5, 0)
  size: 1 (number of obstacles)
  P<k> answer:     no path (cost inf)   [plan() took 0.02 s]
  referee answer:  no path (cost inf)
NOT A COUNTEREXAMPLE: P<k> and the referee both report that no path exists
```

and exits with code 1. With another ID the robot and its C-obstacle change, but no robot has a path on this instance.

## Your report

Write the report in English, in LaTeX, with `report_template.tex`: the AI-use box first, then one section for Part B with one subsection per planner, P1 to P8, then one section for Part C (the dry question, T1). Part A does not go into the report.

- The AI-use box (at most five lines): (i) the tools you used, (ii) the items you used them on, and (iii) one thing a tool got wrong that you caught (or "none found").
- For a flawed planner: the verdict, the figure of your counterexample (`python draw.py --ids ID1 ID2 counterexamples/P<k>.json --planner P<k> --out figs/P<k>.pdf`, or all at once with `python HW1.py --ids ID1 ID2 --figs figs`), the principle the planner violates (one sentence), and its guarantee line (one sentence), the same as in `answers.txt`, which is what the TA grades.
- For a correct planner: the verdict (and no counterexample file).
- Part C: the proof T1, in at most half a page, for the planner you name as `T1.planner` in `answers.txt`.

The template compiles with or without the figures; a missing figure shows a box with the command that makes it.

## Points and grading

| Part | Item | Points | Graded by |
| --- | --- | ---: | --- |
| B | Flawed planner (× 6): a counterexample file that `check_ce.py` confirms | 7 each | script |
| | its size equals the fewest obstacles possible | 2 each | script |
| | the principle (`answers.txt`) | 3 each | TA |
| | the guarantee line (`answers.txt`) | 2 each | TA |
| | Correct planner (× 2): no counterexample file | 3 each | script |
| C | T1 nothing smaller works (proof in the report) | 10 | TA |
| | Total | 100 | script 60, TA 40 |

Each planner is graded by what it is, whatever you claim: if you judge a flawed planner correct, you earn 0 for it, and if you submit a counterexample file for a correct planner, you earn 0 for it. In particular, if you submit no counterexample file for a flawed planner, its principle and its guarantee line earn 0, whatever `answers.txt` says. A file `counterexamples/P<k>.json` counts even if `check_ce.py` does not confirm it (it must still follow the instance rules): the TA then grades both answers of that planner on their merits. A guarantee line is a class of instances on which the planner is correct: a guarantee line whose class contains any instance on which the planner is wrong earns 0, with no partial credit for a false class; in particular, a guarantee line that your own counterexample contradicts earns 0.

Grading of written answers. A script grades your counterexample files; the TA grades your written answers.

- Proof T1 is always graded in full, unless `T1.planner` is `-` (you skip T1) or names a planner that is correct; T1 then earns 0.
- The principles and guarantee lines are sampled. After the deadline, two planners are drawn at random among the flawed planners for which you submitted a counterexample file and wrote at least one non-blank answer (all of them, if there are fewer than two). The TA grades both answers of each drawn planner; a blank one earns 0 of its points.
- Every other non-blank principle and guarantee line of these planners earns its points times f, the fraction of the possible points that the drawn answers earned together: f = (points of the drawn answers) / (points possible for them), a blank drawn answer included. For example, drawn answers that earn 3, 0, 3, and 0 of 3, 2, 3, and 2 points give f = 6/10 = 0.6, so each other principle earns 1.8 and each other guarantee line 1.2.
- Every other blank answer (`-` or nothing) earns 0, and so does every answer of a planner without a counterexample file (above); the planners without a counterexample file, or with two blank answers, are never drawn.
- Your total is the exact sum, rounded to the nearest integer, halves up (89.5 becomes 90).
- You may ask for a full regrade within one week of receiving your grade. The TA then grades every principle and guarantee line of the planners for which you submitted a counterexample file, and the new grade replaces the old one, whether it is higher or lower.

A zip that fails `check_submission.py` is not graded: it earns 0 points.

## Submission

HW1 is due on the date in the handout, at 23:59. No late submission will be accepted unless approved in advance.

One zip per pair, named `HW1_ID1_ID2.zip` (or `HW1_ID.zip` for a student working alone), uploaded to the course website, with these files at the root of the zip (not inside a folder):

```
report.pdf                     your report, from report_template.tex
results.json                   written by python HW1.py --ids ..., never edited by hand
answers.txt                    your one-line answers (principles, guarantee lines, T1.planner)
counterexamples/P<k>.json      one file for each planner you claim flawed
code/                          this kit (with planners/ and referee/) and any scripts of yours
```

`python make_submission.py --ids ID1 ID2` builds this zip (its name lists the IDs in increasing order) and runs `python check_submission.py` on it, printing the exact command. The zip must pass the kit's checker: a zip that fails will not be checked (it is not graded, and earns 0 points). Run the check on the very zip you upload.

## Using the kit in your own scripts

Your scripts may import the kit (run them from this folder, or put it on `sys.path`):

```python
import hw1_io, gen_hw1, check_ce
from referee import referee

ids = ["123456789", "987654321"]                            # your IDs
f = "counterexamples/P<k>.json"                             # with <k> the number of a planner
robot = gen_hw1.robot_for_ids(ids)
inst = hw1_io.load_instance(f)
print(hw1_io.validate_instance(inst))                       # [] if the instance follows the rules
answer = referee.solve(robot, inst["obstacles"], inst["start"], inst["goal"])   # dict: path, cost, path_exists, c_obstacles
record = check_ce.check(f, ids)                             # what --json prints
hw1_io.save_instance("counterexamples/P<k>.json", inst)     # writes the file (and the folder, if needed)
```
