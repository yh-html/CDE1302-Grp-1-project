# Folder reorganisation plan (do AFTER the graded run)

Written 12 Nov 2026. Nothing has been moved yet. Do this after the demo, then do a
props-off link check (`simple_arm_takeoff_land --drone-id 1 --monitor-only`) and
`--plan-only` before the next flight, because configs and commands use repo paths.

## Why it looks messy

1. **Miniforge is installed in the repo folder.** `python.exe`, ~40 `.dll` files, `Lib/`,
   `DLLs/`, `pkgs/`, `envs/cde1302-laptop`, `conda-meta/`, `Uninstall-Miniforge3.exe` …
   Git ignores them, but they are most of what Explorer shows. A conda install can't be
   moved; it has to be reinstalled elsewhere.
2. **Course and personal material sits at the top level** (some ignored, some untracked).
3. **The tracked repo has a few overlapping folders**: CAD in both `CAD/` and `codex/`,
   old SAFMC/waypoint material, and a `CLAUDE.md` that still describes the swarm project.

## Step 1: move course/personal material to `C:\cde1302_course\` (no code uses it)

| Now (in `C:\cde1302\`) | Move to `C:\cde1302_course\` |
|---|---|
| `Lecture Notes/` | `lectures/` |
| `Labs/` | `labs/` |
| `Assignment/`, `Assignment_Submission/`, `CDE1302_UAV/` | `assignment/` |
| `Mini Project/` (brief, briefing, Flying Area PDFs) | `mini_project_brief/` |
| `Pictures/`, `Pictures of RC antena/`, `fc.jpg`, `photo_2026-11-03_16-42-13.jpg` | `photos/` |
| `photos of arena/` (videos + `drone arena.SLDPRT`) | `photos/arena/` |
| `FLIGHT MAPS/`, `Tag_mission_live_map.png` | `flight_maps/` |
| `Drone Frame Design/`, `Actual Mod STL of drone frame/` | `frame_design/` |
| `micoair_docs/` | `datasheets/micoair/` |
| `cde1302-chassis-draft-prompt.md`, `cde1302-chassis-v3-prompt.md`, `cwp.py` | `scratch/` |

Then remove their lines from `.gitignore` (`/Lecture Notes/`, `/Labs/`, `/Pictures/`,
`/Assignment/`, `/CDE1302_UAV/`). The three YAML configs mention `photos of arena/...` in a
comment only; update the comment.

Keep in the repo: `AGENTS.md`, `LICENSE` (decide whether to commit them).

## Step 2: tidy the tracked repo (one commit, run the tests after)

Target layout:

```
cde1302/
  README.md           project overview (updated: single-UAV mission first)
  CLAUDE.md           rewritten for the mini-project (swarm notes moved to docs/archive)
  CMakeLists.txt, sdkconfig*, dependencies.lock, flash_drone.sh   ESP-IDF: must stay at root
  main/  components/  managed_components/  tools/                  firmware: unchanged
  laptop/             mission code + configs: unchanged
  trajectory/         MATLAB generator + CSVs: unchanged (send_trajectory examples use it)
  mini_project/       guides, flight tests, report: unchanged except below
  cad/
    fusion/           <- CAD/ (exports, screenshots, scripts)
    drone_cad_v1/ v2/ v3/ stock_mounts/ stock_plus/   <- codex/*
    README.md         <- codex/README.md + which design is flown (stock)
  docs/
    QGC_MAVLINK_BRIDGE.md
    archive/          <- SAFMC booklet, waypoints/, codex/ASTRA_DRONE_FRAME_PROMPT.md,
                         mini_project/mission/ (SAFMC rules digest), old swarm CLAUDE.md
```

Path updates this needs (all found with `git grep`):
- `README.md` lines 20, 22, 27, 239, 247 (folder table, waypoint links)
- `laptop/simple_waypoint_mission.py:18` (usage example), `mini_project/GUIDE.md`,
  `DRONE_BRINGUP_GUIDE.md`, `PREFLIGHT_GUIDE.md` (waypoint example paths, `codex/` links)
- the five `laptop/tag_mission*.yaml` comments that cite `codex/stock_mounts` and `waypoints/`
- `.gitignore` line `/codex/*/output/` -> `/cad/*/output/`
- delete `codex/drone_cad/.venv` (16k files, a local Python env) instead of moving it

Do the moves with `git mv` so history follows the files, then:
`python -m pytest -q laptop/tests` and `idf.py build` (firmware paths must be unchanged).

Optional: also commit the untracked work that belongs in the repo
(`mini_project/audit/`, `stock_analysis/`, `webots/`, `drone_wiring.drawio`,
`FLIGHT_COMMANDS.html`, `PROJECT_SUMMARY.md`, `codex/drone_cad_v3/`, `stock_mounts/`,
`stock_plus/`), or add them to `.gitignore`.

## Step 3: move Miniforge out of the repo

1. `conda env export -n cde1302-laptop > laptop/environment.lock.yml` (keep a record).
2. Run `C:\cde1302\Uninstall-Miniforge3.exe`.
3. Reinstall Miniforge to `C:\Users\<you>\miniforge3` and recreate the env from
   `laptop/environment.yml`.
4. Delete the leftover ignored entries from `.gitignore` (`/DLLs/`, `/Lib/`, `/python*`, `/*.dll` …).

The flight scripts currently run on `C:\Users\...\Python311\python` (first on PATH), not on
Miniforge, so Step 3 doesn't affect them; check `where python` before and after anyway.

## After all steps

- `git status` shows only intended changes; tests pass; firmware builds.
- Update `GRADED_RUN_CHECKLIST.md` / memory notes if any command changed (none should).

## Done 12 Nov (Steps 1 and 2; staged, not committed)

Step 1, plain moves to `C:\cde1302_course\` (no name clashes):

| From `C:\cde1302\` | To `C:\cde1302_course\` |
|---|---|
| `Lecture Notes/`, `Labs/`, `Mini Project/`, `micoair_docs/` | `lectures/`, `labs/`, `mini_project_brief/`, `datasheets/micoair/` |
| `Assignment/`, `Assignment_Submission/`, `CDE1302_UAV/` | `assignment/<same name>/` (kept apart: both zips are called `CDE1302_Krish_A0322900U.zip`) |
| `Pictures/` (contents), `Pictures of RC antena/`, `fc.jpg`, `photo_2026-11-03_16-42-13.jpg` | `photos/`, `photos/rc_antenna/`, `photos/` |
| `photos of arena/` | `photos/arena/` |
| `FLIGHT MAPS/`, `Tag_mission_live_map.png` | `flight_maps/` |
| `Drone Frame Design/`, `Actual Mod STL of drone frame/` | `frame_design/<same name>/` |
| `cde1302-chassis-*-prompt.md`, `cwp.py` | `scratch/` |

The graded-run screenshots went into the repo instead:
`graded run 1.png`, `GRADED RUN 2.png`, `GRADED RUN 3.png` -> `mini_project/graded_run_2026-11-12/run1..3.png`
with a README. The run logs stay in `laptop/mission_logs/20261112_*` (git-ignored).

Step 2 (`git mv`, except untracked folders, which were moved normally):

| From | To |
|---|---|
| `CAD/` | `cad/fusion/` |
| `codex/drone_cad/`, `codex/drone_cad_v2/` | `cad/drone_cad_v1/`, `cad/drone_cad_v2/` |
| `codex/drone_cad_v3/`, `codex/stock_mounts/`, `codex/stock_plus/` (untracked) | `cad/` |
| `codex/README.md` (rewritten), `codex/PROJECT_SUMMARY.md` | `cad/README.md`, `cad/PROJECT_SUMMARY.md` |
| `codex/ASTRA_DRONE_FRAME_PROMPT.md`, SAFMC booklet, `waypoints/` | `docs/archive/` |
| `mini_project/mission/` (untracked) | `docs/archive/mini_project_mission/` |
| `CLAUDE.md` (swarm) | `docs/archive/CLAUDE_swarm.md`; new `CLAUDE.md` for the mini-project |

Path references fixed: `README.md`, `laptop/simple_waypoint_mission.py`, the five
`laptop/tag_mission*.yaml` comments, `mini_project/GUIDE.md`, `DRONE_BRINGUP_GUIDE.md`,
`PREFLIGHT_GUIDE.md`, `flight_test_2026-11-1{0,1}/NOTES.md`, `cad/drone_cad_v{1,2}/README.md`,
the archived waypoint guide, and `.gitignore` (`/cad/*/output/`; course-folder lines removed).
`pytest laptop/tests`: 248 passed, 2 skipped. `--plan-only` on `tag_mission_demo_traj.yaml`: OK.
Firmware paths untouched; `idf.py build` not run.

Not moved / still to do:
- `codex/drone_cad/.venv` was left where it was (not deleted, not moved), so `codex/` still exists.
  `mini_project/GUIDE.md` now points at it by full path.
- CAD code with hard-coded old paths (not edited): `cad/stock_plus` and `cad/stock_mounts` load the STLs
  from `C:\cde1302\Actual Mod STL of drone frame`; the `legacy_exports_untouched` checks in
  `cad/drone_cad_v*` look for `C:\cde1302\CAD\exports`; `drone_cad_v3/src/baseline.py` writes to
  `codex/drone_cad_v3/output`; `mini_project/audit/*.ps1` and `stock_analysis/analyze_stock.py`
  use the old STL and `codex` paths. Historical docs in `docs/archive/` and `cad/PROJECT_SUMMARY.md`
  still describe the old layout on purpose.
- Untracked work still uncommitted (decide: commit or ignore): `cad/drone_cad_v3/`, `cad/stock_mounts/`,
  `cad/stock_plus/`, `cad/drone_cad_v2/ASSEMBLY_AND_PRINT_ORDER.md`, `docs/archive/mini_project_mission/`,
  `mini_project/audit/`, `stock_analysis/`, `webots/`, `drone_wiring.drawio`, `FLIGHT_COMMANDS.html`,
  `PROJECT_SUMMARY.md`, `RUN_DAY_PROMPT.md`, `AGENTS.md`, `LICENSE`.
- **Step 3 (Miniforge) is left for the user to do by hand.** Its files are still in the repo root.
