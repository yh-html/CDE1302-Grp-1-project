# CDE1302 mini-project: complete guide

Written 17 Oct 2026. Everything here comes from the repo's own docs (`README.md`,
`laptop/TEST_GUIDE.md`, `mini_project/mission/DEMO_PLAN.md`) or from checks run while
building this. Things that were **not** tested are marked **UNTESTED**.

Deadlines: assignment **6 Nov 2026, 11:59 PM**. Mini-project demo **weeks 12-13**
(9 Nov is a public holiday; the demo can shift to 12 Nov). Group report due on Canvas **48 h after your demo**.
**Read section 0 first: the mini-project is a single-UAV AprilTag exploration task, not the SAFMC swarm.**

---

## 0. What the mini-project actually requires (read this first)

Source: `C:\Users\Krish Gaurav\Downloads\CDE1302 Mini Project.pdf` (not in the repo; copy it in). **It is a
single-UAV autonomous exploration task, not the SAFMC swarm/relay challenge.** The repo's `CLAUDE.md` and several
earlier agent outputs (relay planner, Webots relay sim, SAFMC rules digest) were built around the swarm framing and
mostly do not apply.

**Mission (one UAV per group):**
1. Autonomous takeoff from the assigned start, triggered by a command from the ground laptop.
2. Autonomous exploration of a designated search area with AprilTags at unknown locations. Strategy is free:
   waypoints, trajectory tracking, or other. Boundaries and the number of tags are announced beforehand.
3. For every detected tag: record its **ID, position in the UAV's local frame, relative pose and timestamp**.
4. Return to the start area and land. Final heading does not matter.
- No manual teleoperation, manual goal selection or manual target specification unless the examiner allows it.
- The Python program must **record and display timestamps** of: takeoff command sent, each tag detection (ID +
  estimated position), landing command sent.
- Python is the primary language; keep tunable parameters in config files; use standard message types and frames.

**Session:** 30 min total including setup, plus 5 min individual questions. Several trials are allowed with examiner
approval. Timing runs from the takeoff command until the tags are reported and the UAV has landed in the start area.
A fall or comms loss means emergency action, tell the examiner; restarts need approval; partial credit is possible.

**Marks:** demonstration 70% + group report 30%. Within the demonstration:

| Item | Weight |
|---|---|
| Autonomous takeoff and stable hover | 5% |
| Return to start and land | 5% |
| AprilTag detection (unique IDs correct) | 15% |
| AprilTag location accuracy (horizontal error, local frame) | 15% |
| Task completion time | 15% |
| Quality of system implementation (stability, robustness, software organisation) | 15% |
| Individual on-site question | 20% |
| Extra features (better exploration, auto recovery, replanning, **innovative mechanical design**, visualisation) | 10% |

**Group report (30%):** at most 10 pages including cover and appendix, single column, single spacing, Times New Roman
12 pt, GitHub link to the code, submitted on Canvas within 48 h of the demo. Cover: architecture and data flow,
exploration strategy, tag detection and localisation, parameters, results, AI-assisted flight-data analysis,
limitations.

**AI-assisted flight-data analysis (required):** download the PX4 `.ulg` log, combine it with the tag detections, and
plot the full trajectory, start and landing positions, and the estimated tag positions with IDs. Explain how AI was
used, where it was wrong or limited, and how you verified the results. Optional section 8 of the brief: diagnose an
abnormal flight from a `.ulg` with AI, verified experimentally.

**What this means for the work so far:**
- No obstacle or peer avoidance is required, only staying inside the boundaries. The disabled ToF/VFH firmware
  (section 8) is therefore **not a blocker**: use waypoint or trajectory exploration.
- Tag **localisation accuracy** is worth 15% and depends on the camera's pose relative to the drone. A rigid,
  measured camera mount with a known pitch matters more than frame strength.
- The frame/camera-mount work can count under **Extra features** (mechanical design), but only if it flies.
- Everything swarm-specific (`relay.py`, relay phase, Webots relay sim, SAFMC scoring) is out of scope.
- The individual questions are 20%: you must be able to explain every part of the system yourself.

---

## 1. What has been done

### Assignment (individual, minimum-snap gate trajectory) - essentially done
- `Assignment/getQ.m`, `getAbeq.m`: snap Hessian and constraints, verified
  (`results/verify_math.m`: residual 3.0e-10, KKT agreement 9.8e-9, order 6 fails).
- `Assignment/min_snap_fly_gate_task1.m`: 7th order, 2 segments through the gate
  [3.2, -2.25, -1.15], flown in the supplied Simulink model, 5-scale time study.
- Result: simulated miss **0.028 m** (limit 0.1), peak speed 2.86 m/s (limit 3),
  peak accel 3.92 m/s^2 (limit 5), scale 1.25, total 3.96 s.
- `Assignment/report/report.pdf` (5 pages) and `CDE1302_Krish_A0322900U.zip` rebuilt.
- **Still yours:** replace the `[STUDENT TO CONFIRM]` line, view the PDF, rebuild the zip.

### Mini-project hardware/software
| Work | Where | State |
|---|---|---|
| Stock frame analysis (STL holes, thicknesses, guard dims, mass) | `mini_project/stock_analysis/` | done, measured from the Profs' STLs |
| Stock frame improvements (camera cheeks, dowels, TPU skid shoes, thicker ribs) | `codex/stock_plus/` | built by the other Claude Code session: 38 pass / 0 fail / 4 pending |
| Astra (own frame redesign) | `codex/drone_cad_v2/` | **dropped** (6 failed checks, built on guessed hardware) |
| Camera offsets for under-nose mount + tests | `laptop/tag_debug.py`, `laptop/tests/test_tag_debug_mount.py` | done, but **Astra's** offsets: redo for the stock/stock_plus camera |
| Stuck-detection bug | `laptop/stuck_monitor.py` | fixed (progress measured from an anchor, not the last packet) |
| Stale PX4-status test | `laptop/tests/test_protocol.py` | skipped with reason; real packet format now tested |
| SAFMC rules digest, demo plan, risk draft | `mini_project/mission/` | drafted by an agent, **not yet checked against the booklet by you** |
| Webots planner sim | `mini_project/webots/` | ran: 2 explorers + 12 relays, 5.03 m relay; kinematic only |
| Audit scripts | `mini_project/audit/` | snapshot + audit of agent work |

### Not done / open
- Nothing has been printed, flown or flashed. All hardware sizes (motor screw depth, mass, CG, camera
  board/lens, ELRS, strap) are still **PLACEHOLDER** or **ASSUMED**.
- The root cause of the crashes, failing auto-landing, is not addressed.
- `laptop/setup.yaml` is your original (arena x 0-14, explore IDs `[6,7,14,17,19,24,26,27,30]`,
  relay `[]`). The booklet says the field is about 20 m x 20 m; compare before the demo.
- Firmware edits by the other session (`main/at_detect.c`, `main/camera_stream.c`: null checks and
  a clearer log line) now **compile**: `idf.py build` succeeded on 17 Oct (not flashed or flown).
- **The firmware has no obstacle or peer avoidance** (see section 8). The ToF/VFH stack described in
  `CLAUDE.md` was switched off by the course instructor on 30 Jul 2026.

---

## 2. How it all works

```
 Laptop (Python, laptop/)  <-- WiFi UDP -->  ESP32-S3 on each drone (firmware, main/)  <-- UART MAVLink -->  PX4 flight controller
   run_mission.py                              mavlink_task  50 Hz  PX4 telemetry/setpoints
   exploration.py  picks goals                 nav_task      20 Hz  VFH obstacle avoidance, goal nav
   crumb_store.py  breadcrumb map              tof_task      15 Hz  8x VL53L5CX ToF sensors
   stuck_monitor.py  escape goals              at_detect     ~2 Hz  AprilTag detection (never affects flight)
   relay.py  relay chain plan                  wifi_task     10 Hz  telemetry out, commands in
   comms.py  UDP send/receive
```
- Frames: everything is NED, origin at takeoff. `setup.yaml` gives each drone's start offset.
- Ports: telemetry 5005 (drone to laptop), commands 5006, ToF debug 5007, AprilTag debug 5008, camera 5009.
- IPs: drone = `192.168.1.(200 + id)`, laptop must hold `192.168.1.(100 + id)` on the drone WiFi.
- Mission: explore (laptop picks the least-explored gap) then relay (landed drones form a chain
  from the bonus victim to the Start Area).
- Camera: the XIAO ESP32-S3 camera finds tag16h5 tags. `tag_stream.py` runs the **same** C detector
  on the laptop so what you see matches what the drone decodes.
- Assignment: separate. MATLAB generates a minimum-snap polynomial, the Simulink quadrotor tracks it.

---

## 3. Start the laptop programs

The Python environment already exists: `C:\cde1302\envs\cde1302-laptop` (Python 3.12 + pytest).
Run everything from `C:\cde1302` in PowerShell with that interpreter:

```powershell
cd C:\cde1302
$py = 'C:\cde1302\envs\cde1302-laptop\python.exe'
& $py laptop/apriltag_host.py          # must print: tag16h5 ncodes = 22 (expected 22)
```

Then the tools (examples use drone **22**; use your ID):

| Goal | Command |
|---|---|
| Live camera | `& $py laptop/camera_stream.py --esp-ip 192.168.1.222` |
| Camera + AprilTags | `& $py laptop/tag_stream.py --esp-ip 192.168.1.222 --detail` |
| Link check, no flight commands | `& $py laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only` |
| Arm, take off to 0.5 m, hover, land | `& $py laptop/simple_arm_takeoff_land.py --drone-id 22` (type `ARM-22`) |
| Waypoints | `& $py laptop/simple_waypoint_mission.py --waypoints-file waypoints/waypoints_example.txt --confirm --drone-id 22` |
| Trajectory CSV | `& $py laptop/send_trajectory.py --drone-id 22 --takeoff --trajectory trajectory/circle_traj.csv --confirm` |
| **Mini-project mission (use this)** | `& $py laptop/run_tag_mission.py --drone-id 22` (see "Tag mission" below) |
| Swarm mission (explore + relay), **out of scope** | `& $py laptop/run_mission.py --config laptop/setup.yaml` |

### Tag mission (written 17 Oct, matches the mini-project brief)

`laptop/run_tag_mission.py`: takeoff command, lawn-mower sweep of the search area, log each tag (ID, local-frame
position, camera-frame pose, timestamp), return to start, landing command. Parameters are in
`laptop/tag_mission.yaml` (search area, lane spacing, camera geometry, timeouts, geofence).

```powershell
& $py laptop/run_tag_mission.py --drone-id 22 --plan-only     # print the waypoints, no connection
& $py laptop/run_tag_mission.py --drone-id 22 --simulate      # full dry run in the built-in simulator
& $py laptop/run_tag_mission.py --drone-id 22                 # real drone; type TAKEOFF-22 to send takeoff
```
Output goes to `laptop/mission_logs/<time>_drone<ID>/`: `events.csv` (takeoff, every detection, landing, with
timestamps), `observations.csv` (every sighting and the numbers behind it), `tags.csv` (final positions),
`trajectory.csv` (drone path, for the `.ulg` plot), `summary.json`. Events and a final tag table also print on screen.

Safety: it lands on stale telemetry (3 s), leaving the search area plus 0.5 m, Ctrl+C, or an error. It stops
searching early (then returns and lands) on the time limit, when `expected_tags` are found (if
`stop_when_all_found`), or after 3 consecutive waypoint timeouts.

**Before the demo you must set** (all marked PLACEHOLDER/ASSUMED in the yaml): the announced `search_area` and
`expected_tags`; the real `camera` pitch and offsets (the defaults are the old Astra numbers); `lane_spacing_m`
and `max_range_m` from a camera test; and `latency_s`.

Verified so far: 173 laptop tests pass, including a full mission in the simulator and one over real UDP sockets
against a fake drone using the firmware wire format. **Not verified:** any run on a real drone, the camera
geometry, tag localisation accuracy, or the detection latency.

Known limits:
- **Speed (corrected 17 Oct).** The firmware constant `NAV_CRUISE_SPEED_MS` (0.10) is **not used for control**
  (it only appears in a log line). Waypoint mode sends the goal as a position setpoint, so the real speed is set by
  PX4's `MPC_XY_VEL_MAX` on the flight controller; `waypoints/WAYPOINT_FLIGHT_TEST_GUIDE.md` says about 0.2 m/s. I
  have not read the parameter from your drone. The default 4 x 4 m sweep is about 150 s at 0.2 m/s (282 s at 0.1,
  72 s at 0.5): `run_tag_mission.py --plan-only` prints the table. Completion time is 15% of the marks.
  Order of work: (1) fly once and read "max horizontal speed" from `analyze_flight.py` to learn the real speed;
  (2) if you want it faster, raise `MPC_XY_VEL_MAX` in QGroundControl (Vehicle Setup, Parameters; no reflash) in small
  steps, checking optical-flow hold and tag detection at each step (motion blur and pose lag grow with speed; set
  `detection.latency_s`); (3) or fly a trajectory with `send_trajectory.py`. Check with your instructor before
  raising PX4 limits. I have not verified how your PX4 version applies the limit in offboard mode.
- **Lane spacing** comes from the camera: `--plan-only` prints how far to the side a floor tag can be seen
  (about 0.66 m each side at 0.5 m altitude, 45 deg tilt, assumed 30 deg minimum viewing angle and 3 m range) and a
  suggested spacing (about 1.06 m). The viewing-angle and range limits are ASSUMED: test them with a real tag, and
  the calculation assumes tags lie flat on the floor, which the brief does not say.
- **Pose.** The tag packet carries translation only, so "relative pose" is the camera-frame position (no rotation).
- **Roll and pitch** of the drone are not in telemetry and are ignored, so tilt adds localisation error.
- **Landing** is the command plus a wait; telemetry has no touchdown signal.
- The firmware has no obstacle avoidance. Keep the area clear.

### Post-flight analysis (the brief's AI-assisted `.ulg` requirement)

`laptop/analyze_flight.py` combines the PX4 `.ulg` with the mission run folder and writes `flight_overview.png`
(full path, start, landing, tag IDs and positions, altitude and speed), optional `flight_3d.png`,
`flight_summary.md` and `flight_summary.json` into `<run>/analysis/`.

```powershell
& $py laptop/analyze_flight.py --ulg C:\path\to\flight.ulg --run laptop\mission_logs\<run folder>
& $py laptop/analyze_flight.py --ulg flight.ulg --run <run> --truth 3:1.20,-0.40 --truth 5:2.60,0.85   # known tag positions -> error
& $py laptop/analyze_flight.py --run <run>        # telemetry only, before the .ulg is downloaded
```
Get the `.ulg` from the flight controller's SD card (`log/` folder) or QGroundControl, Analyze Tools, Log Download.
The first time, install the new dependency: `& $py -m pip install -r laptop/requirements.txt` (adds `pyulog`).

It aligns the two clocks itself (PX4 time since boot vs the mission's wall clock) by matching the drone path, and
prints how good the match is. If it says `ambiguous`, do not trust the event markers: pass `--offset-s`. It also
re-localises each tag using the logged roll and pitch and shows how much the mission's heading-only geometry
shifted it (the `attitude-corrected` column), and it computes the error against `--truth` positions if you
measure the tags with a tape.

**Verified:** 13 tests on synthetic logs written in the real ULog binary format, a real simulated mission, the
sync, the maths and the outputs. **Not verified: any real `.ulg` from your drone.** Fields it reads:
`vehicle_local_position` (x, y, z, vx, vy, vz), `vehicle_status.arming_state` (2 = armed), `vehicle_attitude.q`,
`vehicle_land_detected.landed`. If a real log differs, the loader says which topic is missing.
For the report, explain how AI helped, where it was wrong, and how you checked it (the summary ends with a checklist).

### Group report skeleton (written 17 Oct)

`mini_project/report/CDE1302_Mini_Project_Report.docx`: a Word skeleton following the brief's required sections
(architecture and data flow, exploration strategy, detection and localisation, parameters, results, AI-assisted
analysis, limitations, appendix). Yellow `[brackets]` are placeholders for the group; the rest is a draft written from
the code, so check each statement against your own drone. It is 5 pages with placeholders, leaving about 5 for results
and figures. `check_report.ps1` opens it in Word and checks the brief's format rules (at most 10 pages, single column,
single spacing, Times New Roman at least 12 pt, GitHub link present):

```powershell
cd C:\cde1302\mini_project\report
powershell -ExecutionPolicy Bypass -File .\check_report.ps1 -Pdf       # also writes a PDF preview
```
Do not re-run `build_report.py` once the group is editing the .docx (it overwrites the file); it exists to
regenerate the skeleton and the architecture figure (`make_architecture_fig.py`). The report is due on Canvas within
48 h of your demonstration, and the brief wants a GitHub link to the source code: the code written here is still
uncommitted in your working copy, so commit and push it to whichever repository you will link.

Network rules (from the README): the laptop must hold `192.168.1.(100+id)` on the drone WiFi
(gateway 192.168.1.1, mask 255.255.255.0); the WiFi must allow client-to-client traffic; allow Python
through Windows Defender Firewall (UDP 5005, 5008, 5009) and set the network to *Private*; do not use
WSL (it drops the drones' packets). `tag_debug.py` takes UDP 5005, so start it only after the flight script.

---

## 4. Flash a drone

**You may not need to.** The README says the ESP32 firmware is already built and flashed. The
laptop-side changes (camera offsets, stuck monitor) need no flashing. Only flash if you want the
two small null-check edits in `main/`, or a different drone ID.

**ESP-IDF is installed on this laptop** (17 Oct) at `C:\esp-idf`, on the pinned commit
(`08e0d30a74ad0bfd5a34933142b80f45619ee410`, describe `v6.1-dev-6940-g08e0d30a74a`), with the
ESP32-S3 toolchain in `C:\Users\Krish Gaurav\.espressif`. `idf.py build` produced
`build\esp-everything.bin` (1.06 MB, 31% of the app partition free). **Flashing has not been tried.**

To use it in any new PowerShell window:
```powershell
. C:\esp-idf\export.ps1
cd C:\cde1302
```
To redo the install from scratch: `git config --global core.longpaths true`, then
`git clone https://github.com/espressif/esp-idf.git C:\esp-idf`, `git -C C:\esp-idf checkout <pin>`,
`git -C C:\esp-idf submodule update --init --recursive`, then `C:\esp-idf\install.ps1 esp32s3`.

Per drone (this is what `flash_drone.sh` automates; that script is bash and sources `export.sh`, so it is meant for
macOS/Linux or Git Bash, **UNTESTED** here):
   ```powershell
   . C:\esp-idf\export.ps1      # once per window
   cd C:\cde1302
   idf.py menuconfig      # Drone Configuration: DRONE_ID, WIFI_SSID/PASSWORD, host IP 192.168.1.(100+id),
                          # drone IP 192.168.1.(200+id), gateway 192.168.1.1, netmask 255.255.255.0
   idf.py build
   idf.py -p COM5 flash monitor     # COM port from Device Manager
   git checkout -- sdkconfig        # sdkconfig is tracked; restore it so the repo stays clean
   ```
   Default SSID is `safmc-wifi`. Drone ID range is 0-30.
4. Open the serial monitor **before** arming. Wait for `Waiting for CMD_START from laptop...`.

---

## 5. Confirm the code works (cheapest first)

1. **Laptop tests** (expect 121 passed, 1 skipped, about 3 s):
   ```powershell
   cd C:\cde1302
   & 'C:\cde1302\envs\cde1302-laptop\python.exe' -m pytest laptop/tests -q
   ```
2. **Assignment:** `cd C:\cde1302\Assignment`, then
   `& 'C:\Program Files\MATLAB\R2025b\bin\matlab.exe' -batch "run('results/verify_math.m')"`
   (4 PASS lines) and `... -batch "run('min_snap_fly_gate_task1.m')"` (about 2 min; ends `gate requirement met: true`).
3. **Audit of the agents' work:**
   ```powershell
   cd C:\cde1302\mini_project\audit
   .\audit.ps1 -Reproduce
   ```
   Expect FAILs only for files the other Claude Code session changes outside the old lanes
   (`codex\stock_plus`, new folders). Read `audit_report.md` and judge each one.
4. **Firmware compiles** (section 4): `idf.py build`. Passed on 17 Oct (exit 0). The only warning in our code is
   the unused `inject_peers_into_histogram` (see section 8).
5. **Hardware ladder, never skipping a rung** (from `laptop/TEST_GUIDE.md` and `DEMO_PLAN.md`):
   1. camera on the bench: `tag_stream.py`, check image orientation (the firmware sets `vflip`), tag IDs, range;
   2. `--monitor-only`: expect `Connected: drone=22 ...` and `age` well below 1 s;
   3. **props off**: run the arm script, read the ESP32 log (`OFFBOARD mode confirmed`, `Armed confirmed`, `Altitude reached`);
   4. props on, restrained/low hover, then `CMD_LAND` repeatedly (auto-landing is the known failure);
   5. one drone navigating a clear area, then add drones one at a time.
   Always have a tested kill path and no people in the flight area while armed.

---

## 6. Recreate the work

| Item | Command |
|---|---|
| Assignment results and figures | `MATLAB -batch "run('min_snap_fly_gate_task1.m')"` in `C:\cde1302\Assignment` |
| Report | `cd Assignment\report; latexmk -pdf report.tex` |
| Submission zip | `cd Assignment; powershell -ExecutionPolicy Bypass -File .\make_submission.ps1` |
| Stock STL analysis | `& 'C:\cde1302\codex\drone_cad\.venv\Scripts\python.exe' mini_project\stock_analysis\analyze_stock.py` |
| Stock frame improvements | `cd codex\stock_plus\src; ..\..\drone_cad\.venv\Scripts\python.exe -u build.py` (rib step about 15 min; cached) |
| Webots sim | `python mini_project\webots\build_world.py`, then run `webots.exe --batch --mode=fast --port=1247 mini_project\webots\worlds\safmc_explore_relay.wbt` (see `mini_project/webots/AGENT_REPORT.md`) |
| Audit baseline | `mini_project\audit\snapshot.ps1` (once, before agents), then `audit.ps1` |
| Parts to print | `Actual Mod STL of drone frame\` (frame x1, panel x1, leg x2 mirrored, guard x4 rotated 90 deg each) or the `codex\stock_plus\output\stl` versions |

---

## 7. Next steps, in order

**This week**
1. Assignment: edit the `[STUDENT TO CONFIRM]` line, open the PDF, rebuild the zip, submit before 6 Nov.
2. Decide stock or stock_plus. Review `codex/stock_plus/README.md` and its 4 pending items before printing.
3. Print **one guard** first and check it on a motor and a prop. Then frame, panel, legs.
4. Measure and record: parts and all-up mass, CG, motor screw depth, camera board/lens position.
5. Point `laptop/tag_debug.py` at the real camera position (it still has Astra's under-nose offsets).
6. Compare `laptop/setup.yaml` arena against `mini_project/mission/RULES_DIGEST.md`
   (field about 20 m x 20 m, Start Area 20 x 6 m, max height 1.4 m).
7. Read `RULES_DIGEST.md`, `DEMO_PLAN.md`, `RISK_ASSESSMENT_DRAFT.md` against the SAFMC booklet yourself.

**Weeks 9-11 (mini-project preparation)**
8. Get ESP-IDF working (or use a lab machine); build; flash one drone; props-off test.
9. Tackle the real crash cause: auto-landing. Test `CMD_LAND` at low altitude over foam first.
10. Fly one drone through `--monitor-only`, hover, waypoints, then `run_mission.py` with one explorer.
11. Add drones one at a time, then rehearse the two-run procedure.
12. Complete the course Risk Assessment (`Labs/Risk Assessment/...xls`) from the draft.

**Weeks 12-13**
13. Demo/assessment. Carry spares: guards, panel, legs, props, screws, batteries, tape, ties.

---

## 8. Known gaps and cautions
- **No obstacle or peer avoidance in the current firmware (checked 17 Oct).** The instructor's commit
  `ba67fbc` (30 Jul 2026, "disable TOF and also limit the cruise speed to 0.1m/s") switched it off:
  `TOF_ENABLED` is 0 (`main/tof_task.h`), `collision_avoid()` returns false immediately, `nav_tick()` no longer
  runs VFH (`blocked[]` is all false), `inject_peers_into_histogram()` is never called, and
  `NAV_CRUISE_SPEED_MS` was cut to 0.10 (but is unused, see "Speed" above). The laptop still broadcasts peer positions (`CMD_SET_PEERS`) and the firmware
  stores them, but nothing reads them. `CLAUDE.md` describes the old behaviour and is out of date.
  Consequences: the VFH exploration and peer avoidance described in `CLAUDE.md` cannot work on the course drones as
  flashed (commit `a7cb948` describes `TOF_ENABLED=0` as the setting for when the ToF ring and mux are not fitted; I
  have not confirmed what hardware your drone has), and drones will not avoid each other or walls. The Webots sim and the demo plan assume the old behaviour, so treat them as planner-only.
  **Resolved by the real brief (section 0):** the mini-project is a single UAV that only has to stay inside the
  boundaries, so use waypoint/trajectory exploration. Re-enabling ToF/VFH is not required.
- Hovering drones that wait for a goal can be flagged stuck after `stuck_timeout_s` (10 s) if they move less than
  0.5 m. Check whether `run_mission.py` ever has drones wait.
- Webots is kinematic: it does not model the firmware VFH, PX4, real dynamics or ToF noise.
- Agent-written documents are first drafts. The audit confirms files and numbers, not engineering judgement.
- Print strength, camera view and flight behaviour are unproven until you test them.
- The AI policy for the assignment: keep the declaration current (Codex and Claude Code) and add your own verification statement.
