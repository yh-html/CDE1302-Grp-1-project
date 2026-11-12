# CDE1302 Mini Project: team summary for the group report

Written 12 Nov 2026, after the graded run. The group report is due **Mon 16 Nov 2026, 23:59** (Canvas, 10 pages maximum including the cover and appendix, Times New Roman 12 pt, single spacing, with the GitHub link).

This document is for teammates who did not follow the day-to-day engineering. It covers what we built, how it works, what went wrong and how each problem was fixed, and how the graded run went. It also says where the evidence for each point lives. Every number comes from a file in the repo or from the graded-run logs. Anything we inferred rather than measured is marked **(inferred)**.

Paths are relative to the repo root `C:\cde1302` unless they start with `C:\`. Some course material (videos, the brief PDFs, the raw flight-map folder) was moved out of the repo to `C:\cde1302_course\` on 12 Nov. Those paths are given in full.

---

## 1. Overview and headline result

We built a single-UAV autonomous AprilTag search system (drone ID 1). The ground laptop sends a takeoff command. The drone then flies a smooth two-lane sweep of the 3.6 m x 3.15 m search area at 0.6 m height, with a downward-looking camera. Each AprilTag it sees is detected on the drone's ESP32-S3, and its position is estimated in the drone's local frame on the laptop. A tag counts as found only after it is seen in 3 separate camera frames. When all expected tags are found, the drone returns to the start along an uploaded trajectory and lands. The laptop prints and logs the timestamps of the takeoff command, each tag detection and the landing command, as the brief requires.

**Graded run, Thu 12 Nov 2026 (16:40 to 16:54): 8 of 9 tags over three runs (3/3, 3/3, 2/3).** All three runs took off, finished and sent the landing command near the start (0.02 to 0.17 m from it in the position estimate). Each took 148 to 151 s from the takeoff command to the end of the mission. Over the runs where a tag was found, its estimate repeated within 0.06 to 0.13 m. The one exception is run 3, where position-estimate drift moved ID 6 by about 0.6 m and caused ID 2 to be missed **(inferred)**. One extra attempt between runs 2 and 3 was stopped by the soft fence and then the hard geofence after 21 s, with an emergency landing, and was flown again.

Getting there took two full days of flight testing (10 and 11 Nov, about 50 logged mission attempts in total). It exposed five real problems: an EKF height and position reset caused by a foam block, optical-flow drift, motion blur, comms gaps, and heading-frame errors from having no compass. Section 4 tells that story, with the cause and the fix for each.

---

## 2. The system

### 2.1 Hardware (drone 1)

| Item | What we used | Source |
|---|---|---|
| Flight controller | MicoAir743v2-AIO-45A (STM32H743, BMI088 + BMI270 IMUs, SPL06 barometer, **no magnetometer**, 4 x 45 A AM32 ESC), running PX4 | memory note on FC wiring; `C:\cde1302_course\datasheets\micoair\` |
| Companion computer | Seeed XIAO ESP32-S3 Sense with its camera (the README names an OV3660) | `README.md`, `main/` |
| Position sensing | MTF-02P optical-flow + rangefinder module, facing down; PX4 EKF2 fuses flow + range (+ baro from 10 Nov) | `mini_project/DRONE_BRINGUP_GUIDE.md` sec. 1; `mini_project/px4_params_drone1_2026-11-10.txt` |
| Frame | Course stock frame and printed mounts (the team dropped its custom frames on 29 Oct, commit `b95fa9b`) | git log; section 2.6 |
| Camera mount | Looks **straight down** (pitch 90 deg) since 11 Nov; it was tilted 45 deg forward before that | commit `016853e`; `laptop/tag_mission_demo_traj.yaml` (`camera:`) |
| Link | ESP32 UART1 (TX GPIO5, RX GPIO4) at 921600 baud to the FC, MAVLink; Wi-Fi UDP to the laptop (laptop 192.168.1.101, ESP32 192.168.1.201) | memory note on FC wiring; `laptop/tag_mission_demo_traj.yaml`; graded-run screenshots |
| RC | ELRS receiver; the safety pilot holds the kill switch and can take over in Position mode (`COM_RC_OVERRIDE=1`) | `mini_project/DRONE_BRINGUP_GUIDE.md`; params file |

### 2.2 Software architecture and data flow

```
                        Wi-Fi UDP                                  UART 921600, MAVLink
  LAPTOP (Python)  <-------------------------->  ESP32-S3 (FreeRTOS, C)  <---------------->  PX4 (MicoAir743v2)
                                                                                              EKF2: optical flow +
  run_tag_mission.py                             mavlink_task: telemetry in,                   rangefinder + baro,
   └ tag_mission.py   (state machine)                          setpoints out (50 Hz)           no compass
       ├ mission_config.py  <- tag_mission_demo_traj.yaml
       ├ sweep_planner.py + traj_gen.py ---- trajectory upload (1279 pts) --> nav_task: plays the trajectory
       │                                       (CMD port 5006)                 on its own clock, holds heading
       ├ drone_link.py (UdpLink / SimLink) <-- telemetry 10 Hz (port 5005) --- wifi_task
       ├ tag_localizer.py  <---------------- AprilTag detections (port 5008) -- at_detect_task (tag16h5, ~2 Hz)
       ├ mission_log.py  -> laptop/mission_logs/<time>_drone1/*.csv, summary.json
  live_map.py   (reads the CSVs only; optional camera panel from port 5009) <-- camera_stream_task (JPEG)
  analyze_flight.py + flight_data.py + flight_plots.py  (after the flight: .ulg + mission log -> figures)
```

A ready-made architecture figure exists: `mini_project/report/fig_architecture.png` (made by `mini_project/report/make_architecture_fig.py`). Check that it still matches the downward camera and trajectory mode before using it.

**Control flow:**
1. The operator types `TAKEOFF-1`. The laptop sends the takeoff command, and the ESP32 switches PX4 to Offboard, arms and climbs to 0.5 m (`CRUISE_ALT_M`, `main/main.c`).
2. During the 12 s takeoff wait, the laptop pre-uploads the whole sweep trajectory, twice.
3. It then sends START. The ESP32 plays the path as position setpoints with the heading held fixed (`traj_tick()` in `main/nav_task.c`).
4. Detections stream to the laptop, which localises and confirms the tags.
5. When all tags are found, the laptop holds for 4 s, uploads a return trajectory, waits until the drone is home, and sends LAND.

**Safety layers** (all in `laptop/tag_mission.py`, values in the YAML):
- Telemetry older than 3 s: land.
- Soft fence, 0.3 m outside the search area: hold, fly home, land.
- Hard geofence, 0.45 m outside the search area (the PDF flight boundary) and never closer than 0.5 m to the measured walls: emergency land.
- Time limit of 600 s.
- Optional start-heading check.
- The RC pilot can take over or kill at any time.

### 2.3 Key parameters (as flown on 12 Nov, `laptop/tag_mission_demo_traj.yaml`)

| Parameter | Value | Why |
|---|---|---|
| Search area (local frame, x forward, y right, origin at takeoff) | x 0.9 to 4.5 m, y -1.8 to 1.85 m | Flying Area PDF; the right edge was widened on 11 Nov |
| Lanes | 2 lanes along x, at y = -1.35 and +1.40 (spacing 2.75 m, edge margin 0.45 m) | One lane over each row of tag spots; the right row was moved out after 11 Nov flights |
| Sweep entry / return | `entry: left`, `return_leg: lateral` (square legs, no diagonals) | Keeps the legs away from the walls and inside the fences |
| Takeoff / sweep height | 0.5 m (firmware) / 0.6 m (trajectory climbs to it) | 0.8 m was wobblier (11 Nov 20:54) |
| Trajectory speed | 0.144 m/s peak; 0.06 m/s within +-0.3 m of each of the 6 tag spots; accel 0.08 m/s^2; corner radius 0.35 m; dt 0.1 s | 0.20 m/s blurred the tags (10 Nov 19:18) |
| Planned path | 1279 points, 127.8 s for the full sweep including return (events.csv) | |
| Detection | `min_frames: 3`, `merge_radius_m: 0.5`, `id_max: 21`, `expected_tags: 3`, `settle_s: 4`, `stop_when_all_found: true` | Misread rejection; early return saves time |
| Firmware gate | hamming <= 1 and decision margin > 55; tag size 0.12 m; QVGA frames; `quad_decimate 1.5` | `main/at_detect.c` |
| Laptop rescue | keep hamming <= 3 if margin >= 70 (localised from pixel centre + altitude) | `rescue_margin` in the YAML |
| Camera geometry | pitch 90 deg, fwd 0.070 m, right 0.0 m, down 0.008 m, FOV 91 x 82 deg | **Offsets are CAD placeholders, not measured** (YAML comment) |
| Fences | soft 0.3 m, hard 0.45 m outside the area, wall clearance 0.5 m, stale timeout 3 s | |
| `max_start_heading_deg` | 0 (check off) | It refused takeoff with the drone square in the box (11 Nov) |
| `look_again` | disabled | It misfired on 10 Nov 19:48 |

PX4 parameters that mattered (`mini_project/px4_params_drone1_2026-11-10.txt`, read on 10 Nov **before** the change):

| Parameter | Value |
|---|---|
| `EKF2_HGT_REF` | 2 (height from the rangefinder) |
| `EKF2_BARO_CTRL` | 0, changed to **1** on 10 Nov |
| `EKF2_RNG_NOISE` | 0.02, changed to **0.05** on 10 Nov |
| `EKF2_MAG_TYPE` | 5 (no compass) |
| `EKF2_OF_CTRL` | 1 |
| `MPC_XY_VEL_MAX` | 0.4 |
| `MPC_XY_CRUISE` | 0.2 |
| `COM_RC_OVERRIDE` | 1 |
| `SDLOG_MODE` | 0 |

### 2.4 Software quality evidence

- Every tunable value is in one YAML file, loaded and validated by `laptop/mission_config.py`, as the brief's "Expected Engineering Practice" asks.
- The same mission code runs against the real drone (`UdpLink`) or a kinematic simulator (`SimLink`) in `laptop/drone_link.py`. The simulator models boxed tags, drift and decode probability (commit `9719e1d`).
- `laptop/tests/`: 18 test files, 211 `def test_` functions by count. The run-day notes (`mini_project/RUN_DAY_PROMPT.md`) say "248 tests pass", which probably counts parametrised cases; re-run `pytest laptop/tests` for the exact figure.
- `laptop/live_map.py` shows the live map seen in every screenshot: search area, fences, planned and flown path, sightings and tag estimates, with an optional camera panel.
- `run_tag_mission.py --detect-only` is a props-off walk-through mode, `--plan-only` checks the plan, `--simulate` runs a dry run, and `--wait-for-go` hovers until the operator types GO.

### 2.5 Simulation work

| What | Result | Evidence |
|---|---|---|
| Simulink: sweep through the course quadrotor model `PosControl_Sim_RPT.slx` (29 Oct, older 8-waypoint stop-and-go plan, 45 deg camera) | 123.9 s takeoff to landing command; max / RMS horizontal tracking error 0.012 / 0.003 m; max tilt 4.4 deg; all 6 tag spots in view for 3.7 to 6.4 s each | `mini_project/simulink_sweep/README.md`, `mini_project/simulink_sweep/results/` |
| Python simulator: 2-lane plan on the PDF rows | Found all 3 tags for all 20 spot combinations (5 lanes: 20/20 but 2x longer; inward spiral: 4/20) | commit `42e6aac` message |
| Boxed tag (0.15 m foam walls): fly over vs detour | Over: 34/36; detour: 26/36 | commits `9719e1d`, `0e0ecd5` |
| Look-again feature | 23/30 to 27/30 in simulation (later disabled after a real misfire) | commit `9719e1d` |

The Simulink check covers the course controller's dynamics only. It does not model PX4, the ESP32, the camera or optical flow (README).

### 2.6 Mechanical design history (brief)

- **10-11 Oct:** a custom frame in Fusion 360 (rev 1, then rev 2 with a tube truss, about 245 g), and a CadQuery frame from Codex (`cad/drone_cad_v1`, `cad/drone_cad_v2`). The v2 PETG prop guards **broke at the forks and the spoke-to-ring joints**. They were redesigned as rev B with five keel spokes and a thicker ring.
- **17 Oct:** `cad/stock_plus`, with boolean improvements to the instructor's stock STLs: camera cheeks, a thicker pad ligament, dowel holes and TPU skid shoes. It prints at 79.9 g against 68.5 g for the stock parts. The stock geometry analysis is in `mini_project/stock_analysis/stock_summary.md`.
- **25-26 Oct:** `cad/drone_cad_v3`, a lighter hollow-box-arm frame (printed set 63.8 g). The rev D guard **broke at the ring when the slicer supports were removed**, and rev E strengthened it (76.2 g, heavier than stock).
- **29 Oct:** the team went back to the stock frame (commit `b95fa9b`). Which printed mounts were actually flown is not recorded (see section 8).
- **11 Nov:** a 3D-printed flow-sensor casing was the suspected cause of the optical-flow drift (section 4). The original casing went back on, and the drift stopped.

This can support a short "extra features / mechanical design" paragraph, but none of the custom frames was flown in the graded run.

---

## 3. Mission design

### 3.1 Search strategy

`laptop/sweep_planner.py` builds a boustrophedon (lawn-mower) sweep. Using `lane_spacing_m: 2.75`, it makes two lanes, one straight down each row of tag spots. `laptop/traj_gen.py` turns the waypoints into a single smooth trajectory: corners are rounded with quadratic Bezier fillets, and a speed profile starts and ends at rest, never exceeds the peak speed, slows in curves, and drops to 0.06 m/s over each of the 6 possible tag spots.

The firmware plays the trajectory as position setpoints while holding the heading. The drone therefore never stops at a waypoint and never turns, which keeps the downward camera steady and the optical flow undisturbed.

Path for the graded config:
1. Sideways left from the start to y = -1.35.
2. Up the left lane to x of about 4.0.
3. Across to y = +1.40.
4. Down the right lane.
5. Sideways home.

Why this design:
- The brief allows waypoints or trajectories. Tags are on a 3 x 2 grid of marked spots in the Flying Area PDF.
- Two lanes over the rows found all 3 tags in every simulated combination, in about half the time of 5 lanes.
- A full-coverage 5-lane plan is a config change (`lane_spacing_m: 0.8`) if tags can be anywhere.

### 3.2 Detection

- The ESP32 runs AprilTag 3.4.5 (`components/esp-apriltag`, tag16h5) on QVGA frames at about 2 Hz. Frame timestamps in the graded logs are about 0.5 to 0.65 s apart.
- It sends a packet for every candidate on UDP 5008. A pose (camera-frame translation) is attached only when the candidate passes the gate: hamming <= 1 and margin > 55.
- The laptop then drops IDs above 21 and rescues high-margin candidates that the gate dropped.

Most raw candidates are noise: run 1's `observations.csv` has 625 rows, of which 280 are accepted sightings of the real tags. A `TagTracker` therefore counts *distinct camera frames*:
- A tag is confirmed after 3 frames.
- An ID seen within 0.5 m of a stronger tag is treated as a misread of it.

This came from a real failure: on 10 Nov, a single misread of tag 2 as "14" ended the sweep early (commit `06b709b`).

### 3.3 Localisation

`laptop/tag_localizer.py` turns each sighting into a local-frame position:

```
p_body  = R_y(pitch) * p_cam + t_cam            (camera -> body; pitch = 90 deg, t_cam = (0.070, 0, 0.008) m)
p_local = p_drone(t_frame) + R_z(heading) * [p_body,x ; p_body,y]
```

- `p_drone` is the PX4 position estimate from telemetry, interpolated to the frame time.
- Roll and pitch are not in the telemetry, so they are ignored (small at 0.144 m/s).
- Sightings are fused with a **1/range^2 weighted mean**, because nearer sightings are more accurate. The reported `spread_m` is the weighted standard deviation.
- The "relative pose" logged for each detection is the camera-frame translation `cam_xyz_m`.

**Important limitation for the report:** a tag estimate can never be better than the drone's own position estimate. With optical flow and no compass, PX4's position can drift without PX4 noticing (sections 4 and 5).

### 3.4 Return home and landing

When all expected tags are confirmed, the drone hovers for `settle_s` (4 s) and then flies a **return trajectory** with the heading held, via the corner (0, 1.35) (`RETURN_TRAJECTORY` in `events.csv`). Once it is within the arrival radius of the start, the laptop sends LAND. If all tags are not found, the full sweep (which ends at home) runs to its end, and LAND follows.

The return used to be a `CMD_GOTO`. That was replaced on 11 Nov because the firmware's goto rotates the drone before flying (commit `c17032d`).

There is no touchdown signal: "landed" means the LAND command plus `land_wait_s` (8 s). That is why `MISSION_END` is 8 s after `LANDING_CMD_SENT` in every log.

### 3.5 Timestamp output (brief section 4.1)

The program prints, and writes to `events.csv`:
- `TAKEOFF_CMD_SENT`;
- `TAG_DETECTED`, with the ID, x and y, first-seen time, frames, range and camera-frame pose `cam_xyz_m`;
- `LANDING_CMD_SENT`.

It also writes `summary.json`, `tags.csv`, `observations.csv` (every sighting with every intermediate value) and `trajectory.csv` (about 5 Hz). Each graded run has these in `laptop/mission_logs/<run>/`.

---

## 4. Timeline: development, testing, failures and fixes

This is the most important section for the report's "experimental results" and "limitations / failure cases" parts, and for the optional "AI-assisted diagnosis of abnormal flight behaviour" (brief section 8).

### 4.1 Before the arena (Jul to early Nov)

| Date | What happened | Evidence |
|---|---|---|
| Jul to Sep | Base firmware and laptop tools: MAVLink bridge, waypoint tests, camera streaming, laptop-side tag detection, minimum-snap trajectory flying (circle and ellipse CSVs). ToF and VFH were disabled for the single-drone task (`ba67fbc`). Precision landing was removed so the camera never affects flight (`ad0def9`). | `git log`; `trajectory/README.md` |
| 17 Oct | The real brief was found: single UAV, AprilTag search. The earlier SAFMC swarm/relay scope (`CLAUDE.md`, `mini_project/webots/`, `mini_project/mission/`) was dropped. | memory note on the brief; `mini_project/GUIDE.md` sec. 0 |
| 18 Oct | Single-UAV tag mission, flight analysis and report skeleton added (`c5439db`); camera-tag relative pose (`d6bbdae`) | git log |
| 29 Oct | `--detect-only` walk-through and `--truth` error report; real arena config; Simulink sweep sim; back to the stock frame | commits `c9f3776`, `c048543`, `b95fa9b` |
| 3-6 Nov | First flight tests. On 6 Nov (practice room, 17 logged runs): telemetry-stale and geofence aborts (e.g. geofence breach at (-0.68, 0.25) 13.7 s after takeoff, so the drone drifted backwards on takeoff), then completed runs. The smooth slow trajectory mode, tag rescue and the soft fence were added (`55fd6a2`). | `laptop/mission_logs/20261106_*`; `mini_project/DRONE_BRINGUP_GUIDE.md` |
| 10 Nov, before flying | The ESP32 would not talk to the FC on battery: **the soldered XIAO GND to FC GND wire had come off** (ping worked on USB power only). Resoldered. The drone is ID 1, not 22 as the guides assumed. | memory note on FC wiring |

### 4.2 Tue 10 Nov: first full day in the real arena (Studio 1)

21 mission runs are logged. The full notes are in `mini_project/flight_test_2026-11-10/NOTES.md`.

| Time | Result | Root cause | Fix |
|---|---|---|---|
| 15:22 | Geofence breach at (-0.82, -0.05) after 12 s | Drone drifted backwards after takeoff | Fences re-derived from measured walls (`b751ea1`) |
| 15:29 | "3 tags" found, but one was ID 14 = **a misread of tag 2**. The mission stopped early and skipped the lane with the real third tag. | A single bad decode counted as a tag | **3-frame confirmation + 0.5 m merge radius** (`06b709b`) |
| 16:09, 16:10 | Soft-fence trips at x = -0.18 / -0.16 on the sideways leg | The legs run along x = 0, so a small backwards drift crosses a 0.15 m soft fence | Soft fence widened to 0.3 m (YAML comment) |
| 16:12-16:25 | 3/3, 2/3, then a geofence breach at (-1.30, 1.00) ("lost track near (2.8, 0.2)") | 4-lane sweep on approximate spots | 2-lane path on the PDF tag rows, slow zones at the spots (`42e6aac`) |
| 17:19, 17:20 | "Drone did not start the trajectory", refused 5 times (1346-point upload) | Chunk loss suspected; the bench test showed uploads mostly complete (60 ms gap 4/5, 25 ms 5/5), so not fully explained. Later found: the **firmware ignores START until it is flying**, and mid-flight uploads lose chunks. | Chunks 60 ms apart, upload logged on the ESP32 (`0347d58`); **pre-upload during the takeoff wait, re-send START** (`9719e1d`, `9bb07b7`). Map: `mini_project/flight_test_2026-11-10/flight_maps/FAILED TRAG.png`, `failed TRAG 2.png`; bench log `laptop/mission_logs/traj_upload_test.txt` |
| 17:47 | Tags 2 and 4; soft-fence trip on the sideways leg home (x = -0.18) | Same backwards drift | 0.3 m soft fence |
| **17:50** | **Geofence abort: EKF position reset over the foam block** | Height came from the rangefinder only (`EKF2_HGT_REF=2`, baro off). Over the 0.15 m foam block boxing a tag, the height estimate fell from 0.52 to 0.33 m, so the drone climbed. Past the block, the range data were rejected (test ratio > 1 at 109.3-111.2 s). With no baro, the height drifted to about 1.06 m. At 111.4 s, EKF2 reset xy by (-0.96, +1.13) m, which tripped the laptop geofence. | **`EKF2_BARO_CTRL=1`, `EKF2_RNG_NOISE` 0.02 to 0.05** (set over USB). A hover and a manual pass over the block were clean. A detour round the spots was added, then removed once the fix held (`9719e1d`, `0e0ecd5`). PX4 log `mini_project/flight_test_2026-11-10/px4_logs/log67_1750_block_reset.ulg`; figure `mini_project/flight_test_2026-11-10/analysis/reset_1750_block.png` |
| 18:19 | Tags 2, 4 (tag 6, boxed, seen in 2 frames only) | Boxed tag visible only from about 0.15-0.30 m short of it, in line | Slow zones at the spots |
| 18:23 | **3/3**, mission end 130 s | | |
| **19:01** | **3/3, landing command at 116.5 s, mission end 124.5 s (best run of the day)**; PX4 baro on | Code = commit `0e0ecd5` | Became the reference code |
| 19:18 | **0/3** | Sweep sped up to 0.20 m/s: **motion blur in evening light** (28 good reads vs 147). Also a stale-telemetry race: look-again moves "started" but never flew. | Back to 0.144 m/s with 0.06 m/s slow zones; race fixed with a regression test (`9bb07b7`) |
| **19:48** | **1/3, hit the left wall** | One misread (ID 14) triggered look-again, which made the flight longer. PX4 believed the drone was on the left lane at y of about -1.35, but the video shows it along the wall at about y = -2.3: **about 1 m of position-estimate drift with no resets**. Gyro bias was small (at most about 5 deg over 84 s). Flow quality was 77-83 in dim light (90-117 earlier). No compass. | Laptop code reverted to the 19:01 code with look-again off (`ec46b88`). Lessons: full room lighting, 30 s still after power-on, RC kill ready. **The geofence cannot catch estimate drift.** PX4 log `mini_project/flight_test_2026-11-10/px4_logs/log78_1948_wall_hit.ulg`; video `C:\cde1302_course\photos\arena\1958.mp4` |

Flight-map screenshots: `mini_project/flight_test_2026-11-10/flight_maps/` (`1620.png`, `1625.png`, `1822.png`, `1826.png`, `1903.png`, `1921.png`, `ABORTED_due to hitting soft wall_notphysically.png`, `FAILED TRAG.png`, `failed TRAG 2.png`, `Tag_mission_live_map_101126_1615.png`). Arena photos and the team's SolidWorks sketch are in `mini_project/flight_test_2026-11-10/arena/`. `analysis/path_shape_comparison.png` and `analysis/pdf_layout_drone_frame.png` are also there.

### 4.3 Wed 11 Nov: downward camera, drift hunt, and the configuration that worked

Before this session, the camera was remounted to look straight down (commit `016853e`; NOTES.md cites it as `c9171d3`, a hash that is not on this branch). 31 runs are logged. The notes are in `mini_project/flight_test_2026-11-11/NOTES.md`.

| Time | Result | Root cause | Fix |
|---|---|---|---|
| **19:42** | 2/3. **PX4 said it flew the rectangle and landed on the box; the video shows it went past the search area to the lockers.** | ULog: no xy, z or heading resets in flight. Gyro-integrated yaw within +-3 deg. Flow quality 77-93. **Integrating the flow sensor's velocity reproduces the EKF path, so PX4 faithfully tracked a wrong optical-flow measurement.** After touchdown there was a 1.9 m / -47 deg reset (an artefact: no compass, so yaw is never aligned). | Hand-carry test next. PX4 log `mini_project/flight_test_2026-11-11/px4_logs/2026-11-11_1942_flight.ulg`; map `mini_project/flight_test_2026-11-11/flight_maps/1945_111126.png`; video `C:\cde1302_course\photos\arena\1945_111126.MOV` |
| 19:58 | Detect-only hand carry, props off: x read 5.44 m out and came back to within 0.11 m; y wander under 0.15 m | (The real walked distance was not recorded.) | |
| **20:02** | **3/3**, landed in the box. The return via `CMD_GOTO` **spun the drone about 300 deg** (bulge on the map). | The firmware's goto rotates, then flies. | `2026-11-11_2002_flight.ulg`; map `2005_111126.png` |
| 20:13 | Known-tags hover mode (tag positions from 20:02): 3/3, but the tags came out 0.37-0.64 m from the reference and the legs were diagonal | Hard-coded positions inherit the drift; the brief says tags are at unknown spots | Mode dropped; back to the sweep. Map `2016_111126.png` |
| 20:14 | Landed off the box | Goto turned the drone about 220 deg while still moving at about 0.3 m/s. The start heading was 8 deg: **with no compass, "forward" is wherever the drone pointed at power-on.** | **Return flown as a trajectory with the heading held; optional start-heading check** (commit `c17032d`; NOTES.md calls it `3bdb74b`). `2026-11-11_2014_flight.ulg` |
| 20:26 | Aborted: telemetry stale | **2-3 s Wi-Fi gaps** all flight, with the live-map camera stream on | Run `live_map` without the camera panel; sit near the router. Map `flight_maps/2030_Failed run comms lost.png` |
| 20:29 | **Pilot took over** ("Pilot took over using sticks" at 55.8 s, kill switch at 62.7 s); laptop log: geofence breach at (-0.74, 0.09) | Start heading 0.1 rad (6 deg): the whole frame was rotated, so it headed for the left wall while PX4 said y = -1.34 | Heading check (`max_start_heading_deg`). `2026-11-11_2029_run2.ulg` (PX4 log 87); map `2031_fAILED RUN 2 near hitting left wall.png`; video `C:\cde1302_course\photos\arena\2031_2nd run.mp4` |
| 20:39, 20:52 | **7 takeoffs refused** by the heading check (-4.5 deg, then +5.1 to +5.6 deg) | The check refused with the drone square in the box | **Check turned off** (`max_start_heading_deg: 0`, commit `f42a447`) |
| 20:41 / 20:45 | 3/3 (landed left of the box) and 2/3 (tag 6: 2 frames) | | Maps `2045_111126 not in landing zone to the left of it.png`, `2050_landed but didnt scan the 3rd tag.png` |
| 20:54 | Sweep at 0.8 m: wobblier. **Battery died**; telemetry stopped and the mission aborted at 124.6 s | Flat battery (the ESP32 lost power) | Back to 0.6 m; spare battery on the checklist. Map `2056_STILLDRIFTING batteried died, didnt scan behind the block.png` |
| 21:09-21:20 | Several telemetry-stale aborts, one geofence abort (-1.08, -0.36) | Flow sensor being moved back into its **original casing** | |
| 21:22, **21:25** | **3/3 both.** 21:25: all tags found and search stopped at 90.7 s, landing command at 124.5 s, mission end 132.5 s, home 0.11 m from start | **Original flow-sensor casing + QGC sensor recalibration** fixed the drift | Config committed as the graded setup (`f42a447`). Map `flight_maps/RECALIBRATED AND IT FLIES LETSGOOO.png` |

The 21:25 tag estimates landed almost on the Flying Area PDF spots:

| Tag | 21:25 estimate | PDF spot | Error |
|---|---|---|---|
| 2 | (1.29, -1.38) | (1.35, -1.35) | 0.07 m |
| 4 | (2.73, -1.41) | (2.70, -1.35) | 0.07 m |
| 6 (foam box) | (4.15, 1.03) | (4.05, 0.90) | 0.16 m |

(The errors are computed from the NOTES.md table. The PDF spots are the nominal layout; the tags were not tape-measured.)

**The SD card stopped logging** after the casing work. MAVLink `SYS_STATUS` reported logging not present and the log list was empty, so the 21:22 and 21:25 flights have no `.ulg` (NOTES.md, "Not logged"). The root cause was found on 12 Nov evening: the card cannot be written to at all (see §5.y).

Also on 11 Nov: the downward camera changed the sweep slow zones to centre on the spots. The look-again offset went to 0. The camera offsets are still the 45-deg-mount values and were never re-measured (memory note on the downward camera; YAML comments).

### 4.4 Summary of failure classes

| Class | Seen | Root cause | Fix / mitigation | Status |
|---|---|---|---|---|
| EKF height + xy reset | 10 Nov 17:50 | Range-only height, baro off, foam block | Baro on, RNG noise 0.05 | Fixed |
| Optical-flow position drift | 10 Nov 19:48 (wall hit), 11 Nov 19:42, 20:29, likely 12 Nov run 3 | Wrong flow measurement: printed casing, calibration, dim light | Original casing, recalibration before every flight, good lighting | Reduced, not eliminated |
| Heading frame | 11 Nov 20:14, 20:29 | No compass; heading set at power-on | Power on square in the box; trajectories hold the heading; check available | Mitigated |
| Comms / power loss | 6 Nov, 10 Nov, 11 Nov 20:26, 20:54, 21:09-21:20 | Wi-Fi gaps with the camera stream on; flat battery | Stale-telemetry landing; map without the camera; fresh battery | Safety layer works |
| Trajectory start | 10 Nov 17:19/17:20 | START ignored before flying; lost chunks | Pre-upload + re-send START + fallback full upload | Fixed (fallback used in graded runs 2 and 3) |
| Misread IDs | 10 Nov 15:29, 19:48; 12 Nov runs 1 and 3 | tag16h5 bit errors | 3-frame rule, merge radius | Fixed (worked on 12 Nov) |
| Motion blur | 10 Nov 19:18 | 0.20 m/s in low light | 0.144 m/s + 0.06 m/s slow zones | Fixed |
| Backwards drift on the first leg | 6 Nov, 10 Nov 15:22/16:09/16:10/17:47, 12 Nov 16:50 | Not diagnosed (no .ulg for 12 Nov) | Soft and hard fences catch it | Open |
| Hardware | 10 Nov: GND wire off; Oct: PETG guards broke; 11 Nov: SD card stopped logging | SD card: every write fails with errno 22, so params and logs can't be saved (§5.y) | Resoldered; guards redesigned; SD card diagnosed, needs a reformat or replacement | §5.y |

---

## 5. Graded run results and analysis (Thu 12 Nov 2026)

The config was unchanged from the 21:25 practice run: `laptop/tag_mission_demo_traj.yaml`, trajectory mode, 1279 points, 0.144 m/s peak, 0.06 m/s in the tag zones. The PLAN line in every `events.csv` reads "search area x[0.9,4.5] y[-1.8,1.85] m, 6 waypoints".
- `--wait-for-go` was not used: the sweep started automatically after the 12 s takeoff wait.
- `heading_rad` at `LINK_ESTABLISHED` was 6.28 (that is, 0) in every run.
- The graded tag layout differed from the practice: ID 2 was on the **right** row near the start, ID 4 on the left row, and ID 6 on the right row at the far end.

Logs: `laptop/mission_logs/20261112_164042_drone1`, `20261112_164515_drone1`, `20261112_165029_drone1` (aborted attempt), `20261112_165155_drone1`. **These folders are git-ignored and exist only on the flight laptop.** Copy them into the repo or the report appendix before they are lost. Screenshots: `mini_project/graded_run_2026-11-12/run1.png`, `run2.png`, `run3.png`.

### 5.1 Results

| Run | Log | Tags (confirmed) | Tag detected at (s after takeoff cmd) | Search stopped | Landing cmd | Mission end | Landing cmd position (dist. to start) |
|---|---|---|---|---|---|---|---|
| 1 | 20261112_164042_drone1 | **3/3** [4, 6, 2] | 46.0, 87.5, 115.7 | 119.7 s (all found + 4 s) | 140.0 s | **148.0 s** | (-0.03, 0.08), 0.09 m |
| 2 | 20261112_164515_drone1 | **3/3** [4, 6, 2] | 48.8, 84.1, 117.2 | 121.3 s | 141.7 s | **149.7 s** (screenshot shows 149.5) | (-0.01, 0.01), 0.02 m |
| attempt | 20261112_165029_drone1 | 0 | - | soft fence at 19.9 s | 21.1 s (emergency) | - | geofence breach at (-0.50, -0.91) |
| 3 | 20261112_165155_drone1 | **2/3** [4, 6]; ID 2 seen in 1 frame only | 49.2, 86.8 | full sweep, `TRAJECTORY_END` 142.4 s | 142.6 s | **150.6 s** | (-0.03, 0.17), 0.17 m |

"Mission end" is the landing command plus the 8 s land wait. The examiner's clock stops at touchdown in the start area. The takeoff commands were sent at 16:40:55.065, 16:45:28.902 and 16:52:05.024.

**Total: 8 of 9 tags.**

### 5.2 Tag estimates, accuracy and repeatability

| Tag | Run 1 | Run 2 | Run 3 | Spread within a run | Sightings |
|---|---|---|---|---|---|
| 2 | (1.21, 1.21) | (1.23, 1.15) | not confirmed (1 frame at (0.77, 0.89)) | 0.035 / 0.041 m | 109 / 115 |
| 4 | (2.71, -1.19) | (2.70, -1.32) | (2.68, -1.22) | 0.038 / 0.049 / 0.033 m | 115 / 104 / 112 |
| 6 | (4.10, 1.26) | (4.11, 1.13) | **(3.61, 1.53), outlier** | 0.044 / 0.059 / 0.047 m | 53 / 88 / 97 |

Numbers computed from `tags.csv`:
- **Run-to-run repeatability.** ID 2 runs 1 and 2 are 0.06 m apart. ID 4 runs 1 and 2 differ by 0.13 m (in y), and all three ID 4 estimates are within 0.08 m of their mean (2.70, -1.24). ID 6 runs 1 and 2 are 0.13 m apart. ID 6 in run 3 is **0.60 m** from the mean of runs 1 and 2.
- **Spread** (the 1/range^2 weighted standard deviation of the sightings) was 0.033-0.059 m. Each tag had 53-115 accepted sightings.
- **Range** at `TAG_DETECTED` was 0.59-0.78 m. The best range per tag was 0.50-0.68 m.
- **Accuracy against the examiner's reference positions is unknown.** We do not have the reference coordinates (section 8). For context, in practice (11 Nov 21:25), estimates were 0.07, 0.07 and 0.16 m from the PDF spots.

### 5.3 Anomalies explained

**(a) The final "pos" on the screenshots is not where the drone landed.**
- Runs 1 and 2 show about (-0.17, -1.31) and (0.25, -1.14); run 3 shows (0.01, 0.50).
- In every run, `trajectory.csv` holds steady near home after the landing command (within about 0.15 m) and then jumps once, while the drone is IDLE on the ground:
  - Run 1 at about 146.3-146.9 s: heading changed by about 45 deg, and xy jumped about 1.5 m, from (-0.02, 0.14) to (-0.12, -1.37).
  - Run 2 at about 149.0-149.2 s: heading changed by about 39 deg, and xy jumped about 1.2 m, to (0.25, -1.14).
  - Run 3 at about 150.1-150.6 s: heading changed by about -26 deg, and xy jumped about 0.5 m, to (0.01, 0.50).
- This is the same post-touchdown estimator reset that the 11 Nov 19:42 ULog showed (1.9 m and -47 deg after touchdown), caused by the yaw never being aligned without a compass.
- A carry would show gradual motion, not a single jump. **So the drone was not carried. Use the `START_AREA_REACHED` / `LANDING_CMD_SENT` positions (0.09, 0.02, 0.17 m from start) as the landing position**, and say that they are estimates (whether it landed inside the box is the examiner's call; the run 1 video confirms it landed inside the start box, see §5.x).

**(b) Run 3 missed ID 2, and its ID 6 is 0.6 m off (inferred: position-estimate drift).**
- ID 4 on the left lane (at about 49 s) agreed with runs 1 and 2. The drift therefore started after that.
- At ID 6, the drone was at the same *estimated* position as in run 1, about (4.0, 1.1-1.3). In run 1, the camera saw the tag just ahead of it (body forward +0.12 to +0.40 m). In run 3, the camera saw it **behind** it (forward -0.37 m) and to the right. The tag-relative geometry inside run 3 is self-consistent, so the drone's *frame* was off by about 0.5 m in x. This has the same signature as the 11 Nov 19:42 flow drift (the real drone ahead of where PX4 thought).
- On the right lane, run 3's estimated position sat at y of about 1.50-1.67 against the planned 1.40. Runs 1 and 2 had only 11 samples above 1.45 between them.
- The single ID 2 frame came at 131.2 s from a range of 1.13 m, when the drone believed it was at (0.29, 1.53). Working back from that sighting and run 1/2's tag position, the drone was really about 0.4 m further forward and 0.3 m further right than it believed. That put the real return lane about 0.6 m outboard of ID 2, so the tag was at the edge of the view and decoded only once. The 3-frame rule then (correctly) refused to confirm it. The y component of the error differs between ID 6 and ID 2, so the drift was not a constant offset.
- Without a .ulg for 12 Nov this cannot be proven. A run-3 video, if one exists, would confirm it.

**(c) Misreads were rejected as designed.**
- Run 1: ID 14 at (1.17, 1.19), 1 frame, which is ID 2 misread.
- Run 3: ID 3 at (2.67, -1.21), hamming 1, 1 frame, which is ID 4 misread.

Both are listed under `ignored_ids` in `summary.json` and never counted. This directly validates the fix made after 10 Nov 15:29.

**(d) Trajectory pre-upload fallback.** In runs 2 and 3, `TRAJECTORY_PREUPLOAD_NOT_STARTED` was logged at 13.5 s, followed by "falling back to full upload". The fallback started the sweep about 1.5 s later both times.

**(e) The aborted attempt (16:50).** On the first, sideways leg (planned along x = 0 to y = -1.35), the drone drifted backwards as well as left. It reached (-0.37, -0.86) at 19.9 s, triggering the soft fence (hold, then return). It then crossed the hard geofence at (-0.50, -0.91) at 20.9 s, and the laptop sent an emergency landing at 21.1 s. No tags were found. The safety layer did exactly what it was designed to do. The same backwards drift on the first leg was seen on 6 Nov and 10 Nov. Report it honestly as a failed attempt that was flown again.

**(f) Completion time vs practice.** At 21:25 on 11 Nov the drone took 124.5 s to the landing command, because all 3 tags were on the first lane or at its far end. In the graded layout, ID 2 was near the end of the return lane, so the sweep had to cover almost the whole path before the early return (140.0-142.6 s to the landing command).

### 5.4 Marking-scheme view of the demo

| Criterion (weight) | What happened |
|---|---|
| Auto takeoff (5%) | 4 of 4 takeoffs reached hover |
| Return and land (5%) | 3 graded runs sent LAND 0.02-0.17 m from the start (estimate); 1 attempt ended in an emergency land |
| Tag detection (15%) | 3/3, 3/3, 2/3 unique IDs; 0 false IDs reported |
| Location accuracy (15%) | Reference positions unknown; runs repeat within 0.06-0.13 m; run 3 ID 6 off by about 0.6 m |
| Completion time (15%) | 148.0-150.6 s to mission end (140-143 s to the landing command) |
| System quality (15%) | Smooth trajectory with no stops or turns, layered safety (it caught the drift attempt), 3-frame misread rejection, config-driven, tests, live map |

### 5.x Graded run 1 video and the examiner's questions

The video is `graded run 1.MOV` (151 s, filmed from behind the start box). Its audio was transcribed locally with Whisper large-v3, so the quotes below are close but not word-perfect. The video starts at the "3, 2, 1, go" countdown, about 3–4 s before the log's t = 0.

What the video shows:
- **Takeoff.** The drone lifts off inside the start box.
- **Sweep.** It flies up the left lane, across the far end, and back down the right lane. This is the loop the plan describes.
- **Tag 2 called.** At about video 119 s someone says "tag two is scanned". The log has TAG_DETECTED id=2 at 115.7 s.
- **Return.** It flies home with the camera holder saying "come on" (video 134–147 s).
- **Landing.** It lands inside the start box at about video 150 s ("Wow… yay… very good").
- **The landing settles a question.** The −1.3 m "final position" on the run 1 map is an estimator jump after touchdown. The drone did not move.

Examiner's questions and the answers given. The "Correct for the report" column is checked against the config and the logs.

| Video time | Examiner asked | Answer given | Correct for the report |
|---|---|---|---|
| 4–30 s | "What's your search strategy?" | "One loop around the whole thing… one circle." | Right. It is a two-lane loop: lanes at y = −1.35 and +1.40 m, then across at x ≈ 4.05, then return. In the config these are the 6 planned waypoints. |
| 43–50 s | (follow-up) | "It covers each point to make sure there is no [tag] there… it will linger at each of the six points." | **Imprecise.** In trajectory mode the drone does not stop at the 6 points; it flies one continuous path. It slows from 0.144 to 0.06 m/s within ±0.3 m of each *expected tag spot*, and it hovers once, for 4 s, after all 3 tags are found. (Whisper heard "RFID"; the word was probably "tag".) |
| 55–64 s | (heights) | "We started at 0.5, then 0.8, then 0.7." | **Partly wrong.** The config says 0.5 m first, 0.8 m tried on 11 Nov 20:54 ("wobblier, battery died"), and **0.6 m** flown in the graded run, after taking off at 0.5 m. |
| 66 s | — | "Over 45 flights in the past two days." | The mission logger recorded **37 mission takeoffs** on 10–11 Nov (20 and 17). Flights with other scripts (manual, takeoff/land tests) aren't in that count, so "over 45" is plausible but unverified. |
| 74–101 s | — | Over the block "it would think it's about to crash and go crazy"; "we turned on the barometer in the FC so it has two sources for height"; "we crashed into the black cover". | Right in substance. This is the 10 Nov 17:50 EKF height reset over the foam block (rangefinder-only height, then a climb and reset). The fix was `EKF2_BARO_CTRL=1` plus `EKF2_RNG_NOISE` 0.05 (see §4). |
| 105–109 s | "Did you manage to detect two of them?" | "Yes, on to the last one." | Matches the log: ID 4 at 46 s and ID 6 at 87.5 s. |
| 113–117 s | "It's not stable at all today, huh?" | "I think it's reasonable." | **Worth addressing in the report.** The examiner noticed the wobble. The run 1 map shows ±0.1–0.2 m lane wander from the optical-flow position hold (no motion capture, no compass). Discuss this as a known limitation (§6). |
| 121–128 s | "All three already?" / "Easy mark, right? Just scan all three." | "Automatically." | Positive: the examiner acknowledged the autonomous 3/3. |

### 5.y Why there is no PX4 log (.ulg) for the graded run

After the graded run the drone was stripped down to the flight controller. On 12 Nov evening the FC was read over USB (COM10) using MAVLink. The checks were the log-list request (`LOG_REQUEST_LIST`) and the PX4 shell commands `logger status`, `ls` and `dmesg`.

| Check | Result |
|---|---|
| Log list over MAVLink | **0 logs** |
| `ls /fs/microsd/log` | **empty** |
| `logger status` | Running in mode "all", 143 subscriptions, "Not logging" (normal when disarmed) |
| `SDLOG_MODE` | 0, which means log from arm to disarm (correct) |
| `df -h` | SD card mounted, vfat, 3580 MB, 169 MB used, 3410 MB free |
| Files on the card | `params` 0 B, `logdata.txt` 0 B, `dataman` 68 KB, `parameters_backup.bson` 2 KB, `param_import_fail.txt` 1.3 KB (could not be read: "read failed: Connection timed out") |
| `dmesg` | Repeated `ERROR [tinybson] file write error -1, errno:22 (Invalid argument)`, `parameter export to /fs/microsd/params failed`, and finally `param auto save failed (-1)` |

**Diagnosis.** The logger was configured correctly and was running. Space was not the problem either: the card is mounted and has room. **Every write to the card fails** with errno 22 (EINVAL), which points to a corrupted FAT filesystem or a failing card. The logger therefore could not create a .ulg when the drone armed. This started after the casing work on 11 Nov; flights up to 11 Nov 20:29 have ULogs.

**What this means for the evidence:**
- No 12 Nov flight has a PX4 log. The graded-run evidence is the laptop mission logs in `laptop/mission_logs/20261112_*`: telemetry at 10 Hz, events, and every tag observation. The live-map screenshots and the run 1 video back them up.
- The required ULog analysis figure uses the 11 Nov 20:02 flight, a 3/3 run with a ULog.
- **Possible side effect (inferred, not verified):** parameter auto-save also fails. Parameter and calibration changes may therefore not have survived a power cycle, which could be why recalibrating before every flight was necessary. This could be checked after the card is fixed: change a parameter, reboot, read it back.

**Fix (not yet done).** Copy `parameters_backup.bson` off the card. Reformat the card as FAT32 on a PC, or replace it. Then confirm `logger status` shows a log being written while the drone is armed. A good addition to the pre-flight checklist: confirm the log list grows after a short arm and disarm.

This is a usable case for the report's optional abnormal-behaviour section: the symptom, the hypotheses, the evidence from the shell output above, and the diagnosis.

---

## 6. Limitations and future work

1. **The position estimate is the accuracy ceiling.** Optical flow with no compass drifts without PX4 knowing. It caused the 10 Nov wall hit, the 11 Nov lockers flight and (inferred) run 3's miss. Improvements:
   - correct the drone position from tags already seen (a known tag acts as a landmark), or re-anchor on return;
   - a magnetometer or external heading reference;
   - vision-based odometry;
   - a hand-carry calibration of the flow scale before each session.
2. **Camera offsets were never measured** on the downward mount (`fwd 0.070`, `down 0.008` are CAD placeholders), and detection latency is assumed to be 0 (`latency_s: 0.0`).
3. **Roll and pitch are ignored** in the localisation, because they are not in the telemetry. The analysis script can re-localise with the .ulg attitude (`analyze_flight.py`).
4. **Two lanes assume the tags are on the marked spots.** For tags anywhere, use a 5-lane plan (about 2x longer). Look-again (re-visiting single glimpses) is coded but disabled. With a fix for the misread trigger, it would have rescued run 3's single ID 2 frame.
5. **No touchdown signal.** Landing equals the command plus 8 s.
6. **Wi-Fi robustness.** Gaps of 2-3 s with the camera stream on. The 3 s stale timeout is a blunt fix.
7. **Logging.** The FC SD card stopped accepting writes on 11 Nov, so the best runs, including all of 12 Nov, have no PX4 log. A pre-flight check of `logger status` or the log list would have caught it before the graded run. Future work: add that check to the checklist and keep a spare formatted card (§5.y).
8. **Backwards drift on the first leg** was never diagnosed.
9. **Battery:** one run lost power mid-flight. There is no battery-voltage check in the mission.

---

## 7. Evidence index (mapped to the report sections the brief asks for)

### Report sections (brief section 6.2)

| Report section | Use this | Path |
|---|---|---|
| System architecture and data/control flow | Architecture figure; module docstrings; firmware tasks | `mini_project/report/fig_architecture.png`; `laptop/tag_mission.py`, `laptop/drone_link.py`; `main/nav_task.c`, `main/at_detect.c` |
| Exploration / navigation strategy | Sweep and trajectory code; the flown config; Simulink sweep figures | `laptop/sweep_planner.py`, `laptop/traj_gen.py`, `laptop/tag_mission_demo_traj.yaml`; `mini_project/simulink_sweep/results/` |
| Detection and localisation method | Localiser frames and fusion; firmware gate; host detector | `laptop/tag_localizer.py`, `main/at_detect.c`, `laptop/apriltag_host.py` |
| Important parameters and decisions | YAML comments record why each value changed; PX4 params | `laptop/tag_mission_demo_traj.yaml`; `mini_project/px4_params_drone1_2026-11-10.txt` |
| Experimental results | **Graded-run screenshots**; graded-run logs; best practice run | `mini_project/graded_run_2026-11-12/run1.png`, `run2.png`, `run3.png`; `laptop/mission_logs/20261112_*` (git-ignored); `mini_project/flight_test_2026-11-11/flight_maps/RECALIBRATED AND IT FLIES LETSGOOO.png` with `mini_project/flight_test_2026-11-11/mission_logs/20261111_212543_drone1/` |
| AI-assisted flight-data analysis (required .ulg plot) | `analyze_flight.py` (+ `flight_data.py`, `flight_plots.py`; uncommitted flight-mode colouring) on a saved ULog; **no 12 Nov .ulg exists**, so use 11 Nov 20:02 (a good 3/3 run with a ULog) | `mini_project/flight_test_2026-11-11/px4_logs/2026-11-11_2002_flight.ulg` + `mini_project/flight_test_2026-11-11/mission_logs/20261111_200237_drone1/` |
| Optional abnormal-behaviour diagnosis (brief sec. 8) | **Best case study: the 17:50 foam-block EKF reset.** It has a hypothesis, the ULog evidence, a parameter fix and a verification flight. Second case: 11 Nov 19:42 flow drift (integrated flow reproduces the EKF path; fixed by casing + recalibration) | `mini_project/flight_test_2026-11-10/analysis/reset_1750_block.png`, `mini_project/flight_test_2026-11-10/px4_logs/log67_1750_block_reset.ulg`, `mini_project/flight_test_2026-11-10/NOTES.md`; `mini_project/flight_test_2026-11-11/px4_logs/2026-11-11_1942_flight.ulg`, `mini_project/flight_test_2026-11-11/NOTES.md` |
| Limitations and failure cases | Failure flight maps; wall-hit ULog | `mini_project/flight_test_2026-11-11/flight_maps/2030_Failed run comms lost.png`, `2031_fAILED RUN 2 near hitting left wall.png`, `2056_STILLDRIFTING batteried died, didnt scan behind the block.png`; `mini_project/flight_test_2026-11-10/px4_logs/log78_1948_wall_hit.ulg` |
| Extra features (demo 10%) | 3-frame misread rejection (it worked in graded runs 1 and 3); smooth trajectory with slow zones; return trajectory holding the heading; pre-upload + fallback; soft and hard fences cut to measured walls; live map with camera panel; detect-only mode; host-compiled firmware detector; boxed-tag simulator; tag layout editor; CAD redesigns | commits `06b709b`, `42e6aac`, `c17032d`, `9719e1d`, `b751ea1`, `bdff550`, `c9f3776`, `016853e`; `laptop/live_map.py`, `laptop/tag_layout_editor.html`; `cad/` |
| AI usage declaration (brief sec. 9) | Commit trailers show Claude was used (`Co-Authored-By: Claude ...`); CAD agent reports | `git log`; `mini_project/stock_analysis/AGENT_REPORT.md`, `mini_project/audit/audit_report.md` |

### Other evidence

- **AI errors caught by the team** (useful for brief section 5.3 / 9):
  - The swarm/SAFMC scope was wrongly assumed until 17 Oct.
  - The deadline was wrongly called late on 11 Nov.
  - Commit hashes in NOTES.md (`c9171d3`, `3bdb74b`) do not exist on this branch; the real ones are `016853e` and `c17032d`.
  - `mini_project/graded_run_2026-11-12/README.md` says "known-tags mission", but the logs show the generated sweep.
  - A Webots sim found a `StuckMonitor` bug (`mini_project/webots/AGENT_REPORT.md`).
  - The CAD audit showed failing checks (`mini_project/audit/audit_report.md`).
- **Videos (outside the repo):** `C:\cde1302_course\photos\arena\`: `1st flight test.mp4`, `1903.mp4`, `1922.mp4`, `1958.mp4` (10 Nov) and `1945_111126.MOV`, `2005_111126.MOV`, `2015_111126.mp4`, `2030_1st run.mp4`, `2031_2nd run.mp4` (11 Nov). Graded run 1: `graded run 1.MOV` (repo root, untracked, 58 MB; see §5.x). No video of runs 2 or 3 yet.
- **Raw flight-map folder** (same images as in `mini_project/flight_test_*/flight_maps/`, plus `Tag_mission_live_map.png`): `C:\cde1302_course\flight_maps\`.
- **Brief, briefing slides and Flying Area PDF:** `C:\cde1302_course\mini_project_brief\`.
- **Existing report draft:** `mini_project/report/CDE1302_Mini_Project_Report.docx` / `.pdf`, built by `mini_project/report/build_report.py`. It has sections 1-8 and Appendix A, with yellow `[[placeholders]]`. `mini_project/report/check_report.ps1` checks the format.
- **Checklists:** `mini_project/GRADED_RUN_CHECKLIST.md` (the graded setup and symptom table); `mini_project/RUN_DAY_PROMPT.md`.

---

## 8. Open questions for Krish (confirm before submission)

1. ~~**The .ulg for 12 Nov.**~~ **Answered (12 Nov evening): there is none.** The FC's SD card rejects all writes (§5.y). The required ULog figure uses 11 Nov 20:02, and the report must say why.
2. **The examiner's reference tag positions.** We need them to quote the localisation error. Without them, we can only report repeatability.
3. **Run 3 drift.** Is there a video of run 3, or any note on whether the drone was visibly off the right lane? Was any tag moved between runs?
4. **Did the examiner accept the landings inside the start box** in all three runs, and how did they time the runs? The 16:50 attempt: was it counted as a restart?
5. **Which printed parts were flown:** the stock frame with the professor's camera and flow mounts, or `cad/stock_plus` parts? Which flow casing counts as "original"? The camera fwd/down offsets on the downward mount were never measured. Can they be measured now for the report?
6. **Copy the graded-run logs** (`laptop/mission_logs/20261112_*`, git-ignored) into the repo or report appendix, so the GitHub link carries the evidence.
7. **Team roles and authorship.** Commits are by Krish, Nuthasith Gerdpratoom, "lf" and CarlsonGYC, plus a PR from `yh-html`. Which of these are teammates and which are teaching staff (the firmware base)? This is needed for the contributions appendix.
8. **Test count.** 211 test functions by count against "248 pass" in the run-day notes. Re-run `pytest laptop/tests` and quote the real number.
9. **Uncommitted analysis changes** (`laptop/analyze_flight.py`, `laptop/flight_data.py`, `laptop/flight_plots.py`) should be committed before the GitHub link goes in the report.
