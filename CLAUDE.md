# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

CDE1302 mini-project: a single indoor quadrotor (drone ID 1) takes off on command, searches a
3.6 m x 3.15 m area for three AprilTags (tag16h5), logs each tag's ID and position, returns and
lands. The graded run was flown on 12 Nov 2026 (results in `mini_project/graded_run_2026-11-12/`).

An ESP32-S3 on the drone is the companion computer: it talks to the PX4 flight controller over
MAVLink/UART, reads eight VL53L5CX ToF sensors and detects AprilTags with its camera. The laptop
app in `laptop/` plans the sweep, sends goals over WiFi/UDP, draws a live map and logs every flight.

The codebase started as an SAFMC 2026 Cat Swarm Challenge system. The swarm-only notes (fleet
exploration, relay, peer avoidance) are in `docs/archive/CLAUDE_swarm.md`; that code still lives
in `laptop/` and `main/` but is not used by the mini-project mission.

## Layout

| Path | What |
|---|---|
| `main/`, `components/`, `managed_components/`, `tools/` | ESP32 firmware (ESP-IDF). `CMakeLists.txt`, `sdkconfig*`, `dependencies.lock`, `flash_drone.sh` must stay at the root |
| `laptop/` | Mission code, configs (`tag_mission*.yaml`, `setup.yaml`), tests, flight analysis. `laptop/mission_logs/` is git-ignored |
| `trajectory/` | MATLAB trajectory generator and CSVs |
| `mini_project/` | Guides, checklists, flight-test notes, graded-run results, report |
| `cad/` | Frame CAD: `fusion/` (Fusion 360 exports), CadQuery projects, `stock_plus/`, `stock_mounts/` |
| `docs/` | QGC bridge notes; `docs/archive/` holds SAFMC material, Lab 5 waypoint files, the old CLAUDE.md |

Course material (lectures, labs, photos, arena videos, flight-map screenshots, instructor STLs)
is outside the repo in `C:\cde1302_course\`.

Miniforge is still installed in the repo root (python.exe, DLLs, `Lib/`, `pkgs/`, `envs/` ...).
It is git-ignored and is not the Python the flight scripts use; use `py -3.11` (or the first
`python` on PATH). See `docs/REORGANISATION_PLAN.md` Step 3.

## Build & Flash (ESP-IDF)

ESP-IDF is the only external dependency; everything else is committed in-tree. The pinned
ESP-IDF version lives in `tools/idf-pin.env`.

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
./flash_drone.sh 1 COM5        # pins CONFIG_DRONE_ID + host IP, restores sdkconfig after
idf.py menuconfig              # "Drone Configuration"
```

## Laptop

Python 3.10+: `python -m pip install -r laptop/requirements.txt`.
`laptop/install_commands.ps1` puts `run_tag_mission`, `live_map`, `tag_stream`,
`send_trajectory`, `analyze_flight` and `simple_arm_takeoff_land` on PATH.

```bash
# Tests
python -m pytest -q laptop/tests

# Mission: print the plan only (no network), simulate, then fly
python laptop/run_tag_mission.py --drone-id 1 --config laptop/tag_mission_demo_traj.yaml --plan-only
python laptop/run_tag_mission.py --drone-id 1 --config laptop/tag_mission_demo_traj.yaml --simulate
python laptop/run_tag_mission.py --drone-id 1 --config laptop/tag_mission_demo_traj.yaml --wait-for-go
python laptop/live_map.py --config laptop/tag_mission_demo_traj.yaml

# Props-off link check
python laptop/simple_arm_takeoff_land.py --drone-id 1 --monitor-only

# Camera with AprilTag overlay; flight log analysis
python laptop/tag_stream.py --esp-ip 192.168.1.201
python laptop/analyze_flight.py --run laptop/mission_logs/20261112_164042_drone1 --config laptop/tag_mission_demo_traj.yaml
```

The graded-run setup and checklist: `mini_project/GRADED_RUN_CHECKLIST.md`. Always recalibrate
the sensors before every flight.

## Firmware architecture (`main/`)

Dual-core FreeRTOS on ESP32-S3:

| Task | Core | Priority | Rate | Role |
|------|------|----------|------|------|
| `mavlink_task` | 0 | 5 | 50 Hz | UART<->PX4: parses telemetry, streams setpoints |
| `tof_task` | 0 | 4 | 15 Hz | 8x VL53L5CX ToF via I2C mux (TCA9548A) |
| `wifi_task` | 0 | 2 | 10 Hz | UDP telemetry to laptop, receives commands |
| `nav_task` | 1 | 4 | 20 Hz | VFH obstacle avoidance, goal navigation, collision avoidance, WiFi killswitch |
| `at_detect_task` | 1 | 1 | ~2 Hz | AprilTag detection (tag16h5); detection only, never affects flight |
| `mission_task` | 1 | 2 | - | CMD_START -> arm -> takeoff -> laptop control -> CMD_LAND |
| `camera_stream_task` | 0 | 1 | on demand | JPEG preview for `camera_stream.py` / `tag_stream.py` |

- **Camera**: two frame buffers shared by `at_detect_task` and `camera_stream_task`. The camera
  has looked straight down (pitch 90) since 11 Nov. `laptop/apriltag_host.py` compiles the same
  `esp-apriltag` sources for the laptop; keep `at_detect.c`'s parameters and gate in sync with it.
- **Setpoints**: `mission_task` owns them during takeoff/landing; `nav_task` takes over on
  `nav_set_goal_ned()`; `nav_cancel()` hands them back.
- **Frames**: NED everywhere. PX4 odom origin is at takeoff; `odom` applies a fixed
  translation-only `map_T_odom` (the start offset).

## UDP protocol

ESP32 port 5005 out / 5006 in. Telemetry 10 Hz (position, heading, nav state, VFH bins, tag
sightings). Commands: `CMD_GOTO`, `CMD_LAND`, `CMD_HOLD`, `CMD_START`, `CMD_CAMERA_STREAM` (1 Hz
keepalive), plus the swarm commands. Debug ports: ToF 5007, AprilTag 5008, camera preview 5009.
`laptop/protocol.py` and the firmware structs must match.

## Key tuning constants

- `NAV_CRUISE_SPEED_MS`, `NAV_YAW_TOL_RAD`, `NAV_ARRIVE_RADIUS_M` in `nav_task.h`
- `VFH_DEFAULT_CONFIG` in `vfh.h`; `COLLISION_DANGER_M` / `COLLISION_CLEAR_M` in `nav_task.c`
- `CRUISE_ALT_M` in `main.c`
- Mission area, sweep spacing, known tags, speeds: the `laptop/tag_mission*.yaml` configs
