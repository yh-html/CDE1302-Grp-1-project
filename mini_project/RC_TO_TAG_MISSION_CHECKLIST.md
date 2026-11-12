# Drone 22: RC recovery to AprilTag mission

Use this checklist after the TA repairs the RC link. It covers the CDE1302 single-UAV task, not the SAFMC swarm stack. All bench checks use **props off**. Connect the battery only after the wiring has been inspected; disconnect it before moving wires. Keep SC kill engaged whenever the powered drone is not flying. Fly only in the supervised area with the instructor's approval.

## 1. Close the RC fault first (props off)

- [ ] Receiver LED shows a link to the RadioMaster Pocket. This alone does not prove PX4 receives RC data.
- [ ] In QGroundControl, `RC_PORT_CONFIG = Radio Controller`, `RC_INPUT_PROTO = CRSF`, and `RC_CRSF_PRT_CFG = Disabled` remain saved on the current PX4 1.15.4 build. These were the settings tested on 4 Nov; do not change them merely because the receiver LED is on.
- [ ] In Analyze Tools > MAVLink Console, run `rc_input status`. It must report RX bytes **greater than zero** on `/dev/ttyS5` and a valid RC signal. On 4 Nov it reported zero bytes and `searching for signal: CRSF`, even with the receiver LED steady blue. Ask the TA to check the receiver's actual TX output through the cable to the FC's `RX6` contact, its 5 V supply and common ground, and the six-position UART1/UART6 plug. Do not identify contacts by wire colour.
- [ ] Vehicle Setup > Radio shows stick and switch movement. Calibrate in Mode 2. In Flight Modes, confirm SB selects the intended modes and SC maps to kill; return SC to kill engaged.
- [ ] In Safety, review RC-loss and Offboard-loss actions with the instructor. With props off and disarmed, turn the transmitter off briefly and confirm PX4 detects RC loss; turn it on and confirm recovery. This bench test proves loss detection, not the in-flight landing action.

Stop if any box above fails. Do not perform a props-on test without a working manual takeover and kill switch.

## 2. Check the laptop, PX4 link, and sensors (props off)

1. Join `TP-Link_E95A`. For drone 22, set the laptop to `192.168.1.122`; the ESP32 is expected at `192.168.1.222`. Confirm the ESP32 is actually configured as drone ID 22. Do not use WSL. Allow Python's UDP ports 5005, 5008 and 5009 and QGroundControl's 14550 through the firewall.
2. In QGroundControl, confirm the correct vehicle, battery, attitude, local position and velocity estimate, downward distance reading, and no critical preflight warnings. Move the disarmed drone by hand over the floor and confirm that the attitude and distance reading respond. Do not fly on a `Not ready` status.
3. Close QGroundControl if it owns UDP 14550, then from `C:\cde1302` run:

   ```powershell
   python laptop/check_px4_esp32_link.py --drone-id 22
   python laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only
   ```

   Expect `RESULT: PASS`, drone ID 22, `Navigation state: IDLE`, and fresh telemetry. If these fail, fix the PX4-to-ESP32 or Wi-Fi link before testing the mission.

## 3. Prove the camera and tag logging (props off)

1. Run `python laptop/apriltag_host.py`; expect `tag16h5 ncodes = 22 (expected 22)`.
2. Run `python laptop/tag_stream.py --esp-ip 192.168.1.222`. Show a printed **tag16h5** tag at about 0.5 to 1 m. The live picture should draw a green outline, ID, and forward/right/down position. Press `q` to close it. This is a visual laptop-side check; it does not by itself verify mission logging.
3. Tape-measure the **real** lens location and pitch relative to the FC centre. The offsets in `laptop/tag_mission.yaml` are placeholders. `laptop/apriltag_host.py` uses a different `CAM_POS_M` placeholder, so the video overlay and mission coordinates can disagree until both are measured and updated.
4. Put the tags on the floor and record their actual IDs and positions from the start point. With the drone disarmed and props off, run:

   ```powershell
   python laptop/run_tag_mission.py --drone-id 22 --detect-only
   ```

   Carry the drone by hand past each tag, then press Ctrl+C. The console should print `TAG_DETECTED` with an ID and local position and finish with a tag table. This mode sends **no** takeoff, goto or land commands. To check horizontal error, add `--truth "ID:X,Y ID:X,Y ID:X,Y"` using your measured IDs and coordinates, not the example IDs in the simulation config.
5. Open the new `laptop/mission_logs/<time>_drone22_detectonly/` folder. Check `events.csv` for timestamped detections, `observations.csv` for sighting details and camera-frame translation (`tx`, `ty`, `tz`), `tags.csv` for final fused positions, and `summary.json` for run status. The current packet and log do **not** contain tag rotation; clarify with the instructor whether translation alone satisfies the brief's "relative pose" requirement. Do not proceed if IDs or positions are wrong or no observations appear.

## 4. Verify the sweep before a flight

From `C:\cde1302` run:

```powershell
python laptop/run_tag_mission.py --drone-id 22 --plan-only
python laptop/run_tag_mission.py --drone-id 22 --simulate
```

`--plan-only` neither connects nor flies. On 4 Nov it printed eight waypoints:

```text
1 (1.20, -1.50)   2 (4.20, -1.50)
3 (4.20, -0.65)   4 (1.20, -0.65)
5 (1.20,  0.20)   6 (4.20,  0.20)
7 (4.20,  1.05)   8 (1.20,  1.05)
return to (0.00, 0.00), then land
```

The configured search rectangle is x = 0.9 to 4.5 m, y = -1.8 to 1.35 m, with a 0.3 m waypoint edge margin. Planned travel including return is 18.1 m; the script estimates 110 s at the assumed 0.2 m/s. These are **planned** values, not measured flight results. Confirm the real launch point, boundaries and which direction is positive x/y with the instructor. With props off, carry the drone from the start toward the search area and then right; check telemetry reports increasing x and then increasing y. Stop if the local frame does not match the plan.

QGroundControl is used to configure and monitor PX4. The sweep is executed by `run_tag_mission.py` through the ESP32, **not** uploaded as a QGroundControl Mission plan. Do not recreate these waypoints in QGroundControl unless the control architecture is deliberately changed and separately tested.

## 5. Flight gates (supervised area only)

1. Complete the mechanical and PX4 checks in `mini_project/DRONE_BRINGUP_GUIDE.md`, including motor order and spin direction with props off, guards, battery, optical-flow orientation, range data, and preflight Ready status. The pilot holds the working RC; assign a laptop operator and observer.
2. With the instructor's approval, fit props and perform a short **manual Position-mode hover and landing** first. Stop if it drifts, oscillates, or loses position or height estimation. Inspect the `.ulg` before advancing.
3. Only after a clean manual hover, run `python laptop/simple_arm_takeoff_land.py --drone-id 22` for an autonomous takeoff, 0.5 m hover and landing. This script asks for `ARM-22`. Confirm PX4 enters Offboard, holds position and lands. Do not use the props-off arm test as proof of hover or landing.
4. Only after a clean autonomous hover, use the short waypoint test in section 7 of the bring-up guide. Confirm its points are inside the available supervised area. Do not run the example file unchanged if it does not fit the space.
5. Only after the earlier gates pass, position the drone at the confirmed start, facing the search area. Run `python laptop/run_tag_mission.py --drone-id 22 --plan-only` again against the final config. Then run `python laptop/run_tag_mission.py --drone-id 22` and type `TAKEOFF-22` when instructed. Keep the RC pilot ready to take over. The script sends the takeoff, eight sweep goals, return-to-start goal and landing command, while printing tag events.
6. Confirm actual touchdown and disarm in QGroundControl. The current mission code records a landing command and then waits 8 s; that wait is **not** touchdown detection. After each flight, engage SC kill, wait for props to stop, disconnect the battery, keep the mission log folder and download the PX4 `.ulg`.

Do not reflash the ESP32. Nothing in this checklist has yet been proven in flight on this drone. The first real run must be treated as a test, with each stage evaluated before the next.
