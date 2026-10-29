# Pre-flight guide: from a built drone to the first mission

Written 29 Oct 2026 for the final run on **Thu 12 Nov, 4:15 to 5:00 pm**. Drone ID **22** is used in every example: **replace it with the ID on your drone** (the number flashed into the ESP32).

Where each step comes from:
- **[repo]** = taken from this repo's own guides (`README.md`, `laptop/TEST_GUIDE.md`, `waypoints/WAYPOINT_FLIGHT_TEST_GUIDE.md`, `QGC_MAVLINK_BRIDGE.md`, `mini_project/GUIDE.md`).
- **[general]** = ordinary PX4 / QGroundControl practice that the repo does not cover. I have not seen your flight controller's parameters, so treat these as things to *check*, not values to type in.
- **[checked 29 Oct]** = I ran it on this laptop.

Not yet fixed anywhere: the real standoff length, which side the FC sits on, the real camera lens position, and the motor screw depth. Settle them before you fly.

---

## 0. The order that keeps you safe

1. Mechanical check, **props off**.
2. Laptop network and software.
3. ESP32 firmware (only if needed).
4. QGroundControl over USB, then over WiFi.
5. Bench tests with **props off**: link, camera, tags, `--detect-only`.
6. First flight tests, in this order: hover, then waypoints, then the mission.
7. Download the `.ulg` log after every flight.

Never skip a step because the previous flight "was fine". Props go on only for step 6.

---

## 1. Mechanical and electrical check (props off)

- [ ] Frame, bottom panel, legs and guards: no cracks. Printed parts that have been through a crash get replaced, not repaired.
- [ ] Motors turn freely by hand, bolts tight, **M2 x 10 screws only**. The safe thread depth into the motor base is **unmeasured**: do not use longer screws. [repo]
- [ ] Motor wires, ESC and FC connectors seated; no wire touches a prop path.
- [ ] Battery: Tattu 3S 850 mAh (58 x 30 x 22 mm, about 76 g). Check the puffing, the voltage on a checker (never fly below the level your instructor gives) and that the Velcro strap holds it. The arm roots clear the battery by only about 0.4 mm on the v3 frame, so check the clearance on the real stack. [repo notes]
- [ ] **Centre of gravity** over the middle of the frame (balance it on a finger under the FC). A forward or sideways CG shows up as drift in the first hover.
- [ ] Optical-flow sensor (MTF-02P): faces straight down, lens clear of the bottom panel, legs and tape, cable not touching the sensor face. If the sensor is dirty or blocked, position hold fails.
- [ ] Camera (XIAO ESP32S3 Sense): firmly seated in its holder, lens clear, tilted 45 degrees, no wire in the view. The pitch and lens position must match `laptop/tag_mission.yaml`; measure them (section 8).
- [ ] ELRS receiver bound to your transmitter, antenna clear of the motors, ESC wires and the camera cable. RC transmitter charged, and you have tested that the mode switch and kill switch work (props off).
- [ ] **ESP32 to PX4 UART wiring** [repo]: ESP32 UART1 TX is GPIO5, RX is GPIO4, 921600 baud, TX/RX crossed and **common ground**.
- [ ] Props: right direction for each motor, tight, undamaged. Fit them **last**, after the bench tests.

---

## 2. Laptop setup

[checked 29 Oct: the Python environment and the detector build work.]

```powershell
cd C:\cde1302
$py = 'C:\cde1302\envs\cde1302-laptop\python.exe'
& $py laptop/apriltag_host.py       # must print: tag16h5 ncodes = 22 (expected 22)  [checked]
```

- [ ] Join the **drone WiFi** (the firmware is set to SSID `TP-Link_E95A`; confirm it is the router you will use on the day).
- [ ] Router gateway `192.168.1.1`, netmask `255.255.255.0`. [repo]
- [ ] **Set the laptop's IP to `192.168.1.(100 + drone ID)`** (drone 22 is `192.168.1.122`). The ESP32 sends telemetry only to that address. The drone itself is at `192.168.1.(200 + ID)` (drone 22 is `192.168.1.222`). [repo]
- [ ] The WiFi must allow **client-to-client** traffic. [repo]
- [ ] Windows Firewall: allow Python for UDP 5005, 5008, 5009 and QGroundControl for UDP 14550; set the WiFi to a **Private** network. **Do not use WSL**: it drops the drones' packets. [repo]
- [ ] Tests still pass: `cd laptop; & $py -m pytest tests -q` (91 passed, 4 skipped on 29 Oct).
- [ ] The simulation still runs: `& $py laptop/run_tag_mission.py --drone-id 22 --simulate`.

---

## 3. ESP32 firmware (flash only if you have to)

The README says the firmware is **already flashed** on the drone's ESP32. [repo] Flash only if: the drone ID is wrong, the WiFi/host IP is wrong, or the instructor gives new firmware.

Facts [checked 29 Oct]: `main/` has not changed since 16 Oct. ESP-IDF is installed at `C:\esp-idf` and a build from 17 Oct exists (`build\esp-everything.bin`). **Flashing has never been tried on this laptop.** `sdkconfig` currently has drone ID 22, host `192.168.1.122`. If your drone is not 22, you must change both.

```powershell
. C:\esp-idf\export.ps1
cd C:\cde1302
idf.py menuconfig                      # Drone Configuration: drone ID, WiFi, host IP
idf.py -p COM7 flash monitor           # use your COM port (Device Manager)
```
`./flash_drone.sh <id> <port>` does the same and pins the ID and host IP (host = `192.168.1.(100+ID)`), but it is a bash script; run it from Git Bash and expect to debug it. [repo]

- [ ] Close QGroundControl first (it holds the port). Keep the repo path free of spaces and non-English letters. [repo]
- [ ] Watch the serial monitor for these lines: [repo]
  ```
  UART1 ready: TX=GPIO5 RX=GPIO4 @ 921600 baud
  QGC MAVLink bridge enabled: PX4 UART <-> UDP <laptop_ip>:14550, local UDP 8888
  Telemetry valid
  Waiting for CMD_START from laptop...
  ```
- [ ] Note that opening the serial monitor **resets the ESP32**. Open it *before* arming, never after.
- [ ] `CRUISE_ALT_M` is 0.5 m in `main/main.c`. `WIFI_CRUISE_ALT_M` in `main/wifi_task.c` must equal it. [repo]
- [ ] ToF is **disabled** (`TOF_ENABLED 0`), so the pre-arm sensor check is skipped and there is no obstacle avoidance. [repo / checked]

---

## 4. QGroundControl setup

### 4a. PX4 over USB first
[repo: bridge doc step 2; the rest general]
- [ ] Connect the flight controller (MicoAir H743V2 AIO, board label `H743V2AIO`) by USB. QGC must recognise it. **Props off.**
- [ ] **Do not reflash PX4 or reset parameters** unless the instructor says so. Your drone is probably already configured. First read it. Save the current parameters to a file (Vehicle Setup, Parameters, Tools, Save to file) so you can always go back.
- [ ] Vehicle Setup pages: Airframe set for a quad, **sensors calibrated** (compass if used, gyro, accelerometer, level horizon), **radio calibrated**, **flight modes** set so you can switch out of Offboard instantly, **power** (3S battery, correct voltage and capacity), **safety** (failsafe actions, kill switch on a spare switch). [general]
- [ ] **Motor test** (Vehicle Setup, Motors) with props off: each motor spins, and the right one for its number. [general]
- [ ] **Optical flow and height:** the mission needs a valid *local position* from PX4. Check in QGC that position and altitude are estimated, and that the flow sensor and its distance reading show live data when you move the drone over the floor. Which PX4 parameters turn this on is **not in the repo**: ask your instructor or compare against the saved parameter file from a drone that flew in week 7. [general]
- [ ] Speed limit: waypoint speed is set by PX4 `MPC_XY_VEL_MAX` (about 0.2 m/s per the repo's waypoint guide). Read the real value from your drone. Do not change it without asking your instructor. [repo]

### 4b. The ESP32 serial link (PX4 side) [repo bridge doc]
- [ ] The PX4 telemetry port wired to the ESP32 is set to MAVLink and the baud rate matches. Example if it is TELEM2: `MAV_2_CONFIG = TELEM2`, `MAV_2_MODE = Onboard`, `SER_TEL2_BAUD = 921600`. Restart PX4 after a change. **Check the real port in your drone's parameters before touching it.**

### 4c. QGroundControl over WiFi through the ESP32 [repo bridge doc]
- [ ] Turn off QGC's auto-connect, then add a manual **UDP** link: Listening Port **14550**, Target Host the ESP32's IP (`192.168.1.222` for drone 22), Target Port **8888**.
- [ ] If QGC does not see the vehicle: check the host IP in the firmware is the laptop's address, both are on the same WiFi, PX4 baud is 921600, TX/RX are crossed with a common ground, and the QGC target port is 8888.
- [ ] **The bridge forwards everything**, including arm and mode commands from QGC. During flights, use QGC to **watch only** and do not send commands from both QGC and the Python scripts. [repo]

---

## 5. Bench tests with the props off

Run from `C:\cde1302`, with `$py` set as in section 2.

1. **Link check** (close QGC first; it listens on 14550): [repo]
   ```powershell
   & $py laptop/check_px4_esp32_link.py --drone-id 22      # expect RESULT: PASS
   & $py laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only
   ```
   Expect `Connected: drone=22, ESP32_IP=192.168.1.222` and 15 s of `nav=IDLE | age=` with `age` well under 1 s. If you see `No telemetry ... (heard drones: ...)`: another ID means the wrong `--drone-id`; `none` means the laptop IP or the firewall.
2. **Camera and tags:** [repo]
   ```powershell
   & $py laptop/camera_stream.py --esp-ip 192.168.1.222
   & $py laptop/tag_stream.py --esp-ip 192.168.1.222 --detail
   ```
   Hold a printed tag16h5 tag in view at about 0.5 to 1 m. It should be outlined with its ID. Use `--fps 5` in flight.
3. **Props-off arm test:** [repo]
   ```powershell
   & $py laptop/simple_arm_takeoff_land.py --drone-id 22      # type ARM-22
   ```
   With the props off, expect `Takeoff timeout: NED z=0.0x ...` instead of `Altitude reached`: that is normal. If you see `OFFBOARD refused (check QGC)` or `arming refused (check QGC)`, PX4 rejected it; fix the cause in QGC.
4. **Walk-through of the whole tag pipeline, no flight** [new, 29 Oct; tested only against a simulated carried drone]:
   ```powershell
   & $py laptop/run_tag_mission.py --drone-id 22 --detect-only --truth "1:1.35,-1.35 2:4.0,-1.35 3:2.7,0.9"
   ```
   Carry the drone by hand along the planned sweep with a tag on the floor, then press Ctrl+C. It prints each tag with its position and the error against the positions you measured with a tape. This is how you check the camera offsets in `tag_mission.yaml`. Keep the walk short and watch the position estimate: PX4's local position may drift if the flow sensor is not running.

Start `tag_debug.py` only *after* the flight script, because it takes UDP 5005. [repo]

---

## 6. Flight tests (props on), in this order

Do the next test only when the previous one was clean. Fly in the real arena layout (3.6 x 3.15 m search area), a clear space with no people near, with an RC pilot ready.

**Test A: arm, hover at 0.5 m, land** [repo]
```powershell
& $py laptop/simple_arm_takeoff_land.py --drone-id 22     # type ARM-22
```
The script sends START, waits 12 s, HOLD, waits 5 s, LAND. Watch QGC: Offboard, armed, about 0.5 m, steady hover, Land, disarmed. In the serial monitor you should see, in order:
`CMD_START received`, `OFFBOARD mode confirmed`, `Armed confirmed`, `Taking off to 0.5 m AGL`, `Altitude reached`, `CMD_LAND received`, `Disarmed - mission complete`, `Waiting for next CMD_START`.
**Pass:** a steady hover with little drift, and a clean landing. If it drifts, goes sideways or oscillates: land with the RC and check the CG, the flow sensor and the PX4 log. Do not continue.

**Test B: short waypoint test** [repo]
```powershell
& $py laptop/simple_waypoint_mission.py --waypoints-file waypoints/waypoints_example.txt --confirm --drone-id 22
```
Always use `--confirm` (type `MISSION-22`): without it the drone takes off as soon as telemetry arrives. Route: (0,0), (0.8,0), (0.8,0.8). It flies straight lines at about 0.2 m/s.

**Test C: the mission itself**
```powershell
& $py laptop/run_tag_mission.py --drone-id 22 --plan-only      # check the waypoints first
& $py laptop/run_tag_mission.py --drone-id 22                  # type TAKEOFF-22
```
- Start with the drone at the launch dot, **facing the search area**, powered on there (PX4's local origin is where it was powered on).
- The search area in `tag_mission.yaml` is x 0.9 to 4.5 m, y -1.8 to +1.35 m, three tags. Confirm with the examiner on the day.
- Output goes to `laptop/mission_logs/<time>_drone22/` (events, tags, trajectory, summary). Takeoff, every detection and landing are printed with timestamps.
- It lands automatically on stale telemetry (3 s), leaving the area plus 0.5 m, Ctrl+C or any error. Time limit 600 s, then it returns and lands. [repo]
- Start with tag-free or `--detect-only` runs if you doubt the camera numbers. Fly a first mission with `max_mission_s` shortened if you want a safe abort.

---

## 7. Emergency [repo: TEST_GUIDE]

- **RC pilot first.** Switch out of Offboard, or use the kill switch. The ESP32 never overrides this. After an RC landing the log shows `Disarmed without CMD_LAND` and it is ready for the next START.
- **Ctrl+C** after START sends CMD_LAND three times: it aborts arming, or lands even mid-climb. Stale telemetry (3 s) does the same. Ctrl+C before START sends nothing.
- If the script or laptop dies without sending LAND, the drone keeps hovering: **use the RC**.
- **WiFi loss:** the ESP32 holds position, and after 3 s sends a non-forced disarm. PX4 normally refuses this in the air (`Disarming denied: not landed`), so take over with the RC.
- `Still armed after 20 s - land with RC`: take over and land with the RC.
- Restart rules on the day: any restart needs the examiner's approval and may cost marks.

---

## 8. Things to measure before the demo

- [ ] **Camera lens position and pitch** on the built drone: forward, right and down from the FC centre (the PX4 position origin), and the 45-degree tilt. Put them into `laptop/tag_mission.yaml` (`camera:` block). Wrong values shift every tag position: tag accuracy is 15% of the demo.
- [ ] **Max range and viewing angle** for a real tag (`detection.max_range_m`, `min_view_elevation_deg`). Currently 3 m and 30 degrees, both assumed.
- [ ] **Detection latency** (`detection.latency_s`, currently 0): the tag packet lags the video.
- [ ] **Real cruise speed**: fly once and read "max horizontal speed" from `analyze_flight.py`. Do not change `MPC_XY_VEL_MAX` without asking your instructor.
- [ ] Standoff length, the FC side (above or below the frame), and the motor screw depth.
- [ ] Tape-measure the tag positions in the arena if you can, to use `--truth`.

---

## 9. After every flight

- [ ] Land, disarm, battery out. Check the battery temperature and motor heat.
- [ ] **Download the `.ulg`**: QGroundControl, Analyze Tools, Log Download, or the SD card's `log/` folder. [repo]
- [ ] Analyse it: [repo]
  ```powershell
  & $py laptop/analyze_flight.py --ulg C:\path\flight.ulg --run laptop\mission_logs\<run folder> --truth 1:1.35,-1.35
  ```
  It writes `flight_overview.png` and a written summary into the run's `analysis` folder. Not yet tried on a real `.ulg` from your drone. If it says the clock sync is `ambiguous`, do not trust the event markers and pass `--offset-s`.
- [ ] Keep the run folder and `.ulg` for the report: the AI usage declaration needs representative prompts, how you verified the results, and the AI's mistakes.

---

## 10. Demo-day checklist (4:15 pm, 45 minutes in all)

Before you go in: battery charged and spare, props and spare props, RC charged, laptop charged, the correct drone ID, the repo up to date (and pushed).

In the room: laptop IP and WiFi, firewall, `--plan-only` against the announced area and tag count, edit `tag_mission.yaml`, link check, one hover, then run the mission. Keep the `Ctrl+C` and RC ready. Run the post-flight analysis afterwards.

---

## What this guide cannot tell you
- Your PX4 parameters (airframe, EKF, optical-flow settings, failsafes) and the radio setup: I have not seen them.
- Whether the firmware on the ESP32 is the build in this repo, or which drone ID it carries.
- Whether the current drone still flies as it did in week 7. The repo has never been flown with these laptop-side changes.
