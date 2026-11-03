# Drone bring-up guide: from parts to the graded run

Updated **Tue 3 Nov 2026** for the **flight test today at 2:00 pm** (start with the "Flight test today" section below). Final graded run: **Thu 12 Nov 2026, 4:15 to 5:00 pm**. The individual assignment is due **Fri 6 Nov, 11:59 PM**.

This guide compiles the lab manuals (Labs 1, 2, 3 and 5), your own Lab 2, 3 and 5 reports, the repo guides, and the mission code, into one step-by-step order. It replaces `PREFLIGHT_GUIDE.md` (older, written before I had read the labs).

Where each step comes from:
- **[Lab 1] [Lab 2] [Lab 3] [Lab 5]** = the lab manuals in `C:\cde1302\Labs\`.
- **[Your Lab n]** = values you recorded in your own reports (`Lecture Notes\`).
- **[repo]** = `README.md`, `laptop/TEST_GUIDE.md`, `waypoints/WAYPOINT_FLIGHT_TEST_GUIDE.md`, `QGC_MAVLINK_BRIDGE.md`.
- **[new]** = from the mission code written for the final run.
- **[general]** = ordinary PX4 / QGroundControl practice that the labs and repo do not spell out. I have not seen your PX4 parameters, so treat these as things to check.
- **[check]** = I could not settle this from the files. You must confirm it on the real drone.

The WiFi password is in the Lab 2 handout (section 4.3, step 3). I have left it out of this file because the repo is public.

---

## TODAY: flight test at 2:00 pm (Tue 3 Nov), step by step

Repo is merged with `origin/main` (3 Nov). What changed upstream: **firmware is unchanged** (only a comment in `main/wifi_task.c`), so **do not reflash**. `tag_stream.py` is now the short student viewer (outline, ID and drone-frame position per tag); the old `--detail` tuning view moved to **`tag_stream_legacy.py`**. `tag_debug.py` now takes the camera mount from `apriltag_host.py` (`CAM_PITCH_DEG`, `CAM_POS_M`). Tests: 195 passed, 2 skipped; `apriltag_host.py` prints `ncodes = 22`.

**Before 2:00 (at your desk, about 30 min)**
1. [ ] Book/confirm the supervised slot and area with the instructor. Assign the four roles (section 0, rule 7).
2. [ ] Battery charged, plus a spare; RC charged; spare props; laptop charged and plugged in.
3. [ ] Hand-check the drone with props **off** (section 1.6): guards on, wiring, FC arrow forward, flow sensor facing down with a clear lens, camera seated, cables clear of the props, antenna taped down, battery strapped. Get the TA/GA check before connecting the battery.
4. [ ] Laptop: join `TP-Link_E95A`, static IP `192.168.1.(100+ID)` (drone 22 = `192.168.1.122`), network set to Private, firewall allows UDP 5005/5008/5009 and QGC 14550. Confirm your **drone ID** is the one on the ESP32.
5. [ ] `cd C:\cde1302` then `git status` shows branch `krish-mission`; tests: `cd laptop; python -m pytest tests -q`.

**At the flight area, props OFF**
6. [ ] RC on, SB and SC safe, **kill switch SC engaged**. Power the drone, wait for ESP32 + FC to boot.
7. [ ] QGroundControl (UDP 14550): vehicle appears, **status Ready, no critical warnings**, position and distance sensor valid (sections 4.2, 4.6, 4.8). Move the drone by hand over the floor: the distance reading and attitude must follow.
8. [ ] `python laptop/check_px4_esp32_link.py --drone-id 22` gives `RESULT: PASS` (close QGC first if it holds 14550), then `python laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only` shows `Navigation state: IDLE` and packet age well under 1 s.
9. [ ] Camera: `python laptop/tag_stream.py --esp-ip 192.168.1.222`. Hold a tag16h5 tag at 0.5 to 1 m: outline and ID appear.
10. [ ] Props-off arm test (section 5, step 4): type `ARM-22`; expect `Takeoff timeout`, not a refusal.

**Props ON (only with the instructor's OK), announce "Arming in 3...2...1"**
11. [ ] Fit props and guards, check direction per motor. Everyone outside the safety zone. Pilot holds the RC.
12. [ ] Manual Position-mode hover first (section 6.3 and 6.4, A to G). If it drifts, stop and fix before any autonomous run.
13. [ ] Autonomous hover: `python laptop/simple_arm_takeoff_land.py --drone-id 22`, type `ARM-22`. Expect Offboard, armed, steady hover at 0.5 m, then land.
14. [ ] Waypoints (section 7, step 2) only if step 13 was clean.
15. [ ] Tag mission (section 7, step 5): `--plan-only` first, then `python laptop/run_tag_mission.py --drone-id 22` (type `TAKEOFF-22`). Only if 13 and 14 were clean and the instructor agrees.

**After each flight**: land, disarm, kill switch SC, props stopped, battery out, check temperatures. Download the `.ulg`, keep the `laptop/mission_logs/` folder, note anything odd (section 9).

**Open items to flag today**: (a) camera offsets are placeholders: `laptop/tag_mission.yaml` uses forward 0.070 / down 0.008 m, while `apriltag_host.py` `CAM_POS_M` is (0.02, 0, 0), so tag positions from `tag_stream.py` and from the mission differ until you measure the real lens and update both; (b) your PX4 parameters have not been checked in this repo; (c) nothing here has flown with the latest laptop code.

---

## 0. Rules you must follow first (from Lab 1 and Lab 3)

These come before any step below.

1. **No free flight unless the teaching staff authorise and supervise it, and only in the designated flight area.** [Lab 1] Book flying time with your instructor *now*. The graded area (Studio 1, E2A) is not somewhere you can test whenever you like.
2. **Propellers off** for configuration, programming, calibration and every communication test. Fit them only when staff say so. Guards on before any flight. [Lab 1]
3. **Kill switch (RC switch SC) engaged whenever the UAV is powered and not flying.** After landing: disarm, then engage the kill switch. [Lab 1, Lab 3]
4. **Battery:** inspect before use; never use a swollen, punctured, leaking or hot one; use only the approved charger, in the designated area, never unattended; disconnect right after the test; connect only with permission. [Lab 1]
5. **Announce before arming:** "Arming in 3...2...1" and "Starting mission in 3...2...1". Everyone outside the safety zone. Never arm on a desk. [Lab 1]
6. **Never catch a falling or flying UAV by hand.** [Lab 3]
7. **Four roles for every flight test:** pilot / safety operator (holds the RC), ground-control operator (laptop and QGroundControl), observer (watches people and obstacles), recorder (notes warnings and results). [Lab 1]
8. **Emergency:** switch SB to Auto Land, disarm after touchdown, then kill switch SC. Use SC at once if a hazard is worse. [Lab 3]

---

## 1. Parts and assembly (Lab 1)

### 1.1 Check the parts [Lab 1, Table 5.1]
Frame (1), prop guards (4), battery and camera mount (1), optical-flow sensor mount (1), ESP32-S3 Sense cover left and right (1 set), ESC + FC acrylic cover (1), ESC + FC board (1), motors (4), optical-flow sensor (1), ESP32-S3 Sense (1), ELRS receiver (1), M2 x 30 nylon standoffs (4), M2.5 x 15 nylon standoffs (4), M2.5 x 30 nylon standoffs (4), M2 nylon nuts (4), M2.5 nylon nuts (4), M2 x 10 screws (12), M2 x 15 screws (4), M3 x 15 screw (1), M3 flat washers (3), M3 spring washer (1), M3 nut (1).

How this matches the 3D-printed parts your professor gave you: "battery and camera mount" is the bottom plate (`BaseMountExtended_2.STL`), the optical-flow mount and cover are the `MTF-02P AIO Mount` and `Cover`, and the ESP32 cover is `Left (Open)` and `Right (Open)`.

**[check]** My earlier analysis found that the holes in `BaseMountExtended_2` (four holes, Ø2.58 mm, 30 x 67.1 mm apart) do not line up with the holes in the `main_frame.stl` you described. Lab 1 says the bottom plate goes on the frame with four **M2.5 x 30 mm** standoffs, which fits the Ø2.58 holes. So **hold the printed plate against your printed frame before you tighten anything.** If the four standoff holes do not line up, stop and ask the instructor which plate and frame pair belong together. Do not force it.

### 1.2 Before you start [Lab 1, 6.1]
Battery disconnected. All propellers removed. Stable work surface. Do not over-tighten. Do not force connectors. Check connector orientation. Do not pull on wires. Cable ties only where needed. Cables must never reach the propeller area.

### 1.3 Frame and motors [Lab 1, 6.2]
1. Find the front of the UAV and the four motor positions; confirm the motor numbering and rotation convention (section 4.7 checks this in QGroundControl).
2. Fit the motors to the frame with the **prop guard between the frame and the motor**, using **M2 x 10 screws**. The kit has 12 of these: 4 go to the flow sensor and 8 to the motors (2 per motor).
3. Route the motor cables; none may be trapped or stretched.

### 1.4 Flight controller [Lab 1, 6.3]
1. Fit the four vibration isolators into the four holes of the flight controller.
2. Mount the FC on the frame on the side **opposite the motors**, with **M2 x 15 screws and M2 standoffs**. The screws go through the frame and isolator from the bottom side and are tightened with the standoffs.
3. **The arrow printed on the FC must point to the front of the frame.**
4. Check: correct orientation, firm mounting, isolators fitted and not over-tightened, no strain on cables, USB port still reachable.

### 1.5 Sensors and bottom plate [Lab 1, 6.4]
1. **Optical flow:** plug in the flow cable (the middle one in its box). Fit the sensor between its mount and cover with **four M2 x 10 screws**. It must face straight down with the lens clear.
2. **ESP32:** pass the antenna cable through the left side of its mount, connect it to the ESP32, put the camera module on, fit the heatsink to the back of the ESP32 inside the case, close the left and right covers, then fix the module to the bottom plate with the **M3 x 15 screw and M3 nut**, tilted as in the lab figure. Put washers in the middle to stop it wobbling.
3. Fit the bottom plate to the frame with the **four long M2.5 x 30 standoffs and M2.5 nuts**, with the ESP32 on the **front** side of the frame. The short M2.5 x 15 standoffs are the landing gear.
4. Stick the ESP32 antenna down with its double-sided tape, away from the propellers and from conductive material.
5. Connect all cables. Fit the acrylic top cover on the four M2 standoffs on the FC, with four M2 nuts.
6. Secure every cable with ties so none is ever in the propeller sweep.

### 1.6 Prop guards and wiring [Lab 1, 6.6 and 7]
- Guards: all four fitted, firm, not touching the motors, not blocking sensors or the props.
- Wiring: no bare conductor, no damaged or sharply bent wire, connectors fully in, polarity connectors the right way round, signal and power cables on the intended ports.
- Connections to confirm: battery connector, FC power, motors and ESCs, receiver, optical-flow sensor, ESP32.
- **Do not connect the battery until the TA or GA has checked the UAV.**

### 1.7 Weigh and balance [general]
Weigh the built drone. Balance it on a finger under the FC to find the centre of gravity; it should sit over the middle of the frame.

---

## 2. Laptop setup (Lab 5 pre-lab, README)

1. Python environment: **[Lab 5]** `conda env create -f laptop/environment.yml`, then `conda activate cde1302-laptop`. On this laptop the environment already exists at `C:\cde1302\envs\cde1302-laptop`.
2. Install the C compiler in it: `conda install -c conda-forge gcc`. Check it: `python laptop/apriltag_host.py` must print `tag16h5 ncodes = 22 (expected 22)`. **[checked 29 Oct: it does.]**
3. Keep the repo and conda in paths with no spaces: `C:\cde1302`, `C:\miniforge3`. [Lab 5]
4. **Firewall:** allow Python through Windows Defender Firewall when it asks; set the drone WiFi to **Private**; allow UDP 5005, 5008, 5009 (Python) and 14550 (QGroundControl). **Do not use WSL.** [Lab 5, repo]
5. **Network:** join the lab WiFi `TP-Link_E95A` (password in the Lab 2 handout). Gateway `192.168.1.1`, netmask `255.255.255.0`. Give the laptop the static address **`192.168.1.(100 + drone ID)`**: drone 22 is `192.168.1.122`. The drone is at `192.168.1.(200 + ID)`: drone 22 is `192.168.1.222`. The ESP32 sends telemetry only to the laptop address. [README, Lab 2]
6. **[check] Your drone ID.** Examples use 22; Lab 5 uses 2. Use the number flashed into your ESP32. Ask your TA if you are unsure.
7. Tests still pass: `cd laptop; python -m pytest tests -q` (195 passed, 2 skipped on 3 Nov). The simulation still runs: `python laptop/run_tag_mission.py --drone-id 22 --simulate`.

---

## 3. ESP32 firmware and RC (with the instructor)

Lab 2 (4.2) says to **ask an instructor** to show you three things; do not improvise these:
1. **Bind, configure and calibrate the RC** (RadioMaster Pocket). Receiver LED steady blue = connected. Do not touch the trims. [Lab 2, Lab 3]
2. **Flash the ESP32 and set the static IP** for the ground-control laptop. Lab 5 (4.1) repeats this: "approach a GA to update ESP32 firmware". [Lab 2, Lab 5] The firmware is already flashed on the drone in the normal case. [README]
3. **Attach and secure the battery.** [Lab 2]

RC layout used in the labs: **Mode 2**; left stick = throttle (up/down) and yaw (left/right); right stick = pitch (up/down) and roll (left/right); **SB = flight-mode switch; SC = kill switch.** [Lab 2, Lab 3]
**Warning:** your submitted Lab 3 table lists roll, yaw and pitch on different sticks from this. Use the Lab 2 description (it matches Mode 2) when you fly, and check against your transmitter.
Your Lab 2 report recorded the mode switch on **channel 6**. [Your Lab 2]

If you must flash yourself (only if the instructor says): `. C:\esp-idf\export.ps1`, `cd C:\cde1302`, `idf.py menuconfig` (Drone Configuration: drone ID, WiFi, host IP), then `idf.py -p COM7 flash monitor`. Close QGroundControl first. **It has never been tried on this laptop**, and the serial monitor resets the ESP32 each time it opens. [repo, new]

---

## 4. QGroundControl: connect, configure, calibrate (Lab 2)

Props off for all of this.

### 4.1 Connect over USB (configuration)
1. Remove the battery (unless told otherwise). Connect the FC by USB-C. Start QGroundControl.
2. Wait for the parameters to finish loading and the vehicle icon to appear. [Lab 2, Method A]
3. **Save the parameters to a file now** (Vehicle Setup, Parameters, Tools, Save to file) so you can always go back. [general]

### 4.2 Connect over WiFi (monitoring and flight)
1. Battery installed. RC on, **kill switch engaged**. Laptop on the lab WiFi.
2. Power the UAV and wait for the ESP32 and FC to finish starting.
3. QGroundControl: Application Settings, Comm Links, Add. **Name `ESP32`, Type `UDP`, Port `14550`.** Save, Connect. [Lab 2, Method B]
4. Confirm the vehicle icon, flight mode, battery and attitude display appear.
5. If the vehicle does not appear: the alternative setup in `QGC_MAVLINK_BRIDGE.md` uses listening port 14550 and target host = the ESP32's IP, target port 8888. Check that the laptop IP is right, that the WiFi is shared, and that UDP 14550 is allowed through the firewall. [repo]

### 4.3 Vehicle configuration [Lab 2, 5.1]
Open Vehicle Configuration, then Summary and Airframe. Do **not** change the airframe unless told to. **Your drone, from your Lab 2 report:** vehicle type Generic Quadcopter, airframe Quadrotor X, autopilot PX4, mode switch on channel 6, **3 cells**. Check yours still matches. [Your Lab 2]

### 4.4 Sensor calibrations [Lab 2, 5.2 to 5.4]
Do them in this order, with the drone still and on a stable surface. Do not touch the table.
1. **Gyroscope:** Sensors, Gyroscope. Start, and do not move the UAV until the bar completes.
2. **Accelerometer:** Sensors, Accelerometer. Follow the orientation QGroundControl shows (level, sides, nose down, nose up, upside down). Hold still in each until accepted. Do not press on or bend the frame.
3. **Level horizon:** Sensors, Level Horizon, on a **genuinely level** surface, after the accelerometer.
4. Compass: your Lab 2 listed a compass among the sensors. The FC has no onboard compass; an external one is supported. [Lab 1] If QGroundControl asks for compass calibration, do it only as the instructor shows you.

### 4.5 Attitude check [Lab 2, 5.5]
Keep it disarmed, props off. Roll right and left, pitch nose down and up, turn clockwise and anticlockwise. The horizon on screen must move the same way. Return it level.

### 4.6 Distance-sensor check [Lab 2, 5.6]
Analyze Tools, MAVLink Inspector, expand `DISTANCE_SENSOR`. The reading must update as you lift the drone over the floor, and the orientation must match the way the sensor faces. **Your recorded values:** update rate 10 Hz, minimum distance 2, maximum 600 (centimetres, so 0.02 to 6 m), orientation 25 (which is downward-facing). Do not change distance-sensor parameters. [Your Lab 2]

### 4.7 Motor numbering and spin direction [Lab 2, 5.7]
1. Props **off**. Put a small piece of tape on motor 1.
2. Vehicle Configuration, Actuators. Attach the battery. Switch **Actuator Testing** on.
3. Raise the **Motor 1** slider slowly. Motor 1 must turn **counter-clockwise** as in the diagram on screen.
4. If it is wrong: switch testing off, click **Set Spin Direction 1** for motor 1, test again; if still wrong use **Set Spin Direction 2**. Then check motors 2, 3 and 4 against the diagram.
5. (PX4's usual quad-X layout, from memory, for a sanity check only: motors 1 and 2 counter-clockwise, 3 and 4 clockwise. **Follow the diagram on your screen, not this note.**) [general]

### 4.8 Pre-flight status [Lab 2, 5.8]
On the bench, disarmed, read the notification area and the vehicle health summary. **Record every warning.** Your Lab 3 connection record shows the entries **"Hold"** and **"Not ready"**. [Your Lab 3] I can't tell which item "Not ready" belongs to, but the vehicle must read **ready** before any flight, with no critical warnings. Ask the instructor about anything left unresolved.

---

## 5. Bench tests with the laptop (props off) [Lab 5, repo]

All from `C:\cde1302`. Drone ID 22 in the examples.

1. **Communication check** (close QGroundControl first if it is on UDP 14550):
   - `python laptop/check_px4_esp32_link.py --drone-id 22` should end with `RESULT: PASS`.
   - `python laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only`. Expect `Connected to drone 22`, the ESP32 IP, `Navigation state: IDLE` and a packet age well under 1 s. [Lab 5]
   - If you get no telemetry: wrong drone ID, wrong laptop IP, or the firewall. [Lab 5 appendix]
2. **Serial monitor** (`idf.py -p COMx monitor`, opened *before* arming): expect `UART1 ready ... 921600 baud`, `Telemetry valid` and `Waiting for CMD_START from laptop...`. [repo]
3. **Camera and tags:** `python laptop/camera_stream.py --esp-ip 192.168.1.222` then `python laptop/tag_stream.py --esp-ip 192.168.1.222` (for the tuning view with rejects and the drone's own detections use `tag_stream_legacy.py --detail`). Hold a printed tag16h5 tag at about 0.5 to 1 m. You should see the outline, tag ID and distance. **Your Lab 5 readings: about 10 fps, ESP frame age 130 ms, laptop latency 167 ms.** [Your Lab 5] Use `--fps 5` in flight.
4. **Props-off arm test:** `python laptop/simple_arm_takeoff_land.py --drone-id 22`, then type `ARM-22`. With props off, expect `Takeoff timeout: NED z=0.0x` in place of `Altitude reached`. If it says `OFFBOARD refused` or `arming refused`, PX4 rejected it: read the QGroundControl messages. [repo]
5. **Walk-through of the tag pipeline, no flight** [new]: `python laptop/run_tag_mission.py --drone-id 22 --detect-only --truth "1:1.35,-1.35 2:4.0,-1.35 3:2.7,0.9"`. Carry the drone by hand along the sweep over a tag on the floor, then press Ctrl+C. It prints each tag position and the error against your tape-measured positions. This is how you check the camera numbers in `tag_mission.yaml`.

---

## 6. RC and manual flight tests (Lab 3), with supervision

Do this **before** any autonomous flight. It proves the position hold and the optical flow, which the autonomous mission depends on.

### 6.1 Remote-control inspection [Lab 3, 4.2]
Remote battery OK; SB and SC both in safe positions at power-on; receiver connected (steady blue LED); inputs detected; **SB** flight-mode switch works; **SC** kill switch works; failsafe settings checked.

### 6.2 Connection record [Lab 3, 4.3]
Battery percentage, flight mode shown, **velocity estimation valid**, **distance-sensor reading**, any warnings. QGroundControl should show the vehicle ready.

### 6.3 Final checklist before arming [Lab 3, 6.1]
Flight area clear; remote on; correct UAV connected; battery secure; props correct; guards secure; **optical flow operating**; **distance measurement valid**; **position estimate valid**; **Position mode selected**; no critical warning; pilot knows the disarm procedure; safety observer ready; **instructor approved.**

### 6.4 Test sequence in Position mode [Lab 3, 6.2 to 6.8]
A. Arm and watch the motors briefly. Disarm at once for odd sound, vibration or behaviour.
B. Controlled take-off to the height the instructor sets; release pitch and roll sticks and see whether it holds position.
C. Hover for the set time; note drift, altitude change, oscillation, heading change, optical-flow warnings.
D. Small forward and backward moves; release the stick and see it stop and hold.
E. Left and right.
F. A small yaw, then check heading and position hold.
G. Controlled landing; disarm; **kill switch SC**; wait for the props to stop; disconnect the battery.

**Pass:** it takes off vertically, holds a stable hover with little drift, and stops when you release the sticks. If it drifts, oscillates or wanders: land with the RC and check the CG, the flow sensor, the floor texture and lighting, and vibration. Do not go on. [Lab 3, Lab 5 appendix]
Optical flow works poorly on dark, reflective, transparent or plain floors, in poor light, and if the lens is blocked. [Lab 1]

---

## 7. Autonomous flight tests (Lab 5), with supervision

Step order; do the next only after the previous one is clean. Run every command with `--confirm` where it exists.

1. **Arm, take off, hover, land:** `python laptop/simple_arm_takeoff_land.py --drone-id 22`, type `ARM-22`. In QGroundControl: mode changes to **Offboard**, vehicle armed, climbs to about **0.5 m**, steady hover, then land, disarmed. Expected ESP32 lines: `CMD_START received`, `OFFBOARD mode confirmed`, `Armed confirmed`, `Taking off to 0.5 m AGL`, `CMD_HOLD received`, `CMD_LAND received from laptop`. [Lab 5]
2. **Waypoints:** `python laptop/simple_waypoint_mission.py --drone-id 22 --waypoints-file waypoints/waypoints_example.txt --takeoff-wait 5.0 --arrival-timeout 30.0 --confirm`, type `MISSION-22`. Horizontal drift should stay under 0.3 m; the waypoint acceptance radius is 0.25 m. [Lab 5, repo]
3. **Trajectory tracking** (optional, not needed for the graded task): `send_trajectory.py` with the ellipse CSV, drone placed as in the Lab 5 figure. [Lab 5]
4. **Camera and AprilTags in flight** at `--fps 5`. Tags never affect the flight. [repo]
5. **The mission itself:** `python laptop/run_tag_mission.py --drone-id 22 --plan-only`, then without `--plan-only` (type `TAKEOFF-22`).

**If anything goes wrong:** RC pilot first (SB to Auto Land, or SC). Ctrl+C after the start command sends land three times; stale telemetry for 3 s does the same. If the script or laptop dies without sending land, the drone keeps hovering, so use the RC. If WiFi is lost the ESP32 holds position and tries a non-forced disarm after 3 s, which PX4 refuses in the air, so take over with the RC. [Lab 5, repo]

---

## 8. Settings that decide your marks (`laptop/tag_mission.yaml`)

| Setting | Now | What to do |
|---|---|---|
| `search_area` | x 0.9 to 4.5 m, y -1.8 to +1.35 m | From the Flying Area PDF. Confirm the start position with the examiner. |
| `expected_tags` | 3 | Three tags per the PDF. |
| `camera` pitch, forward, right, down | 45 degrees, 0.070, 0, 0.008 m | **Placeholders from CAD.** Measure the real lens: forward, right and down from the FC centre (the PX4 origin). The sensor-to-mount geometry changes if you swap holders. Wrong values shift every tag position. Tag accuracy is 15% of the demo. |
| `detection.latency_s` | 0 | **Measure.** Your Lab 5 numbers (130 ms frame age, 167 ms laptop) were for the video stream. The repo notes say the drone's own tag packets lag the video by about 1 s. |
| `max_range_m`, `min_view_elevation_deg` | 3 m, 30 degrees | Assumed. Test with a real tag. |
| `cruise_speed_mps` | 0.2 | Assumed. The real speed is set by PX4 `MPC_XY_VEL_MAX`; fly once and read the speed from `analyze_flight.py`. Ask your instructor before changing PX4 limits. |
| Standoff length | 30 mm | From Lab 1 (M2.5 x 30). This replaces my earlier 25 and 32 mm guesses. |

Heading: the drone must start at the launch dot **facing the search area** (confirmed with you). PX4's local origin is where the drone was powered on, so power it on there. [repo]

---

## 9. After every flight

- Land, disarm, **kill switch**, wait for the props to stop, battery out. Check battery and motor temperature. [Lab 3]
- Download the `.ulg`: QGroundControl, Analyze Tools, Log Download, or the FC's SD card `log/` folder. [Lab 5 appendix, repo]
- `python laptop/analyze_flight.py --ulg <file.ulg> --run laptop\mission_logs\<run folder> --truth 1:1.35,-1.35`. Not yet tried on a real log. [new]
- Keep the run folder and `.ulg` for the report. The report also needs an **AI usage declaration**: tools, representative prompts, how you verified, mistakes found.

---

## 10. Plan for the next 10 days

Flight tests need supervision and the lab area, so ask for slots **today**.

| When | What |
|---|---|
| Mon 2 to Tue 3 Nov | Print the professor's camera and flow parts and test-fit them. Assemble per section 1. Weigh and balance. Ask your instructor for supervised flying slots. |
| Wed 4 Nov | Laptop setup check (section 2). RC bind and QGroundControl calibration with the instructor (sections 3 and 4). Bench tests (section 5). |
| Thu 5 Nov | Manual Position-mode flights (section 6) if you have a slot. **Finish and submit the assignment.** |
| Fri 6 Nov | Assignment due 11:59 PM. |
| Sat 7 to Sun 8 Nov | Rest of the bench work. Measure camera numbers with `--detect-only`. |
| Tue 10 to Wed 11 Nov | Autonomous tests (section 7): hover, waypoints, then two or three full missions with logs. Run the `.ulg` analysis. Fill the report. Commit and push the code. |
| Thu 12 Nov, 4:15 pm | Final run (30 min including setup, plus 5 min individual questions). Report due within 48 hours. |

### Run-day timeline (45 minutes) [suggested]
- Before you go in: charged battery plus a spare, spare props, charged RC and laptop, correct drone ID, repo up to date.
- 0 to 10 min: laptop IP and WiFi, firewall, QGroundControl connection, the preflight checklist in 6.3, the announced area and tag count into `tag_mission.yaml`, `--plan-only`.
- 10 to 15 min: link check, then one short hover if the examiner allows.
- 15 to 25 min: run the mission (type `TAKEOFF-22`). Keep the RC in hand. Say "Starting mission in 3...2...1".
- Remainder: a second trial only with the examiner's approval. Any restart needs approval and may cost marks.
- Be ready for the 5-minute individual questions: you must explain the whole system.

---

## 11. Troubleshooting [Lab 5 appendix and repo]

| Symptom | Check |
|---|---|
| No telemetry in Python | Laptop IP, `CONFIG_HOST_IPV4_ADDR`, UDP 5005, drone ID, firewall. |
| Offboard works but arming fails | Read QGroundControl pre-flight messages: local position, safety switch, sensor calibration, estimator status, battery warnings. |
| Arms but does not take off | ESP32 setpoint stream active; altitude estimate valid; NED altitude sign correct. |
| PX4 leaves Offboard at once | `send_setpoint()` still running; UART link ESP32 to PX4 stable (TX/RX crossed, common ground, 921600 baud). |
| Drifts after take-off | Optical-flow quality, floor texture, lighting, rangefinder data, sensor orientation, vibration, EKF set-up. |
| Land command received but no landing | PX4 acknowledgement, flight-mode change, altitude validity, another app sending conflicting commands. |
| QGroundControl does not see the vehicle over WiFi | Laptop IP, same WiFi, UDP 14550 allowed, PX4 telemetry baud 921600, QGC UDP link settings. |

---

## 12. Master checklist (print this)

Before touching the battery
- [ ] Parts counted; frame, guards, motors, plate checked for cracks.
- [ ] FC arrow forward; FC on the side opposite the motors; isolators fitted.
- [ ] Bottom plate fits on the four M2.5 x 30 standoffs (checked, not forced).
- [ ] Flow sensor faces down, lens clear. Camera tilted, secure, lens clear.
- [ ] Antenna stuck down; all cables tied clear of the propellers.
- [ ] Wiring inspected; TA or GA has checked it.

Bench (props off)
- [ ] Laptop IP `192.168.1.(100+ID)`, WiFi, firewall, `apriltag_host.py` prints 22.
- [ ] RC bound and calibrated; SB and SC work; kill switch engaged; failsafe checked.
- [ ] QGroundControl: airframe, gyro, accelerometer, level horizon, attitude check, distance sensor, motor directions, **status Ready, no critical warnings.**
- [ ] `check_px4_esp32_link.py` PASS; `--monitor-only` shows low packet age; camera and tag stream working.
- [ ] `--detect-only` walk-through done; camera numbers in `tag_mission.yaml` measured.

Flight (supervised)
- [ ] Manual Position-mode hover and moves clean (section 6).
- [ ] Autonomous hover, waypoints, then mission, each clean (section 7).
- [ ] Logs and `.ulg` kept; analysis run.

Run day
- [ ] Section 10 timeline; roles assigned; spares packed; area and tag count confirmed with the examiner.

---

## 13. What I could not confirm

- **Your drone ID**, and whether the ESP32 firmware matches this repo.
- **Which plate and frame pair** your drone really uses (section 1.1).
- **Your PX4 parameters**: optical-flow and height settings, failsafes, the telemetry port, the speed limit. The labs and repo do not list them. Save your parameters file and compare with an instructor's.
- **The motor layout diagram** on your screen: follow it.
- **Whether QGroundControl needs a compass calibration** on your set-up.
- Anything flight-related has not been tried with the mission code on the real drone. The drone has not flown since week 7.
