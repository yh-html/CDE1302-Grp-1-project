# Drone 22 quick setup guide (for a teammate)

Hi! This is the short version of what to do with drone 22. The graded run is **Thu 12 Nov, 4:15 to 5:00 pm**. Go through the steps in order and stop if any step fails. The full detail is in `mini_project/DRONE_BRINGUP_GUIDE.md` (skip its "TODAY: 3 Nov" section, it is out of date).

**Safety rules:** props **off** for every bench step. Kill switch **SC engaged** whenever the drone is powered and not flying. Fly only in the supervised area with the instructor's OK. Say "Arming in 3...2...1" out loud before arming.

---

## 1. Laptop setup (about 15 min, once)

1. Install **Python 3.10+**, **Git** and **QGroundControl**.
2. Install a C compiler. The tag viewer compiles the AprilTag library on its first run:
   - with conda: `conda install -c conda-forge gcc`
   - or install MinGW-w64 gcc (MSYS2 or WinLibs) and make sure `gcc` works in a new terminal.
3. Get the code and the Python packages:

   ```powershell
   git clone https://github.com/yh-html/CDE1302-Grp-1-project.git C:\cde1302
   cd C:\cde1302
   git checkout krish-mission
   python -m pip install -r laptop/requirements.txt
   python -m pytest laptop/tests -q
   ```

   The tests should pass (a couple of skips is fine).
4. Network: join WiFi **`TP-Link_E95A`** (the password is in the Lab 2 handout, section 4.3, step 3). Set a **static IP of `192.168.1.122`**, mask `255.255.255.0`. Set the network to **Private**.
5. Windows Firewall: allow Python on **UDP 5005, 5008 and 5009**, and QGroundControl on **UDP 14550**. Don't use WSL.

**Do not reflash the ESP32.** Its firmware is already correct. It is drone ID 22 at `192.168.1.222`.

## 2. RC check (props off)

1. Turn the RC on, put SB and SC in safe positions, with **SC (kill) engaged**. Then plug in the battery.
2. In QGroundControl: Analyze Tools → MAVLink Console → type `rc_input status`. It must show **RX bytes above zero** and a valid signal. A blue receiver LED on its own does **not** prove the RC works.
3. In Vehicle Setup → Radio, move the sticks and switches and check that the bars move. In Flight Modes, check SB changes mode and SC is the kill switch.
4. If the RC check fails, **stop** and ask the TA. Never fly without a working kill switch.

## 3. Drone and link checks (props off)

1. QGroundControl over WiFi (UDP 14550): the vehicle shows **Ready** with no critical warnings. Lift the drone by hand and tilt it: the attitude and the downward distance reading must follow.
2. Close QGroundControl, then run from `C:\cde1302`:

   ```powershell
   python laptop/check_px4_esp32_link.py --drone-id 22
   python laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only
   ```

   Expect `RESULT: PASS`, then `Navigation state: IDLE` with fresh telemetry. Press Ctrl+C to stop.

## 4. Camera and tag checks (props off)

1. `python laptop/apriltag_host.py` should print `ncodes = 22 (expected 22)`.
2. `python laptop/tag_stream.py --esp-ip 192.168.1.222` opens the live video. Hold a printed **tag16h5** tag 0.5 to 1 m from the camera: a green outline and its ID appear. Press `q` to quit.
3. Lay the tags on the floor. Run the detect-only mode, which **never flies**:

   ```powershell
   python laptop/run_tag_mission.py --drone-id 22 --detect-only
   ```

   Carry the drone by hand at about 0.5 m height past each tag. Look for `TAG_DETECTED` lines, then press Ctrl+C. You get a tag table, and a log folder appears in `laptop/mission_logs/`.

## 5. Flying (only with the instructor's OK)

Go one step at a time. If a step goes wrong (drifting, wobbling, losing height), land and stop.

1. Fit props and guards, and check each prop is on the correct motor. Everyone stands outside the safety zone. The pilot holds the RC the whole time.
2. **Manual hover** in Position mode with the RC. Take off, hover, land.
3. **Autonomous hover**: `python laptop/simple_arm_takeoff_land.py --drone-id 22`, then type `ARM-22`. It should take off to 0.5 m, hover and land.
4. **Mission dry run** (does not fly): put the drone at the start point, facing the search area.

   ```powershell
   python laptop/run_tag_mission.py --drone-id 22 --config laptop/tag_mission_demo_traj.yaml --plan-only
   ```

   Check that the planned path fits inside the area.
5. **Mission**: run the same command **without** `--plan-only` and type `TAKEOFF-22` when it asks. The drone flies a slow continuous sweep (0.12 m/s) and prints tags as it finds them. It returns and lands once it has found all 3. The pilot stays ready to take over.
   - If the flying room is the small practice room and not the official area, use `laptop/tag_mission_small_traj.yaml` instead.

## 6. After every flight

1. Confirm in QGroundControl that the drone has landed and is **disarmed**. The script waiting 8 s does not mean it has touched down.
2. Engage SC kill, wait for the props to stop, unplug the battery.
3. Keep the `laptop/mission_logs/<time>_drone22_.../` folder and download the PX4 `.ulg` log (QGC → Analyze Tools → Log Download). Send both to Krish.

## If something goes wrong

| Problem | Try |
|---|---|
| No telemetry / `check_px4_esp32_link` fails | Laptop IP is `192.168.1.122`? Firewall ports allowed? Close QGC (it holds port 14550). Power-cycle the drone. |
| Tag viewer says no compiler | Install gcc (step 1.2), open a new terminal. |
| No video in `tag_stream.py` | Check the `--esp-ip` is `192.168.1.222` and that UDP 5009 is allowed. |
| QGC shows "Not ready" | Don't fly. Read the warning, usually a calibration or a missing position estimate. |
| Drone drifts in hover | Land. Check the flow sensor lens is clean and facing down, and the floor has texture. |

Call or message Krish before changing any config or code.
