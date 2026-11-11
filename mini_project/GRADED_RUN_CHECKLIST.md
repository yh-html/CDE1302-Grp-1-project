# Graded run checklist (drone 1)

This is the setup that flew 3/3 on 11 Nov at 21:25: config `laptop/tag_mission_demo_traj.yaml`, a 0.6 m sweep, with lanes at y = -1.35 and +1.40. Don't change the config on the day unless the announced area or tag spots differ.

The commands work from any folder. If one isn't found, run `laptop\install_commands.ps1` once and open a new terminal.

## The night before

- [ ] Charge the flight batteries, including a spare, plus the RC and the laptop. A flat battery ended the 20:54 run.
- [ ] **SD card in the flight controller.** On 11 Nov it stopped logging after the casing work, so there is no `.ulg` for the good runs. The report needs a PX4 log.
- [ ] Spare props, and the flow sensor in its **original casing** (the 3D-printed one caused the drift).

## At the arena (props off)

1. [ ] **Recalibrate the sensors in QGroundControl** (Vehicle Setup > Sensors). This is what fixed the drift on 11 Nov. Do it every time before flying.
2. [ ] In QGroundControl, check there are no preflight errors, and that the distance sensor reads the floor height and changes when you lift the drone. **Close QGroundControl afterwards.** It locks the USB port and competes for Wi-Fi.
3. [ ] Check the logs are recording: with QGC closed, the log list must not be empty. In QGC: Analyze Tools > Log Download > Refresh.
4. [ ] Join the drone Wi-Fi with the laptop at `192.168.1.101`. Sit near the router. Earlier runs aborted on Wi-Fi gaps.
5. [ ] Link check: `simple_arm_takeoff_land --drone-id 1 --monitor-only` should show `age` well below 1 s.
6. [ ] Plan check against the announced area: `run_tag_mission --drone-id 1 --config tag_mission_demo_traj.yaml --plan-only`
7. [ ] Put the drone square in the start box, facing +x (towards the far end). Power-cycle it there and **don't touch it after**.

## Fly

Terminal 1 (map without the camera stream, to save Wi-Fi):
```powershell
live_map --config tag_mission_demo_traj.yaml
```
Terminal 2:
```powershell
run_tag_mission --drone-id 1 --config tag_mission_demo_traj.yaml --wait-for-go
```
- [ ] `heading_rad` on `LINK_ESTABLISHED` should be about 0.00 or 6.28. If it isn't, check the drone is square in the box.
- [ ] Type `TAKEOFF-1`, wait for the hover, then type `GO`.
- [ ] The RC pilot keeps a hand on the sticks and the kill switch. `LAND` or Ctrl+C in Terminal 2 lands at once.
- [ ] Film it from behind the start box.

The program prints the timestamps the brief asks for: takeoff command (`TAKEOFF_CMD_SENT`), each detection (`TAG_DETECTED`, with ID, x, y and camera-frame pose) and landing command (`LANDING_CMD_SENT`), plus the final tag table.

## After

- [ ] Kill switch on, battery out.
- [ ] Download the `.ulg`. Close QGC first, or use QGC's Log Download.
- [ ] Keep `laptop/mission_logs/<time>_drone1/`, the `.ulg`, the live-map screenshot and the video for the report. The report is due Mon 16 Nov, 23:59.
- [ ] Analyse it: `analyze_flight --ulg <file.ulg> --run laptop\mission_logs\<run folder>`

## If something goes wrong

| symptom | cause seen on 11 Nov | do |
|---|---|---|
| `ABORT telemetry stale` | Wi-Fi gaps, or the ESP32 lost power (flat battery) | move the laptop nearer the router, run `live_map` without `--drone-id`, use a fresh battery |
| drifts sideways while the map looks fine | optical flow (casing / calibration) | land, recalibrate, check the flow casing |
| `TRAJECTORY_PREUPLOAD_NOT_STARTED` | first upload dropped a chunk | nothing: it re-uploads by itself |
| `TRAJECTORY_RETRY` 5x then `RETURNING_TO_START` | the pilot took over (ESP32 is no longer in control) | land with the RC |
