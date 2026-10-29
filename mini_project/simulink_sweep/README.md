# Sweep path through the course Simulink quadrotor

Flies the mission's real sweep (8 waypoints plus return, from `laptop/tag_mission.yaml`) through the
course model `trajectory/position_control/PosControl_Sim_RPT.slx` and reports tracking, timing and
whether the camera model would have seen each of the six possible tag spots.

**What it is:** a dynamics check with the course's position and attitude controllers.
**What it is not:** PX4, the ESP32 firmware, WiFi, the real camera or the optical-flow sensor.

## Run

```powershell
python mini_project/simulink_sweep/export_plan.py          # writes plan.json from the mission config
```
Then in MATLAB R2025b (needs Simulink, Aerospace Blockset, Optimization Toolbox):
```matlab
run('mini_project/simulink_sweep/run_sweep_sim.m')         % about 3 to 5 minutes
```
Outputs go to `results/`: `sweep_topview.png`, `sweep_timehistory.png`, `sweep_summary.json`,
`sweep_waypoints.csv`, `sweep_tag_coverage.csv`.

## Assumptions (not measured on the drone)

| Setting | Value | Where |
|---|---|---|
| Cruise speed | 0.2 m/s | `tag_mission.yaml` (really PX4 `MPC_XY_VEL_MAX`, unread) |
| Horizontal acceleration | 0.5 m/s^2 | `run_sweep_sim.m` |
| Climb / descent | 0.3 m/s, 0.5 m/s^2 | `run_sweep_sim.m` |
| Yaw rate / acceleration | 0.8 rad/s, 2.0 rad/s^2 | `run_sweep_sim.m` |
| Motion | stop at every waypoint, turn in place, then fly | `sweep_reference.m` |
| Camera | 45 deg, 0.070 m forward, 0.008 m down (CAD placeholders) | `tag_mission.yaml` |
| Tag spots | 3 x 2 grid, 0.45 m inside the search area (diagram, not measured) | `export_plan.py` |

## Results of the run on 29 Oct 2026 (R2025b)

| Quantity | Value |
|---|---|
| Takeoff command to landing command | 123.9 s (includes the 12 s takeoff wait) |
| Max / RMS horizontal tracking error | 0.012 m / 0.003 m |
| Max simulated speed | 0.271 m/s (35% above the 0.2 m/s reference, 3.6 s in total above 110%) |
| Max tilt | 4.4 deg |
| Every waypoint inside the 0.25 m arrival radius | yes, about 1.4 s before the reference arrives |
| Tag spots in the camera model's view | 6 of 6, each in view for 3.7 to 6.4 s |

## Reading the numbers

- **Time:** the Python mission simulator reports about 93 s because it turns instantly and has no
  acceleration. Turns and accelerations here add about 30 s. The real mission sends the next waypoint as
  soon as the drone is inside 0.25 m, which would save roughly 1.4 s per waypoint (about 13 s).
- **Speed overshoot:** a property of the course controller at the start of each leg. PX4 will behave
  differently; do not quote it as a property of the real drone.
- **Coverage:** geometric only (field of view, range 3 m, 30 deg minimum viewing angle, heading-only
  geometry, tags flat on the floor). It does not test that the detector decodes the tag.
- The Simulink `Demux4` warning is the same harmless one reported in the assignment run.
