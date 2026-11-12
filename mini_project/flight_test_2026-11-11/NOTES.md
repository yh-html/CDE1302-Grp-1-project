# Flight tests, 11 Nov 2026 (drone 1, Studio 1 arena)

Evidence for the report: flight-data analysis and the diagnosis of abnormal behaviour. The
videos (`1945_111126.MOV`, `2005_111126.MOV`, `2015_111126.mp4`, `2030_1st run.mp4`,
`2031_2nd run.mp4`) stay in `C:\cde1302_course\photos\arena\`, because they are too large for git.

Before this day the camera was remounted to look straight down (pitch 90). See commit c9171d3.

## Runs

| time | config / code | result |
|---|---|---|
| 19:42 | 2-lane sweep, 0.5 m | 2/3. PX4 said it flew the rectangle and landed on the box; the video shows it went past the search area to the lockers and landed near the foam block. See below |
| 19:58 | detect-only hand carry | went to x = 5.44 and came back to within 0.11 m. y wander under 0.15 m |
| **20:02** | same | **3/3**, landed in the box. The return via CMD_GOTO spun the drone about 300 deg (bulge on the map) |
| 20:13 | known-tags hover mode (positions from the 20:02 run) | 3/3, but the tags were 0.37-0.64 m from the reference and the legs were diagonal. The mode was dropped |
| 20:26 | sweep | aborted: telemetry stale. 2-3 s Wi-Fi gaps all flight, with the live_map camera stream on |
| 20:29 | sweep, heading 0.1 rad at start | "Pilot took over using sticks" at 55.8 s, because it was heading for the left wall; then the kill switch |
| 20:41 / 20:45 | return flown as a trajectory (3bdb74b) | 3/3 (landed left of the box) and 2/3 (tag 6: 2 frames) |
| 20:54 | sweep height 0.8 m | wobblier. Battery died: telemetry stopped and the mission aborted at 124 s |
| 21:09-21:20 | flow sensor back in its original casing | several telemetry-stale aborts and one geofence abort while it was sorted out |
| 21:22, **21:25** | **original flow casing + sensor recalibration**, 0.6 m, right lane y = 1.40 | **3/3 both runs**. 21:25: 3/3 in 90.7 s, home at 0.11 m from the start. Map: `flight_maps/RECALIBRATED AND IT FLIES LETSGOOO.png` |

## 19:42: the EKF followed a wrong optical-flow measurement (`px4_logs/2026-11-11_1942_flight.ulg`)

- There were no xy, z or heading resets in flight. The gyro-integrated yaw stayed within +-3 deg, and flow quality was 77-93.
- Integrating the flow sensor's velocity reproduces the EKF path, so PX4 faithfully tracked a wrong flow measurement.
- After touchdown the estimate reset by 1.9 m in xy and -47 deg in heading. That is an artefact: there is no compass, so yaw is never aligned.

## 20:14: landing off the box (`px4_logs/2026-11-11_2014_flight.ulg`, PX4 log 84)

- The return used CMD_GOTO, and the firmware's rotate-then-fly turned the drone about 220 deg while it was still moving at about 0.3 m/s.
- The start heading was 8 deg, because there is no compass and "forward" is wherever the drone pointed at power-on.
- Fixes: the return is now a trajectory with the heading held, and an optional start-heading check (both in 3bdb74b). The check is off in the configs, because it refused takeoff with the drone square in the box.

## 20:29: pilot takeover (`px4_logs/2026-11-11_2029_run2.ulg`, PX4 log 87)

PX4 believed the drone was on the left lane (y = -1.34). On video it was close to the left wall. The RC override switched it to Position mode at 55.8 s, and the kill switch was used at 62.7 s.

## What fixed it

The drift was in the optical-flow measurement itself, not in the mission code. It went away after the flow sensor was put back in its original casing and the sensors were recalibrated. With that fix, the 21:25 run measured the tags almost exactly on the Flying Area PDF spots:

| tag | 21:25 estimate (x, y) | PDF spot |
|---|---|---|
| 2 | (1.29, -1.38) | (1.35, -1.35) |
| 4 | (2.73, -1.41) | (2.70, -1.35) |
| 6 (foam box) | (4.15, 1.03) | (4.05, 0.90) |

So the earlier tag 6 readings of y = 1.4-1.5, which the right lane was moved out for, were drift. The 1.40 lane still found tag 6 at 0.6 m height.

## Not logged

The flight controller stopped logging after the casing work. MAVLink SYS_STATUS reports logging not present, and LOG_REQUEST_LIST returns 0 logs, so the 21:22 and 21:25 flights have no `.ulg`. Reseat or replace the SD card before the graded run.
