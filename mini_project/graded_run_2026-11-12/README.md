# Graded run, 12 Nov 2026

Drone 1, generated-sweep trajectory mission (`laptop/tag_mission_demo_traj.yaml`). Screenshots are the laptop live map at the end of each run.

| Run | Screenshot | Mission log | Result |
|---|---|---|---|
| 1 | [run1.png](run1.png) | `20261112_164042_drone1` | 3/3 tags [2, 4, 6] in 148.0 s |
| 2 | [run2.png](run2.png) | `20261112_164515_drone1` | 3/3 in 149.5 s |
| 3 | [run3.png](run3.png) | `20261112_165155_drone1` | 2/3 [4, 6] in 150.6 s |

The logs are in `laptop/mission_logs/<log>/` (`summary.json`, `trajectory.csv`, `tags.csv`,
`observations.csv`, `events.csv`). That folder is git-ignored, so the logs exist only on the
flight laptop.

`laptop/mission_logs/20261112_165029_drone1` is also from this session: it stopped after 21 s
on a geofence breach at (-0.50, -0.91) and found no tags. Run 3 was flown after it.
