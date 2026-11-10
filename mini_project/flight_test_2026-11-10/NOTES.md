# Flight tests, 10 Nov 2026 (drone 1, Studio 1 arena)

Evidence for the report (flight-data analysis, abnormal-behaviour diagnosis). The videos
`1st flight test.mp4` (18:19), `1903.mp4`, `1922.mp4` and `1958.mp4` stay in
`C:\cde1302\photos of arena\` because they are too large for git (371 MB).

## Runs

| time | config / code | result |
|---|---|---|
| 16:15-16:25 | 4-lane sweep, PDF-ish area | 3/3, 2/3, 0/3 (16:25 lost track near (2.8, 0.2)) |
| 17:19, 17:20 | 1346-pt trajectory | "drone did not start the trajectory" (refused 5x) |
| 17:47 | 4-lane, 1.0 m spacing | tags 2, 4; soft-fence trip on the sideways leg home (x = -0.18) |
| **17:50** | 4-lane | geofence abort: **EKF reset over the foam block** (PX4 log 67, see below) |
| 18:19 | 2-lane on the PDF tag rows + slow zones | tags 2, 4 (tag 6 boxed, 2 frames) |
| 18:23 | same | **3/3, 130 s** |
| **19:01** | same, PX4 baro on | **3/3, 124.5 s** (best run; code = commit 0e0ecd5) |
| 19:18 | 0.20 m/s, short slow zones, look-again | 0/3: motion blur in evening light (28 good reads vs 147); look moves never flew (stale-telemetry race) |
| **19:48** | look-again misfired on a misread | 1/3, **hit the left wall** (PX4 log 78, see below) |

Laptop code after the day: commit ec46b88 = the 19:01 code with look_again off.

## 17:50: foam block -> EKF reset (PX4 log 67, `analysis/reset_1750_block.png`)

PX4 had EKF2_HGT_REF=2 (rangefinder), EKF2_BARO_CTRL=0 and EKF2_RNG_NOISE=0.02.
1. Over the 0.15 m foam block that boxes a tag, the height estimate fell from 0.52 to 0.33 m, so the drone climbed.
2. Past the block, the range readings were rejected (rng_vpos test ratio > 1, 109.3-111.2 s).
3. With no baro, the height drifted to about 1.06 m.
4. At 111.4 s EKF2 reset xy by (-0.96, +1.13) m, which tripped the laptop geofence.

Fix: EKF2_BARO_CTRL=1, EKF2_RNG_NOISE=0.05 (set over USB). A hover plus a manual pass over the block afterwards were clean (visual check).
Original params: `../px4_params_drone1_2026-11-10.txt`.

## 19:48: wall hit = 1 m position-estimate drift (PX4 log 78 + 1958.mp4)

- **PX4:** a normal flight to 84 s (setpoint tracked, no resets, yaw ~0°), at (3.35, -1.37) on the left lane. RC kill at 84.3 s. The jump to (3.2, -2.7) happened during the fall and impact reset.
- **Video:** the drone flew along the left (whiteboard) wall at about y = -2.3, about 1 m left of where PX4 thought it was, clipped the wall near x ≈ 3.3 and dropped.
- **Cause:** a position-estimate drift of about 1 m.
  - The gyro z bias was small (at most about 5° over 84 s, about 0.3 m).
  - Most of the error is optical-flow drift: flow quality was 77-83 in dim evening light (90-117 earlier).
  - The drone has no compass (EKF2_MAG_TYPE=5).
  - Tag 2 was localised 0.3 m off its earlier position, so the frame had already slipped early in the flight.
- **Contributing:** the look-again detour (triggered by one misread ID 14) made the flight longer and more back-and-forth.
- **Mitigation:** full room lighting, 30 s still after power-on, short direct flights, RC kill ready. The geofence cannot catch estimate drift.

## Other findings
- **Trajectory start:** the firmware ignores START until it is flying, and mid-flight uploads lose chunks. Pre-upload during takeoff, then re-send START.
- **Speed:** 0.20 m/s blurs tags in low light. 0.144 m/s with 0.06 m/s slow zones worked.
- **Boxed tag:** visible only from about 0.15-0.30 m short, in line (camera faces +x and the heading is fixed). The sim harness and its results are in the commit messages of 9719e1d.
