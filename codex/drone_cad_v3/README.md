# CDE1302 - v3 frame (tapered closed hollow-box arms)

Built from `codex/drone_cad_v2` (left unchanged) after Krish approved candidate A at stiffness criterion X (see `output/reports/TOPOLOGY_DECISION.md`). Millimetres. Axes: x forward, y right, z up with the motor stack on +z; flight reports use z down.

```powershell
.\build.ps1      # builds with ..\drone_cad\.venv, runs tests, packages
# or: cd src; ..\..\drone_cad\.venv\Scripts\python.exe -u build3.py
```

## What it is
- **Frame_PETG**: four tapered closed hollow-box arms (12 -> 6 mm wide, 9 -> 6 mm deep, 1.68 mm walls, 45 deg chamfered roofs) hanging below the pad plane (z 3), joined to an 8 x 8 mm hollow-box diamond through the four FC bosses, with links to the four bottom-mount bosses. Pads D20 x 3, D8 centre, four D2.3 holes on a cardinal 6 mm pattern (round holes; the 1 mm slot travel was dropped). No deck, no integrated cheeks, no ELRS pocket.
- **Guard x4**: v2 rev B five-keel-spoke generator with a 1.68 mm ring, 6 mm band (z 20.1-26.1, PLACEHOLDER), 2.8 x 4.5 mm spokes and a tapered ring underside (no support under the ring).
- **BottomPlate_PETG**: 66 x 44 x 1.68 plate, windows, front ears and rear ELRS tab on the (+-40, +-12) holes, twin camera cheeks (2.52 mm). **BatteryPad_TPU**, **CameraCradle_TPU**, **Foot x4 TPU** (cups under the mount holes).
- **Coupons**: FitCoupon_Frame_PLA, _Guard_PLA, _GuardSpoke_PETG, _ArmBox_PETG.

## Files
`config/parameters.json` (all inputs, placeholders flagged), `src/parts3.py` (v3 frame/plate/envelopes), `src/parts.py` (guard, cradle; v2 code with the v3 guard edits), `src/build3.py` (build + validation), `src/topology.py`, `compare_candidates.py`, `fem_check.py`, `budget.py`, `baseline.py` (planning step). `src/build.py` and `src/validate.py` are the v2 files, imported only for helpers.
Outputs: `output/{step,stl,3mf,renders,reports}`, `Assembly.glb`, `Manufacturing.zip`, `Source_and_reports.zip`.

## Tests
`..\drone_cad\.venv\Scripts\python.exe -m pytest tests -q` (needs `output/reports/validation.json` from a build).

## camera
`laptop/tag_debug.py` `CAM_FWD_OFFSET_M` (currently 0.054, from the v2 pivot 50 mm): the v3 camera pivot is at x 64.38 mm and the lens origin 6 mm along the optical axis at 45 deg gives **about 0.0686 m** (placeholder lens offset; not edited).
