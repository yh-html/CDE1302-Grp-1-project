# CDE1302 — Astra organic deep-truss CAD, v2

Code-native CadQuery/OpenCascade prototype. No Fusion dependency. Dimensions are millimeters. This is a copy of Codex's Astra 01 project (`codex/drone_cad`, left unchanged) with the review changes below. The build writes only inside `codex/drone_cad_v2`.

The read-only preservation check now uses a baseline of the 15 files in `CAD/exports` as they were when v2 was made (`config/legacy_hashes.json`). The original Astra 01 baseline had already been broken by the Fusion rev-2 export, which moved files into `CAD/exports/rev1`.

## v2 changes

| | Astra 01 | v2 | Why |
|---|---|---|---|
| Camera tilt | turned the lens 30–75° **up** (`geometry.tilt` sign) | 30–75° **nose-down** | The camera scans the AprilTag in the final course run; `laptop/tag_debug.py` assumes 45° nose-down |
| Camera mount | cheeks on the frame, from the deck corners; pivot (34, 0, 42), the highest point of the drone | **under the nose**: two upright cheeks rise from the bottom plate's front corners; pivot (50, 0, −16) (`camera_mount` `under_nose`) | Nothing of the drone is in the camera's view at any tilt (see *Camera and AprilTag*). The frame no longer carries cheeks |
| Camera field of view | 80 × 60° placeholder | 91 × 82° from the firmware calibration (`main/at_detect.c`) | Use the lens the firmware actually uses |
| Lens hood | none | TPU lip 1.5 mm past the lens tip, outside the field of view | A flat hit lands on TPU, not on the lens |
| Guard ring | OD 99, z 22–35 (13 mm band, 2 mm wall, 9.4 mm tip gap) | ID 88, z 20.1–28.1 (8 mm band centred on the prop, 5.9 mm tip gap), 2.4 mm wall (6 lines), 1.4 mm outward lip on the top edge, rounded C ends | The guards were 43–50% of aircraft inertia; the lip stiffens the C-ring without narrowing the prop gap |
| Guard spokes | three Y-forks of Ø3.4 round tube butting into the band's lower edge | five separate keel-section spokes (3.2 × 5.0 mm: round top, 50° V underneath), each passing under the band and running up its outside as a full-height rib; 1.5 mm collar around the spoke roots | Printed PETG guards broke at the forks and the spoke-to-ring joints (see *Guard rev B*) |
| Guard print orientation | ring down: spokes and the motor seat print on tree supports | pad down: nothing on the spokes or pad, supports only under the ring's lower edge | The breaks were at joints printed as overhangs on supports |
| Frame underside | lower chords sag below the pads (`arm_lower_sag` 4), ties 2 mm below the bed | flat at z 0 (`arm_lower_sag` 0, ties on the bed, truss depth from the upper chord) | Pads and motor-screw seats print on the bed, not on supports |
| Bottom plate | battery stops up, sensor nest walls down (9 mm tall) | single-sided: 0.8 mm underside recess for the MTF-02P, plus cable and battery lightening windows | Prints flat with no supports |

Results, from each build's own `validation.json` (placeholder hardware masses, solid-plastic densities):

| | Astra 01 | v2 |
|---|---|---|
| Printed parts | 109.9 g | 102.8 g |
| All-up | 267.9 g | 260.8 g |
| Thrust-to-weight (sim thrust) | 4.71 | 4.84 |
| Jxx / Jyy / Jzz (×10⁻⁴ kg·m²) | 5.37 / 5.72 / 8.74 | 4.67 / 5.19 / 8.08 |
| Frame / bottom plate / guard (each) | 45.0 / 10.6 / 10.6 g | 40.5 / 11.3 / 9.7 g |
| Support-candidate area: frame / bottom plate / guard | 56.7 / 29.0 / 10.3 cm² | 16.5 / 2.2 / 4.3 cm² |
| Camera view blocked by the drone (45° down) | n/a: the camera pointed up | 0% at every tilt, 30–75° |
| Checks (pass / fail / pending) | 212 / 1 / 11 | 216 / 0 / 11 |

Astra 01's one failure was the preservation baseline. v2 adds three checks (camera ground clearance, the cheek wall and the guard rib wall); every check passes, including the camera field of view. The 11 pending items need the real parts: lens optics, continuous wall thickness, motor screw depth, MTF-02P holes, receiver fit, strap fit, material density, weighed mass/CG, strength, the motor/prop envelope and wiring.

Geometry robustness: `geometry.union` now refuses a fuse that silently drops inputs (it checks volume) and falls back to pairwise fuzzy fuses; the guard spokes are swept ovals fused one Y at a time.

## Guard rev B

The first printed guards (Codex's Y-fork design, PETG, 100% infill) broke at the forks and where the spokes joined the ring. The pads and bands survived. The causes:
- The 3.4 mm spokes are all wall, so the 100% infill did nothing.
- The joints were sharp and small. Roughly 270 g of force on the ring could crack one (estimate: 30 MPa across layers, ×2 for the notch, 22 mm lever).
- Printed ring-down, the spokes, the forks and the motor seat were built on tree supports. Their scars and overhang-printed joints became notches.

Rev B changes:
- **Five separate spokes, no forks.** The end spokes sit 5° inside the C ends.
- **Rib joints.** Each spoke passes under the band and runs up its outside face as a full-height rib, so the joint spans the band height instead of butting into its free edge.
- **Keel-section spokes.** They print without support at any slope.
- **A 1.5 mm root collar** outside the motor bell's keep-out. The pad stays 2 mm, so motor screw engagement is unchanged.
- **A stiffer ring:** 2.4 mm wall plus an outward top lip, so the C doesn't fold under a thumb.

Print the guards pad-down as exported, with 6 walls (infill does nothing), 240–250 °C and the part fan at 30–40% so the layers fuse. Tree supports go only under the ring's lower edge. Before printing four, print `FitCoupon_GuardSpoke_PETG` (one spoke, its pad and a 44° piece of ring) the same way and break-test it by hand against an old guard.

## Camera and AprilTag

The camera (the XIAO ESP32-S3 board itself) hangs under the nose, ahead of the battery and below the frame. The TPU cradle pivots on an M3 screw between two upright PETG cheeks that are part of the bottom plate. The cheeks print standing on the bed with the plate.

- **View.** The validation sweeps the full 91 × 82° field (firmware calibration, 240 mm range) every 3° from 30° to 75° nose-down, and nothing on the drone is in it: guards, props, frame, plate, feet and battery are all clear. Rendered view: `output/renders/camera_view_v2_under_nose.png`.
- **Lens position for the tag code.** At 45° the lens is about 54 mm ahead of the frame centre and 20 mm below the frame underside (about 34 mm below the FC). From 30° to 75° it moves between 55 and 52 mm forward. `laptop/tag_debug.py` uses `CAM_FWD_OFFSET_M = 0.02`; with this mount it should be about **0.054**. `CAM_PITCH_DEG` must match the detent you set (30/45/60/75).
- **Clearances over the whole tilt range.** The cradle sweeps x 34–66, z −32 to 0:
  - 14.3 mm above the ground;
  - 2.8 mm ahead of the battery stops;
  - just under the frame underside;
  - 23.5 mm from the nearest prop.
- **Nose-down tip-over** (pivoting on the front feet): first ground contact is the TPU cradle at 33° (worst tilt, 30°). Then the plate's cheeks and the front guards at about 41°. The cheeks' front-lower edge is cut back at 50° so PETG is not the first thing to hit, and the cut needs no support.
- **Cable.** Route the camera cable up past the front of the battery to the FC; keep it outside the cradle's sweep.
- **Top mount, kept as an option.** Set `camera_mount` to `top` and `camera_pivot` to (38, 0, 41) to build the deck-top cheeks instead. There, the front guards and props block about 18% of the image at 45° and the field-of-view check fails. That study, and every other position tried, is in `output/renders/camera_study/`.
- The camera size, lens protrusion and board orientation are still placeholders. The firmware sets `vflip`, so check which way up the board sits in the cradle.

## Rebuild

On Windows with Python **3.11** and the Python launcher installed, from any PowerShell directory:

```powershell
& 'C:\cde1302\codex\drone_cad_v2\build.ps1'
```

If this copy has no `.venv`, the script reuses the identical environment in `..\drone_cad\.venv`. Otherwise it creates `.venv` inside this project if missing, installs exact versions from `requirements-lock.txt`, builds the B-reps and exports, renders the assembly, writes numerical reports, then runs pytest. Initial dependency installation needs internet access. It does not alter the workspace's Miniforge installation. Allow several minutes for boolean solids and checks. Read `output/reports/progress.log` while it runs. `-Force` regenerates solids rather than using the content-addressed B-rep cache. Deleting `output/cache` is optional; it is not source.

`requirements.txt` records direct dependencies; `requirements-lock.txt` records the tested full environment. Linux/macOS can use Python 3.11, install the lock file's applicable packages, and run `python src/build.py`, then `python -m pytest tests`; the validated build here is Windows.

## Inspect

- `output/renders/assembly_iso.png`: overview; `frame_detail.png` and `guard_detail.png`: shape detail.
- `output/renders/assembly_top.png`, `assembly_side.png`, `assembly_underside.png`, `assembly_exploded.png`: assembly views.
- `output/renders/clearance_*.png`: hardware and sight envelopes, including prop discs.
- `output/step/Assembly_with_hardware.step`: named full assembly and hardware envelopes.
- `output/step/Assembly_printed.step`: printed parts only. Each individual STEP is also available.
- `output/Assembly.glb`: colored inspection mesh. `output/stl/Assembly_VISUAL_ONLY.stl` is not a printable single component.
- `output/reports/VALIDATION.md`, `validation.json`, `PRINT_SHEET.md`, `TOPOLOGY_DECISION.md`: engineering results and print preparation.

Canonical geometry is B-rep source, not a mesh approximation. STEP retains assembly positions. STL and generic Core 3MF contain one part per file, oriented for printing and placed at z=0. Four separately named guards and four feet are included: print each file once. They are not Bambu Studio projects with machine presets.

## Change dimensions

Edit `config/parameters.json`, retaining each value's provenance/status and uncertainty notes, then rerun the command. Hardware dimensions, clearances, material densities and component masses live there. Construction point ratios in `src/parts.py` control the organic paths; `src/geometry.py` supplies smooth circular sweeps and tapered lofts. `src/build.py` assembles and exports; `src/validate.py` checks the finished solids. A parameter/source digest invalidates the geometry cache. Changing arbitrary dimensions can create collisions: review every failed check before printing.

110 mm side spacing is authoritative; the stated 77.8 mm arm radius is rounded (derived radius 77.7817 mm). CAD axes: x forward, y right, z **up**. The CG and inertia reports transform to x forward, y right, z **down**.

## Assembly and slicing

1. Open individual 3MF or STL files in Bambu Studio. Select the actual A1/A1 mini and filament profile. Use the print sheet's initial walls, infill, orientation and support guidance. Check all slots/holes in the layer preview. Tree supports are required for the raised deck and deep arms, and under the guard ring's lower edge (guards print pad-down; no supports on their spokes). Add a brim as needed. Individually place parts; don't slice the assembly mesh.
2. Dry-fit the motor pads and locating pins before a full frame print. The frame pad is 5 mm; each guard pad is 2 mm. All four motor screws use the preserved 12 × 12 mm pattern with short radial slots. The four guards are positioned/labeled in their corresponding assembly locations; preserve their key orientation.
3. Mount the bottom plate using four 30 mm standoffs at (±21.4, ±21.4). Feet mount below the plate with screws through their caps. The integrated hollow frame legs require a top screw that passes the full leg height: verify a suitable M2.5 screw and actual standoff thread depth before purchasing/torquing. Avoid crushing PETG.
4. The battery rests on the TPU pad above the bottom plate. Thread Velcro through both slots and fully around the battery. Check retention inverted. Place the downward MTF-02P in its underside tape nest and retain it without crossing the optical aperture. Mounting-hole spacing remains unknown.
5. Pass the FC through the deck's opening and mount on 5 mm isolators at the 25.5 mm pattern. Verify the USB connector is on the assumed +y side and route cables clear of the props. The receiver pocket is behind the opening; use ties and the rear antenna notch.
6. Fit the camera in the TPU cradle, retain through its tie slots, and set it between the two cheeks at the front of the bottom plate, within 30–75° nose-down, nominal 45°. The mating faces have 15° detents; gently clamp the TPU teeth into the cheek grooves. The cheeks have 3.3 mm holes and the cradle has 2.5 mm pilot holes; verify short M3 screw retention in a coupon before assembly, keeping screw tips out of the camera envelope. Verify actual board, heatsink, lens, flex cable and screw fit. Check the entire mechanical/optical sweep, not only the nominal pose.

**Motor screw depth is unresolved:** M2×10 through the 5+2 mm stack has nominal 3 mm engagement before any washer allowance. Measure safe motor thread depth and winding clearance. Do not choose a longer screw without that measurement.

The design is a prototype flight component. Watertight meshes and zero modeled interference do not prove printed strength, adhesion, fatigue life, vibration behavior or crash survivability. Measure actual masses/CG, inspect the prints and dry-fit the complete electronics/prop stack before any powered test. Exact outstanding checks appear in `VALIDATION.md`.

API references used: [CadQuery class reference](https://cadquery.readthedocs.io/en/latest/classreference.html), [STEP/assembly export](https://cadquery.readthedocs.io/en/latest/importexport.html).
