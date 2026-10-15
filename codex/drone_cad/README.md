# CDE1302 — Astra organic deep-truss CAD

Code-native CadQuery/OpenCascade prototype. No Fusion dependency. Dimensions are millimeters. This build writes only inside `codex/drone_cad`.

The external legacy exports changed during this task: five of twelve initial hashes are recoverable in `CAD/exports/rev1`; seven no longer match any available copy. No files there were written by this build. The preservation check remains failed rather than replacing the baseline or overwriting current exports. See `validation.json` for the file-by-file comparison. The rebuild produces review packages but returns a failing verification status while this external baseline discrepancy remains.

## Rebuild

On Windows with Python **3.11** and the Python launcher installed, from any PowerShell directory:

```powershell
& 'C:\cde1302\codex\drone_cad\build.ps1'
```

The script creates `.venv` inside this project if missing, installs exact versions from `requirements-lock.txt`, builds the B-reps and exports, renders the assembly, writes numerical reports, then runs pytest. Initial dependency installation needs internet access. It does not alter the workspace's Miniforge installation. Allow several minutes for boolean solids and checks. Read `output/reports/progress.log` while it runs. `-Force` regenerates solids rather than using the content-addressed B-rep cache. Deleting `output/cache` is optional; it is not source.

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

1. Open individual 3MF or STL files in Bambu Studio. Select the actual A1/A1 mini and filament profile. Use the print sheet's initial walls, infill, orientation and support guidance. Check all slots/holes in the layer preview. Tree supports are required for the raised deck, deep arms and inverted guard spokes. Add a brim as needed. Individually place parts; don't slice the assembly mesh.
2. Dry-fit the motor pads and locating pins before a full frame print. The frame pad is 5 mm; each guard pad is 2 mm. All four motor screws use the preserved 12 × 12 mm pattern with short radial slots. The four guards are positioned/labeled in their corresponding assembly locations; preserve their key orientation.
3. Mount the bottom plate using four 30 mm standoffs at (±21.4, ±21.4). Feet mount below the plate with screws through their caps. The integrated hollow frame legs require a top screw that passes the full leg height: verify a suitable M2.5 screw and actual standoff thread depth before purchasing/torquing. Avoid crushing PETG.
4. The battery rests on the TPU pad above the bottom plate. Thread Velcro through both slots and fully around the battery. Check retention inverted. Place the downward MTF-02P in its underside tape nest and retain it without crossing the optical aperture. Mounting-hole spacing remains unknown.
5. Pass the FC through the deck's opening and mount on 5 mm isolators at the 25.5 mm pattern. Verify the USB connector is on the assumed +y side and route cables clear of the props. The receiver pocket is behind the opening; use ties and the rear antenna notch.
6. Fit the camera in the TPU cradle, retain through its tie slots, and adjust the side pivots within 30–75°, nominal 45°. The mating faces have 15° detents; gently clamp the TPU teeth into the cheek grooves. The cheeks have 3.3 mm holes and the cradle has 2.5 mm pilot holes; verify short M3 screw retention in a coupon before assembly, keeping screw tips out of the camera envelope. Verify actual board, heatsink, lens, flex cable and screw fit. Check the entire mechanical/optical sweep, not only the nominal pose.

**Motor screw depth is unresolved:** M2×10 through the 5+2 mm stack has nominal 3 mm engagement before any washer allowance. Measure safe motor thread depth and winding clearance. Do not choose a longer screw without that measurement.

The design is a prototype flight component. Watertight meshes and zero modeled interference do not prove printed strength, adhesion, fatigue life, vibration behavior or crash survivability. Measure actual masses/CG, inspect the prints and dry-fit the complete electronics/prop stack before any powered test. Exact outstanding checks appear in `VALIDATION.md`.

API references used: [CadQuery class reference](https://cadquery.readthedocs.io/en/latest/classreference.html), [STEP/assembly export](https://cadquery.readthedocs.io/en/latest/importexport.html).
