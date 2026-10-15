# Astra prompt: autonomously generate the CDE1302 drone CAD

Run this prompt in a GPT-6 Astra task whose working directory is `C:\cde1302`.

---

You are the mechanical engineer and code-native CAD agent for a small crash-prone CDE1302 quadrotor. Create the actual parametric CAD in this workspace using code and a local solid-modeling kernel. Do not use or control Autodesk Fusion. Do not stop at advice, sketches, a concept render, or CAD instructions for the user to follow.

Your job is complete only when the workspace contains reproducible CAD source, generated solid-model files, printable meshes, preview renders, and a validation report for the complete drone-frame stack.

## Outcome

Build a lightweight organic deep-truss frame and four separate raised prop guards while preserving the known hardware interfaces. Include the integrated body deck and front camera cheeks, plus compatible bottom plate, TPU camera cradle, feet, and battery pad.

Optimize for:

- crash survivability and retention of components;
- low mass and low rotational inertia;
- FDM printability on Bambu A1/A1 mini printers;
- parametric editability;
- propeller, motor, camera, sensor, battery, and electronics clearances;
- reproducibility from a clean checkout with one documented build command.

This is a prototype flight component. Do not claim it is structurally safe merely because geometry checks pass. Clearly separate computed results, engineering estimates, and items requiring physical verification.

## Context to use

Read `codex/PROJECT_SUMMARY.md` first. Consult the following only when relevant to the current design or validation step:

- `cde1302-chassis-draft-prompt.md` for the detailed requirements and measured values;
- `CAD/exports/PRINT_NOTES.md` and `CAD/exports/mass_raw.json` for the superseded first-draft baseline;
- `CAD/screenshots/` for the existing functional layout;
- `Pictures/` for the current physical drone and hardware;
- `Pictures/reference/` for the intended organic visual language.

The old Fusion geometry is reference material, not a required dependency and not the target shape. Do not modify or overwrite `CAD/exports/` or the prior screenshots.

## Working area and deliverables

Create a self-contained project under:

`C:\cde1302\codex\drone_cad\`

Use this layout unless the selected CAD tool has a strong reason to vary it:

```text
drone_cad/
  README.md
  requirements.txt or environment.yml
  build.ps1
  config/
    parameters.json
  src/
    geometry helpers
    part generators
    assembly and validation entry points
  tests/
  output/
    step/
    stl/
    3mf/
    renders/
    reports/
```

The final output must include:

- editable parametric source code;
- a native solid/B-rep export for every part, preferably STEP;
- STL and 3MF for each printable part;
- a complete assembly STEP or equivalent;
- an assembly mesh for visual inspection;
- isometric, top, side, underside, and exploded renders;
- clearance renders with motor, prop-disc, FC, battery, camera, MTF-02P, and receiver envelopes visible;
- `validation.json` containing machine-readable results;
- `VALIDATION.md` containing a concise human-readable engineering report;
- a print sheet with quantity, material, orientation, walls, infill, supports, bounding box, volume, and estimated mass;
- a one-command rebuild script documented in `README.md`.

## CAD toolchain

First inspect the local environment. Prefer a scriptable boundary-representation CAD stack based on OpenCascade, such as CadQuery or build123d, because the result must support smooth solids, booleans, fillets, STEP export, and numerical inspection.

If no suitable stack is installed, create an isolated environment inside `drone_cad` and install the minimum required dependencies. Do not modify the base Miniforge installation or unrelated workspace folders. Record exact dependency versions. Use OpenSCAD only as a fallback if an OpenCascade-based stack cannot be made to work, and document the lost capabilities.

Use mesh libraries only for export, collision checks, slicing-oriented analysis, and rendering—not as a substitute for the canonical parametric solid model.

All dimensions are in millimeters. Store authoritative design inputs in `config/parameters.json`; keep derived values in code and report them. Every generated artifact must be rebuildable from the parameters and source.

## Required architecture

- Quad-X layout with `arm_r = 77.8 mm` and 110 mm side-to-side motor spacing.
- Four organic deep-truss arms:
  - curved upper and lower chords;
  - chords merge around each motor pad and splay toward the body;
  - branching web members with open lens-shaped gaps;
  - smooth, generously blended nodes instead of abrupt T-junctions;
  - thicker roots and slimmer motor ends;
  - adjacent-arm ties only where they improve the load path or torsional behavior.
- A raised central body/deck integrated into the frame, with legs flowing into the arm roots.
- The FC passes through the deck opening and mounts on the existing isolator pattern; preserve access to USB-C.
- Camera-mount cheeks are integrated into the front frame.
- The camera cradle is a separate sacrificial TPU part, adjustable from 30 to 75 degrees with 45 degrees nominal.
- Battery remains below the frame on a separate bottom plate with a Velcro strap.
- MTF-02P faces straight down near the CG with a clear 42-degree view cone.
- ELRS receiver has a pocket and antenna exit using a parameterized placeholder envelope.
- Four separate, replaceable PETG prop guards.

## Prop guards

Each guard is a raised C-shaped ring around the prop plane, open toward the body. Use curved forked/Y-shaped spokes that branch near the ring and form triangular or lens-shaped openings. The spokes rise from a keyed motor-pad interface and stay below the prop disc. Front guards must clear the frame, camera mount, and camera view throughout the full tilt range.

Key each guard with a boss or two locating pins so it self-aligns and cannot rotate. Preserve the stack order frame -> guard -> motor and all four M2 motor fasteners.

## Known hardware and geometry

- Motor: Flywoo NIN 1404 V2, 4850 KV.
- Motor base: four M2 holes on a 12 x 12 mm square.
- Motor bell: 19.4 mm diameter, 14.1 mm high.
- Motor shaft: 1.5 mm diameter x 4 mm.
- Propeller: HQProp T3x3x3, 76.2 mm diameter, three blades.
- Flight controller: MicoAir743v2-AIO-45A; preserve the mounting pattern represented by the existing parameters and 5 mm isolators.
- Camera: Seeed XIAO ESP32S3 Sense.
- Battery: Tattu R-Line V5.0 3S 850 mAh, 58 x 30 x 22 mm, approximately 76 g.
- Optical-flow sensor: MTF-02P.
- Receiver: ELRS with T-antenna; exact model and size are unknown.
- Existing structural/motor-pad thickness: 5 mm.
- Guard minimum inner diameter: 82.2 mm for 3 mm radial prop-tip clearance.
- Guard target outer diameter: approximately 100 mm.
- Guard target overall height: approximately 30 mm.
- Minimum structural wall: 1.6 mm.
- Materials: PETG for rigid structure; TPU for feet, battery pad, and sacrificial camera cradle; PLA only for fit coupons.
- Bambu A1 build volume: 256 x 256 x 256 mm.
- Bambu A1 mini build volume: 180 x 180 x 180 mm.

## Mandatory clearances and interfaces

- Use all four M2 motor holes with short radial/tolerance slots around the 12 x 12 mm pattern.
- Keep frame, body, camera mount, and non-guard hardware at least 4 mm from every prop disc.
- Keep guard spokes at least 3 mm below the prop disc.
- Keep guard spokes at least 1.5 mm from the motor bell.
- Keep at least 10 mm between neighboring guards.
- Keep the front guards outside the camera field of view throughout 30–75 degrees of tilt.
- Keep the optical-flow 42-degree view cone unobstructed.
- Preserve access for FC USB-C, battery strap, antenna routing, wiring, and fasteners.
- Prevent the bottom plate and feet from placing the flow-sensor lens or battery directly onto the ground.
- Preserve the useful first-draft interfaces where the source evidence supports them, but do not reproduce the rejected flat frame shape.

## Visual language

Use the reference images as shape-language evidence rather than geometry to trace.

The result should have:

- flowing tubular or oval members;
- a visible deep section through each arm;
- sculpted load paths and organic open cells;
- large smooth transitions at branch nodes;
- an intentional raised-body silhouette;
- coherent geometry when viewed from the front, side, and top.

Avoid:

- flat plates with decorative lightening holes;
- straight U-channel arms;
- flat perforated prop-guard rings;
- arbitrary surface noise or fragile cosmetic branches;
- features that only look organic but do not form a continuous printable load path.

## Autonomous workflow

Work through the entire task without waiting for topology approval. Safe local actions—including reading files, creating the isolated CAD environment, writing source, running builds/tests, rendering, and revising failed geometry—are authorized.

1. Inspect the relevant project evidence and establish the code-native CAD toolchain.
2. Convert known dimensions, decisions, and placeholders into a clean parameter file with provenance fields.
3. Develop at least three topology candidates at low detail.
4. Render and score them against mass tendency, crash load paths, printability, interface risk, and visual fit.
5. Select the strongest candidate and record the decision in `output/reports/TOPOLOGY_DECISION.md`.
6. Build the complete parametric frame, guards, bottom plate, camera cradle, feet, battery pad, hardware envelopes, and assembly.
7. Run geometry/unit tests and numeric validation.
8. Render the result and inspect the images yourself. Correct visible defects, disconnected members, implausibly thin transitions, boolean artifacts, and interference.
9. Iterate until all machine-checkable requirements pass or a specific unresolved measurement makes a check impossible.
10. Export final manufacturing files and reports.

Do not ask the user to select among routine topology options. Make and document the engineering choice. Ask only if a missing fact creates materially different, safety-relevant designs that cannot both be parameterized. When possible, use a clearly labeled placeholder and continue.

## Geometry and build tests

Automate checks for:

- successful solid generation with no null or invalid bodies;
- expected connected-component/body count;
- watertight printable meshes;
- positive volume and plausible bounding boxes;
- minimum-wall samples at critical members;
- bed fit for each printer;
- hole and interface locations against parameter values;
- clearance between prop envelopes and every solid;
- clearance between motor bells and guard spokes;
- separation between neighboring guards;
- optical-flow view-cone obstruction;
- camera sightline obstruction across the tilt range;
- assembly collisions;
- mass estimates using parameterized densities;
- CG and inertia estimates using component masses and generated geometry.

Where exact continuous minimum-thickness or field-of-view analysis is impractical, implement a conservative sampled check, disclose its resolution, and include diagnostic renders.

## Mass and performance report

Report volume and estimated mass for every printed part. Use explicitly labeled placeholder densities until real spool data is provided.

Report:

- printed-structure mass;
- estimated all-up mass;
- CG in millimeters using axes x forward, y right, z down;
- Jxx, Jyy, and Jzz about the CG in kg m^2;
- comparison with the superseded first-draft mass data;
- comparison with the simulation baseline: mass 0.302 kg and inertia 3.92 / 4.05 / 6.39 x 10^-4 kg m^2;
- estimated thrust-to-weight using `4 * Ct * omega_max^2 / (m * g)`, with `Ct = 2.9625e-7` and `omega_max = 3232 rad/s`.

## Unknowns that must remain explicit

- maximum safe screw depth into the motor base;
- MTF-02P mounting-hole spacing;
- ELRS receiver dimensions;
- Velcro strap width and thickness;
- actual PETG and TPU densities;
- current physical drone all-up mass and CG;
- final physical strength, layer adhesion, fatigue life, and crash behavior.

Represent geometric unknowns as named parameters with comments containing provenance, assumed value, and accommodated range. Do not silently turn an assumption into a fact.

The current 5 mm frame plus 2 mm guard stack leaves approximately 3 mm of motor-thread engagement with an M2x10 screw. Flag this in the validation report and do not recommend a longer screw without verified safe depth.

## Completion criteria

Do not finish after the first successful build. Finish only when:

- all required parts and assembly artifacts exist;
- the build runs from the documented command;
- tests and validation have been run;
- renders have been visually inspected and obvious defects corrected;
- failed or indeterminate checks are listed precisely;
- the old CAD exports remain untouched;
- `README.md` tells Krish exactly how to rebuild, inspect, change parameters, and prepare the files for slicing;
- the final response links the key source, renders, STEP files, printable files, and validation report.

Begin now by reading `codex/PROJECT_SUMMARY.md`, inspecting only the evidence needed for the design, and creating `codex/drone_cad`.

---

## Minimal task launcher

For a new Astra task, submit:

> Read and execute `C:\cde1302\codex\ASTRA_DRONE_FRAME_PROMPT.md`. Generate the CAD autonomously in `C:\cde1302\codex\drone_cad`; do not use Fusion and do not stop at a plan or concept render.
