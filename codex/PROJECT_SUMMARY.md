# CDE1302 quadrotor project summary

Reviewed: 10 October 2026

## Executive summary

Claude produced a complete parametric first draft of the drone stack in Autodesk Fusion 360 and exported printable 3MF files, screenshots, print notes, and mass-property data. The mechanical interfaces appear substantially worked through, and the assembly was reported as having zero interference.

The first draft is **not the final design**. Its flat U-channel arms and flat, perforated prop guards were rejected on aesthetic and structural-form grounds. The next task is to retain the useful dimensions and interfaces while replacing the frame and guards with an organic, deep-truss design: curved upper and lower arm chords, blended branch nodes, open lens-shaped gaps, a raised central body/deck, and separate raised C-shaped guards.

## Relevant workspace map

The root `C:\cde1302` is also a Miniforge/Python installation, so most root-level DLLs and runtime folders are unrelated to the drone project.

- `cde1302-chassis-draft-prompt.md` — Claude's detailed Fusion brief and latest status/handoff.
- `CAD/exports/` — first-draft printable parts, print notes, and mass data.
- `CAD/screenshots/` — rendered views of the test coupon, frame, guards, plate, cover, camera mount, feet, and full assembly.
- `CAD/scripts/fusion_helpers.py` — reusable Fusion API helpers for parameters, components, sketches, extrudes, patterns, fillets, shells, transforms, bounding boxes, and screenshots.
- `Pictures/` — photos of the current physical drone and components.
- `Pictures/reference/` — four visual references for the desired organic frame/guard language.
- `CDE1302_UAV/CDE1302_UAV/` — a separate Git repository containing MATLAB/Simulink quadrotor dynamics, control, trajectory generation, and course exercises.
- `Labs/` and `Lecture Notes/` — course material; relevant for engineering rationale but not required to begin the CAD redesign.

## What Claude completed

- Created the Fusion design `CDE1302 Frame v1` in project `CAD Workshop Final`.
- Built a complete first-pass stack with about 143 user parameters.
- Exported:
  - frame;
  - four prop guards;
  - bottom/battery plate;
  - top cover;
  - PETG camera base and TPU camera cradle;
  - four TPU feet;
  - optional TPU battery pad;
  - PLA fit-test coupons.
- Documented print orientations and baseline slicer settings for a 0.4 mm nozzle and 0.2 mm layers.
- Added placeholder bodies/masses for motors, props, battery, FC, XIAO camera, MTF-02P, ELRS receiver, and miscellaneous electronics.
- Reported zero assembly interference for the first draft.
- Generated screenshots showing the design and a `mass_raw.json` mass-properties export.

## First-draft quantitative result

The mass export reports an estimated all-up mass of **0.2344 kg**, including placeholder component masses and solid-body material estimates. The calculated CG is approximately **(2.10, 0.00, -0.49) mm** relative to the model origin. Material densities are generic placeholders, so these figures are comparative rather than final.

The exported structural parts include a 22.15 g frame, four 6.13 g guards, a 7.93 g bottom plate, 2.11 g TPU battery pad, 5.42 g top cover, 3.39 g camera base, 5.76 g camera cradle, and four 1.27 g feet.

## What was rejected or superseded

- The flat U-channel frame was rejected visually.
- The flat guards at frame level were rejected; the intended guards rise to prop height and use curved, forked/Y-shaped spokes.
- The separate slide-lock top cover is superseded by revision 2: make the raised body deck part of the frame so it cannot detach in a crash.
- The separate PETG camera base is superseded by revision 2: integrate the mounting cheeks into the front of the frame, retaining only the TPU cradle as a sacrificial removable part.

The BottomPlate, TPU feet, TPU camera cradle, battery pad, and fit-test coupon can probably carry over, subject to interface checks against the new frame.

## Target design language

- Organic, flowing, strong-looking geometry rather than a flat plate with lightening holes.
- Each arm is a deep truss with an upper chord and lower chord.
- Chords merge near the motor pad and spread apart toward the central body.
- Branching web members create open lens-shaped gaps.
- Large blends at nodes; no abrupt T-junctions.
- Thicker at the body roots and slimmer toward the motors to avoid unnecessary tip mass and rotational inertia.
- Thin ties between adjacent arms for torsional stiffness.
- Raised central body/deck whose legs flow into the arm roots.
- Prefer printable self-supporting slopes of at least 45 degrees where practical; tree supports are acceptable.

The reference images show this style clearly; the existing first-draft screenshots show the functional interfaces that should be preserved.

## Known platform and dimensions

- Quad-X, PX4.
- Motor center spacing: 110 mm side-to-side; diagonal about 155.6 mm; `arm_r = 77.8 mm`.
- Motors: Flywoo NIN 1404 V2, 4850 KV; four M2 base holes on a 12 x 12 mm square; bell diameter 19.4 mm; height 14.1 mm; shaft 1.5 x 4 mm.
- Props: HQProp T3x3x3, 76.2 mm diameter, three blades.
- Flight controller: MicoAir743v2-AIO-45A, labeled `H743V2AIO, AM32_45A, PX4`.
- Camera: Seeed XIAO ESP32S3 Sense.
- Optical flow: MTF-02P, looking straight down with a clear 42-degree view cone.
- Receiver: ELRS board with T-antenna; exact model/dimensions unknown.
- Battery: Tattu R-Line V5.0 3S 850 mAh 150C, 58 x 30 x 22 mm, approximately 76 g.
- Frame/motor-pad structural thickness currently 5 mm.
- Guard target: approximately 82.2 mm minimum inner diameter for 3 mm radial tip clearance; approximately 100 mm outer diameter; approximately 30 mm total height.
- FC isolator height: 5 mm. Existing FC stack height: about 30 mm.
- Camera tilt: 45 degrees nominal, adjustable from 30 to 75 degrees.
- Minimum structural wall: 1.6 mm (four 0.4 mm perimeters).
- Materials: PETG for structure; TPU for impact/sacrificial parts; PLA only for coupons.
- Printers: Bambu A1, 256 mm cube; A1 mini, 180 mm cube.

## Mechanical requirements to preserve

- Use all four M2 motor holes with short tolerance slots around the 12 x 12 mm pattern.
- Keep the prop disc at least 4 mm from frame, spokes, camera mount, and body.
- Keep guard spokes at least 3 mm below the prop disc and 1.5 mm from the motor bell.
- Key each guard to its motor pad using a boss or two pins so its screw holes self-align and it cannot rotate.
- Stack remains frame -> guard -> motor, currently using M2x10 screws from below.
- Keep the camera and frame inside the front guard envelope and out of the camera view across 30–75 degrees.
- Battery remains underneath on a bottom plate with a Velcro strap.
- Put the ELRS receiver in a pocket with antenna exit.
- Retain USB-C access to the FC.
- The FC drops through the integrated deck opening and is fixed on its isolators.
- Feet must absorb landing impacts and protect the guards, props, battery plate, and optical-flow lens from the ground.

## Unresolved measurements and risks

1. Maximum safe screw depth into the motor base. The current 5 mm frame + 2 mm guard stack leaves about 3 mm engagement from an M2x10 screw; a longer screw could hit windings.
2. MTF-02P mounting-hole spacing or a trusted CAD model.
3. ELRS receiver model and dimensions.
4. Velcro strap width and thickness.
5. Actual all-up mass and CG of the current drone.
6. Actual PETG and TPU spool densities.
7. Confirm the guard measurements originally stated as 40/50 mm are radii, not diameters.
8. Confirm the new organic frame's printed mass, stiffness, clearances, bed fit, and impact behavior before flight.

## Simulation repository notes

The nested UAV repository contains the 0.302 kg NUSWARM simulation baseline and moments of inertia near 3.92, 4.05, and 6.39 x 10^-4 kg m^2. It also contains working control/Simulink material plus intentionally incomplete minimum-snap coursework:

- `getAbeq.m` has unfinished end/waypoint constraint rows.
- `getQ.m` has an unfinished snap-cost expression.
- both minimum-snap task scripts have an empty 11-column Simulink reference signal;
- task 2 also has unfinished acceleration constraints for a 60-degree gate roll.

Those MATLAB TODOs are separate from the frame-redesign task unless the physical model is later fed back into simulation.

## Recommended next sequence

1. Open the existing Fusion design and inspect its current user parameters and component tree without recreating them.
2. Preserve the validated motor, FC, bottom-plate, camera-cradle, foot, and guard-key interfaces.
3. Suppress or replace only the superseded frame, guard, cover, and camera-base geometry.
4. Build one representative organic arm/body sector first and review its silhouette, prop clearance, and printability.
5. Pattern the approved sector into the complete frame.
6. Rebuild one raised C-shaped guard and verify its keyed interface, prop clearance, front camera clearance, and print orientation before patterning/exporting.
7. Recheck interferences, mass/CG/inertia, bed fit, wall thickness, overhangs, and screw engagement.
8. Export revised files under new revisioned names so the first draft remains recoverable.
