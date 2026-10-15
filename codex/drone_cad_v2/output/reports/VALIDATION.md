# Engineering validation — Astra 01 v2

**Prototype; not flight-qualified.** Geometry checks do not establish crash survivability or structural safety.

Automated checks: **216 pass, 0 fail, 11 indeterminate**. See `validation.json` for every numeric result and collision pair.

## Computed geometry and mass

- Printed solid-volume structure: **102.84 g**.
- Estimated all-up mass: **260.84 g**, vs captured legacy 244.68 g and simulation 302 g.
- CG, x forward/y right/z down: **(2.16, -0.00, 0.99) mm**.
- Jxx/Jyy/Jzz about CG: **0.000467095 / 0.000519135 / 0.00080794 kg m²**.
- Simulation inertia: 3.92 / 4.05 / 6.39 × 10⁻⁴ kg m².
- Estimated thrust-to-weight: **4.84**, using 4 Ct ωmax²/(m g), Ct=2.9625e-7 and ωmax=3232 rad/s. This simulation estimate is not a measured 3S motor thrust result.

Comparison uses the captured legacy_mass_reference.json snapshot (244.68 g), obtained after external export changes. The initial project summary cited 234.38 g; its original mass_raw.json hash is no longer available, so that earlier model cannot be fully recomputed.

Printed masses from exact B-rep volumes and placeholder solid densities; intrinsic inertia from the export tessellation. Hardware mass overrides. Prop uniform-disc inertia approximation. Misc mass distributed over a 30 x 30 x 10 mm central box. No slicer infill deduction.

The 110 mm motor side spacing implies a 77.7817 mm radius; the supplied arm_r=77.8 is rounded. Guard OD=93 mm, ring ID=88 mm (required >= 82.2 mm, prop-tip gap 5.9 mm); ring band z 20.1-28.1 mm, centred on the prop plane.

Legacy printed structure was 86.68 g; change +16.16 g. All-up estimate changes +6.6% vs the legacy draft and -13.6% vs simulation. Legacy CG-corrected Jxx/Jyy/Jzz: 0.00040603 / 0.000441419 / 0.000675455 kg m² (converted from supplied origin inertias). Simulation inertia changes: +19.2% / +28.2% / +26.4%. Comparisons inherit the legacy placeholder masses and density assumptions.

Nose-down tip-over about the front feet: first ground contact at CameraCradle_TPU (worst tilt) 33.1 deg, BottomPlate_PETG 40.7 deg, Frame_PETG 49.2 deg, Guard_1_PETG 40.0 deg, Guard_2_PETG 40.0 deg. Lower angle = touches first.

## Failures

No failures in implemented checks for the configured placeholder envelopes.


## Indeterminate and physical verification

- **camera_actual_optics**: Field 91 x 82 deg derived from the firmware calibration (main/at_detect.c, QVGA); lens protrusion and board orientation are still placeholders. Mechanical sweep sampled every 3 deg, not continuous proof.
- **continuous_minimum_wall**: Critical line sections and analytic nominal member bounds checked; no certified global medial-axis wall analysis. Rounded free edges taper by construction; inspect slicer thin-wall preview and print coupons.
- **safe_motor_thread_engagement**: M2x10 nominal engagement 3.0 mm, without washer allowance. Maximum safe motor thread depth unknown. Verify engagement and winding clearance; do not fit longer unverified screws.
- **MTF_mount_holes**: Unknown pitch: retention nest uses foam tape plus a nonoptical retaining strap; verify actual board and lens/connector offsets.
- **ELRS_fit**: 20 x 12 x 4 mm placeholder with antenna notch and ties. Measure real board/antenna bend radius.
- **strap_fit**: 12 x 2 mm placeholder strap, 1 mm width / 0.6 mm thickness allowance. Check threading and secure battery under inversion.
- **material_density**: PETG 1.27 and TPU 1.21 g/cm3 placeholders; weigh actual prints. Solid-volume mass is not slicer mass.
- **physical_mass_CG**: Current drone actual all-up mass and CG not supplied; compare measured assembled prototype with estimates.
- **physical_strength**: No FEA, fatigue, crash, layer adhesion, motor vibration or flight qualification. Geometry validity is not proof of structural safety.
- **motor_prop_envelope**: Hub seating and full blade axial swept envelope assumed. Measure physical prop height/sweep and rerun clearances.
- **electrical_routing**: USB corridor and antenna notch modeled; solder joints, flex cables, wire bend radii and screw-tool sweeps require dry assembly.

## Methods and limitations

- B-rep validity, one-solid counts and positive volume from OpenCascade. STL winding, watertightness, connectivity and volume agreement from independently processed triangle meshes.
- Conservative bounding-box separation bounds, with exact B-rep minimum distances wherever a bound does not establish the required clearance, to 2 mm thick prop discs; intentional motor-shaft/prop-hub engagement excluded from collisions. Guard pad is an intentional motor mounting surface; bell-to-spoke check excludes that pad.
- Assembly common-volume intersections use bounding-box broad phase and a 1e-4 mm³ reporting threshold. Touching mating faces are allowed; penetration is not.
- Camera FOV uses full rectangular solid frusta at 3° tilt increments from 30–75°, inflated to cover the intervening ±1.5° pitch and lens-origin motion. Moving camera/prop clearance includes the maximum between-sample rotational travel. Camera/frame mechanical overlap remains a sampled check; actual optics are unverified. MTF analysis uses a full expanding solid cone from a finite assumed lens aperture.
- Wall checks combine local line intersections and analytic nominal diameters/ligaments, not exhaustive minimum wall measurement. Support area is the downward-facing surface beyond 45°, excluding the first 0.25 mm above the bed. It is not a slicer prediction.
- All assembly STEP coordinates use z up; CG/inertia are transformed to requested flight z down. Individual print meshes are transformed to the bed; STEP parts retain assembly coordinates.
- Original exports are checked against the preserved SHA256 manifest.
