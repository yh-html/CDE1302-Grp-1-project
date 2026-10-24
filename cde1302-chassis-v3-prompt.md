CDE1302 – design a lighter, more efficient 3D-printed frame ("v3") with the SAME motor pattern, FC pattern and mounting holes as the stock frame

ROLE AND GOAL
I'm Krish. I fly a 3 in quad (PX4) for the CDE1302 single-UAV AprilTag exploration mini-project. I already have two frames:
- stock frame (instructor's STLs, "Actual Mod STL of drone frame/"): 68.5 g printed, flew cleanly.
- stock_plus (codex/stock_plus, my reinforced stock): 79.9 g printed, T/W 5.30.
- drone_cad_v2 / Astra (codex/drone_cad_v2): organic truss, 260.8 g all-up, T/W 4.84. Heavier than stock; guards broke (rev B fix).
Design a new frame that is LIGHTER than stock (target: frame <= 55 g in PETG, stretch 45 g), at least as strong in a crash as stock_plus, and aerodynamically cleaner. Reduce mass first, then reduce obstruction under the props.

HARD CONSTRAINTS (do not change; all measured from the stock STLs, see codex/drone_cad_v2/config/parameters.json)
Coordinates: Astra axes, x forward, y right, z up for geometry (z down in flight reports). Stock +Y is forward.
- Motor centres: (+-56.5, +-56.5) mm. Side 113.0 mm, diagonal radius arm_r = 79.9031 mm.
- Motor mount per motor: pad D20 x 3.0 mm, centre clearance hole D8.0, FOUR M2 holes on a CARDINAL pattern at radius 6.0 mm ((+-6,0),(0,+-6), 12 mm span, NOT a 12x12 square). Keep the 1 mm slot travel only if it does not cost strength.
- Prop guards stay stock-compatible: guard pad D20 x 3.0, same hole pattern, stack order frame -> guard -> motor, M2x10 from below. Prop 76.2 mm, guard ring OD 92, tip gap >= 3 mm, neighbouring guards >= 10 mm apart.
- FC holes: 25.5 mm square rotated 45 deg (centres at (+-18.031,0),(0,+-18.031)), D2.3, 5 mm rubber isolators. FC must keep its exact position relative to the motors (so it keeps the stock CG and PX4 tuning).
- Bottom-panel / leg mounting holes: (+-12, +-40) in stock coordinates, D2.8 in frame (D2.75 in bottom_panel.stl). Rotated into Astra axes: forward = stock +Y.
- Camera: XIAO ESP32S3 Sense, tilt adjustable 30-75 deg (default 45), 91 x 82 deg FOV, lens view not blocked by props, guards or frame. Hinge on the stock tongue near y = 64 is the part that snapped; a twin-cheek or under-nose mount is acceptable.
- Battery Tattu 3S 850 mAh 58 x 30 x 22 mm ~76 g on a bottom plate with Velcro strap. MTF-02P flow sensor facing down (42 deg cone clear). ELRS receiver pocket with antenna exit. FC USB-C reachable.
- Printers: Bambu A1 (256^3), A1 mini (180^3). PETG structure, TPU for crash parts (feet, camera cradle). min_wall 1.6 mm.

WHAT "LIGHTER AND MORE EFFICIENT" MEANS HERE
1. Mass: fewer grams in the frame, guards, plate and fasteners. Every extra 10 g costs about 0.13 of T/W. Remove material that carries no load; do not add stiffness for its own sake (stock showed no excess vibration in Lab 3).
2. Aerodynamic efficiency: minimise structure directly under each prop disc (downwash blockage lowers effective thrust). Keep arms, pads and guard spokes thin and edge-on to the flow, open the area under the rotors, no flat plates in the wash, no hardware between prop and FC wash path beyond what is needed. Quantify as projected area under the prop discs (mm^2), before vs after.
3. Print efficiency: parts print without supports where possible (>= 45 deg overhangs), flat on the bed, short print time, low filament use.
4. Crash survival: load paths through the arm roots and motor pads; crash energy goes into TPU parts first. Past failures to avoid: Y-fork guard joints, the camera hinge knuckles, spoke-to-ring joints.

SUGGESTED APPROACH (decide with data, then explain before building)
a. Baseline: read codex/stock_plus and codex/drone_cad_v2 reports (VALIDATION.md, PRINT_SHEET.md) for mass, per-part masses and T/W. Weigh a real printed stock frame if I give you the figure.
b. Generate 3 topology candidates that all keep the fixed interfaces above, e.g. (A) tapered I-beam / hollow-box arms, (B) optimised-lattice or topology-optimised single-wall arms, (C) X-brace with a minimal body ring. Compare mass, bending/torsion stiffness (simple beam model or FEA), projected area under the props, support need. Recommend one.
c. Use variable cross-section: deep at the arm root (where bending moment peaks), slim at the motor end (arm tip mass raises inertia by distance squared).
d. Thin-wall design tuned to the slicer (wall thickness a multiple of the 0.42 mm line width, 2-3 perimeters, low or zero infill). Say which print settings the mass estimate assumes.
e. Integrate parts to remove fasteners and the separate top plate, but keep crash-sacrificial parts (camera cradle, feet) separate and replaceable.
f. Guard: lighter ring, five separate keel-section spokes (rev B proved safer than Y-forks), dowel holes for 1.75 mm filament if that helps alignment. Keep stock-compatible pad.

DELIVERABLES
1. Short design brief: chosen topology, why, expected mass vs stock (68.5 g) and stock_plus (79.9 g), T/W estimate (T/W = 4*Ct*omega_max^2/(m*g), Ct = 2.9625e-7, omega_max ~ 3232 rad/s, all-up mass incl. placeholders: motors 8.5 g each, props 2 g each, battery 76 g, FC 10 g, camera 7 g, flow 1.5 g, misc 20 g).
2. Parametric CAD (CadQuery in a new codex/drone_cad_v3 copied from drone_cad_v2, reuse its venv and test harness) driven by the parameters above. Do not edit drone_cad, drone_cad_v2 or stock_plus.
3. Validation report: interface check (every fixed hole centre and diameter matches the list above, tolerance 0.05 mm), prop-disc clearance >= 4 mm, camera view clear at 30-75 deg, bed fit, min wall, overhang/support report, interference check, mass and CG, inertia vs sim baseline (m 0.302 kg; J 3.92 / 4.05 / 6.39 e-4 kg m^2).
4. One 3MF per part in output/3mf, named <Part>_<material>.3mf, with suggested orientation and slicer settings, plus a FitCoupon for the motor pad / FC hole pattern / one guard spoke break test before the full print.
5. List of placeholder values still to measure, ordered by how much they change geometry (motor safe screw depth, standoff length, leg mounting, strap size, ELRS size, spool density).

HOW TO WORK
- Plan first: show the 3 candidates and a recommendation, wait for my OK before building the full frame.
- Never present a guessed dimension as fact; add a PLACEHOLDER parameter and flag it.
- Keep scripts small and run the tests after each part. Report failures with output; do not hide them.
- Do not change laptop/ or main/ firmware. CAM_FWD_OFFSET_M in laptop/tag_debug.py will need updating once the camera position is final: tell me the new value but do not edit it.
- If a requirement conflicts (e.g. lighter vs. crash-proof), say so and give a recommendation.
