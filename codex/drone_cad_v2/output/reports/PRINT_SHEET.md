# Print sheet — Astra 01 v2

One part per manufacturing file; each listed file quantity 1. Four separate guard and foot files are included. 0.4 mm nozzle, 0.2 mm layers. These are starting settings requiring slicer review and fit coupons. Masses assume solid CAD volume and placeholder densities.

| Part | Qty | Material | Print box mm | Volume cm³ | Solid mass g | Orientation | Walls / infill | Supports |
|---|---:|---|---|---:|---:|---|---|---|
| Frame_PETG | 1 | PETG | 135.0 × 135.0 × 38.2 | 31.924 | 40.54 | Flat, as exported | 4 walls / 35% gyroid | Tree, build plate only, for the deck and upper chords; pads, lower chords and ties are on the bed |
| Guard_1_PETG | 1 | PETG | 93.8 × 93.8 × 23.1 | 7.606 | 9.66 | Pad down, as exported | 6 walls (spokes and ring are all wall; infill does nothing); 240-250 C, part fan 30-40% so layers fuse | Tree, build plate only, under the ring lower edge between spokes; none on the spokes or pad |
| Guard_2_PETG | 1 | PETG | 93.8 × 93.8 × 23.1 | 7.606 | 9.66 | Pad down, as exported | 6 walls (spokes and ring are all wall; infill does nothing); 240-250 C, part fan 30-40% so layers fuse | Tree, build plate only, under the ring lower edge between spokes; none on the spokes or pad |
| Guard_3_PETG | 1 | PETG | 93.8 × 93.8 × 23.1 | 7.606 | 9.66 | Pad down, as exported | 6 walls (spokes and ring are all wall; infill does nothing); 240-250 C, part fan 30-40% so layers fuse | Tree, build plate only, under the ring lower edge between spokes; none on the spokes or pad |
| Guard_4_PETG | 1 | PETG | 93.8 × 93.8 × 23.1 | 7.606 | 9.66 | Pad down, as exported | 6 walls (spokes and ring are all wall; infill does nothing); 240-250 C, part fan 30-40% so layers fuse | Tree, build plate only, under the ring lower edge between spokes; none on the spokes or pad |
| BottomPlate_PETG | 1 | PETG | 89.5 × 50.0 × 24.0 | 8.915 | 11.32 | Flat, as exported | 4 walls / 35% gyroid | None: flat, battery stops up; the 0.8 mm sensor recess bridges; the camera cheeks stand on the bed, front edge 50 deg, M3 pivot holes bridge |
| BatteryPad_TPU | 1 | TPU | 58.0 × 30.0 × 1.6 | 2.779 | 3.36 | Flat, as exported | Solid, 100% | Inspect overhangs; avoid TPU supports |
| CameraCradle_TPU | 1 | TPU | 28.8 × 28.9 × 15.0 | 3.548 | 4.29 | Cradle back on bed | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_1_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_2_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_3_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_4_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |

Both printers: check reported individual-part bed fit; allow brim/support margin. Whole assembled mesh is for inspection, never slice it as a single part. STEP parts retain assembly positions; STL/3MF parts are already placed/oriented at z=0. A 3MF is generic Core 3MF, not a Bambu slicer project.

Use PETG for the rigid parts and TPU 95A for the cradle/feet/pad. PLA is only for interface coupons. Keep supports out of motor slots and pin bores, remove all debris, drill/ream only to the verified nominal interface and inspect layer bonding. v2: the frame underside is flat, so the motor pads and their screw seats print directly on the bed.

Optional fit coupons: print one `FitCoupon_Frame_PLA` and one `FitCoupon_Guard_PLA` flat as exported, 4 walls, 100% infill, no support. These test the four motor slots and paired locating pins/bores; they are excluded from assembly mass.

`FitCoupon_GuardSpoke_PETG`: one guard spoke with its pad and a 44 deg piece of ring. Print it in PETG, pad down, with the guard settings, then break-test the spoke and ring joint by hand before printing four guards.
