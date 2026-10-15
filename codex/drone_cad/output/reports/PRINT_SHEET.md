# Print sheet — Astra 01

One part per manufacturing file; each listed file quantity 1. Four separate guard and foot files are included. 0.4 mm nozzle, 0.2 mm layers. These are starting settings requiring slicer review and fit coupons. Masses assume solid CAD volume and placeholder densities.

| Part | Qty | Material | Print box mm | Volume cm³ | Solid mass g | Orientation | Walls / infill | Supports |
|---|---:|---|---|---:|---:|---|---|---|
| Frame_PETG | 1 | PETG | 135.0 × 135.0 × 55.5 | 35.464 | 45.04 | Flat, as exported | 4 walls / 35% gyroid | Tree, build plate only; inspect interfaces |
| Guard_1_PETG | 1 | PETG | 97.2 × 97.2 × 30.0 | 8.372 | 10.63 | Ring down | 4 walls / 35% gyroid | Tree, build plate only; inspect interfaces |
| Guard_2_PETG | 1 | PETG | 97.2 × 97.2 × 30.0 | 8.372 | 10.63 | Ring down | 4 walls / 35% gyroid | Tree, build plate only; inspect interfaces |
| Guard_3_PETG | 1 | PETG | 97.2 × 97.2 × 30.0 | 8.372 | 10.63 | Ring down | 4 walls / 35% gyroid | Tree, build plate only; inspect interfaces |
| Guard_4_PETG | 1 | PETG | 97.2 × 97.2 × 30.0 | 8.372 | 10.63 | Ring down | 4 walls / 35% gyroid | Tree, build plate only; inspect interfaces |
| BottomPlate_PETG | 1 | PETG | 64.0 × 50.0 × 9.0 | 8.344 | 10.60 | Flat, as exported | 4 walls / 35% gyroid | Underside sensor nest needs support or flip and support stops |
| BatteryPad_TPU | 1 | TPU | 58.0 × 30.0 × 1.6 | 2.779 | 3.36 | Flat, as exported | Solid, 100% | Inspect overhangs; avoid TPU supports |
| CameraCradle_TPU | 1 | TPU | 28.8 × 28.9 × 15.0 | 3.021 | 3.66 | Cradle back on bed | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_1_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_2_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_3_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |
| Foot_4_TPU | 1 | TPU | 12.0 × 12.0 × 14.0 | 0.967 | 1.17 | As exported, feet base up | 4 walls / 30% gyroid, slow TPU profile | Inspect overhangs; avoid TPU supports |

Both printers: check reported individual-part bed fit; allow brim/support margin. Whole assembled mesh is for inspection, never slice it as a single part. STEP parts retain assembly positions; STL/3MF parts are already placed/oriented at z=0. A 3MF is generic Core 3MF, not a Bambu slicer project.

Use PETG for the rigid parts and TPU 95A for the cradle/feet/pad. PLA is only for interface coupons. Keep supports out of motor slots and pin bores, remove all debris, drill/ream only to the verified nominal interface and inspect layer bonding. Frame motor pads need a stable support setup because the lower chords extend beneath them.

Optional fit coupons: print one `FitCoupon_Frame_PLA` and one `FitCoupon_Guard_PLA` flat as exported, 4 walls, 100% infill, no support. These test the four motor slots and paired locating pins/bores; they are excluded from assembly mass.
