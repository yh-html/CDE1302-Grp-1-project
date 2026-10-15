# CDE1302 frame stack: print sheet (draft, 10 Oct)

Source: Fusion "CDE1302 Frame v1" (CAD Workshop Final). Every file is one part; print the count shown.
Settings assume a 0.4 mm nozzle and 0.2 mm layers. `min_wall` = 1.6 mm = 4 perimeters.

| File | Material | Qty | Size (mm) | Orientation | Settings |
|---|---|---|---|---|---|
| Frame_PETG.3mf | PETG | 1 | 133.5 × 133.5 × 6.6 | as exported (channel floor on the bed) | 4 walls, 40 % gyroid, no supports; A1 or A1 mini |
| PropGuard_PETG.3mf | PETG | 4 | Ø100 × 4 | as exported (flat) | 4 walls, 30 %, no supports; A1: all 4 on one plate, A1 mini: 2 per plate on the diagonal |
| BottomPlate_PETG.3mf | PETG | 1 | 61.6 × 49.6 × 5.5 | as exported (corner stops up) | 4 walls, 40 %, no supports (the 0.8 mm MTF recess bridges fine) |
| TopCover_PETG.3mf | PETG | 1 | 46 × 46 × 6 | as exported (pocket up) | 4 walls, 40 %, no supports; the 1.2 mm flex beams print as 3 perimeters, keep them |
| CameraMount-Base_PETG.3mf | PETG | 1 | 21.4 × 31.4 × 23.4 | as exported (base on the bed, cheeks up) | 4 walls, 40 %, no supports |
| CameraMount-Cradle_TPU.3mf | TPU 95A | 1 | 30 × 27.8 × 30 | lay the open back on the bed (exported tilted at cam_tilt) | 3 walls, 30 %, ≤ 30 mm/s, no supports |
| Foot_TPU.3mf | TPU 95A | 4 | Ø12 × 15 | flip: cap on the bed, open tube up | 5 walls (solid wall), no supports |
| BatteryPad_TPU.3mf | TPU 95A | 1 (optional) | 58 × 30 × 1 | flat | solid |
| TestCoupon_PLA.3mf + TestCoupon-GuardPad_PLA.3mf | PLA | 1 + 1 | 100 × 34 × 6.6, Ø23.5 × 2 | flat | regenerated at frame_t = 5 mm, 4 slots |

Densities used for the mass check are generic placeholders (PETG 1.27, TPU 1.21 g/cm³), and parts are counted as solid.

## Hardware
- Motors: 4 × M2×10 per motor (16 total, stock used 8), from below: frame pad 5 mm + guard pad 2 mm → **3 mm into the motor**. Check the Flywoo thread depth before going longer.
- FC stack: unchanged (5 mm rubber isolators, M2×30 nylon standoffs). Run one M2 nut onto each stud ~2.4 mm above the standoff shoulder and fix it with CA/threadlocker: these are the slide-lock heads for the top cover.
- Top cover: drop it 6 mm behind its final position (heads through the round keyholes), push forward until each stud clicks past the neck.
- Bottom plate: 4 × M2.5×30 standoffs at (±21.4, ±21.4). Bottom screw goes up through foot cap + plate into the standoff (head inside the foot), top screw through the 5 mm frame boss.
- Camera: base to frame with 2 × M2 (+ nyloc nuts); cradle on 2 × M3×6–8 self-tapped into the 4 mm TPU side walls (snug, not tight, so the detent can click).
- Battery: velcro strap through the two plate slots at x = 0. MTF-02P: foam tape in the underside recess (holes once the spacing is known). ELRS: zip tie through the slots beside the pocket, antenna coax out of the rear notch.
