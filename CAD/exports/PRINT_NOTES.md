# CDE1302 frame stack: print sheet (rev 2, organic)

Source: Fusion "CDE1302 Frame v1" (CAD Workshop Final). Rev 1 files are in `rev1/` for reference.
Settings assume a 0.4 mm nozzle and 0.2 mm layers. `min_wall` = 1.6 mm = 4 perimeters.

| File | Material | Qty | Size (mm) | Orientation | Settings |
|---|---|---|---|---|---|
| Frame_PETG.3mf | PETG | 1 | 133.5 × 133.5 × 30 | as exported: flat underside (floor, lower tubes, pads, tie rods, cheek feet) on the bed | 4 walls, 30 % gyroid; **tree supports** (auto) for the arched upper tubes, the deck ring, the receiver pocket and the cheek pivot bosses; support Z gap 0.2 mm; A1 or A1 mini |
| PropGuard_PETG.3mf | PETG | 4 | Ø100 × 20.1 | **flip**: ring flat on the bed, pad up | 4 walls, 30 %; tree supports under the spokes and pad; A1: all 4 on one plate, A1 mini: 2 per plate on the diagonal |
| BottomPlate_PETG.3mf | PETG | 1 | 61.6 × 49.6 × 5.5 | as exported (corner stops up) | 4 walls, 40 %, no supports |
| CameraMount-Cradle_TPU.3mf | TPU 95A | 1 | 30 × 27.8 × 30 | lay the open back on the bed (exported tilted at cam_tilt) | 3 walls, 30 %, ≤ 30 mm/s, no supports |
| Foot_TPU.3mf | TPU 95A | 4 | Ø12 × 15 | flip: cap on the bed | 5 walls, no supports |
| BatteryPad_TPU.3mf | TPU 95A | 1 (optional) | 58 × 30 × 1 | flat | solid |
| TestCoupon_PLA.3mf + TestCoupon-GuardPad_PLA.3mf | PLA | 1 + 1 | 100 × 34 × 6.6, Ø23.5 × 2 | flat | still valid for the pad, pins, slots, FC holes, strap slot and detent holes |

The top cover and the separate camera base are gone: the deck and the camera cheeks are now part of the frame.
Densities for the mass check are generic placeholders (PETG 1.27, TPU 1.21 g/cm³), and parts are counted as solid.

## Assembly and hardware
- **Bottom standoffs first.** 4 × M2.5×30 nylon standoffs under the floor at (±21.4, ±21.4). The top screw is an **M2.5×30** dropped from the deck top down through each hollow corner leg (26 mm) into the standoff. The bottom screw goes up through the foot cap and plate into the standoff, with the head inside the TPU foot.
- **FC (fixed, never removed).** Drop it through the deck's ring opening onto its 5 mm isolators on the floor bosses. Fasten with M2 screws from below through the floor, nuts on top (reachable through the opening). Solder the motor leads, then zip-tie them to the arm tubes.
- **Motors.** 4 × M2×10 per motor (16 total) from below: frame pad 5 mm + guard pad 2 mm, so 3 mm into the motor. Check the Flywoo thread depth before going longer. Each guard drops onto the 2 pins on its pad first.
- **Camera.** The TPU cradle sits between the frame cheeks on 2 × M3×6–8, self-tapped into its 4 mm side walls. Keep them snug, not tight, so the detent nubs can click between 30/45/60/75°.
- **Receiver.** Sits in the pocket on the deck tail, held by a zip tie through the two slots; the antenna coax exits through the rear notch.
- **Battery** strapped with velcro through the bottom-plate slots. **MTF-02P**: foam tape in the underside recess (holes once the spacing is known).
