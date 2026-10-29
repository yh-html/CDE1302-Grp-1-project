"""Printed-set mass and T/W for each candidate pick (reads output/reports/candidates.json). Common parts are ESTIMATES (flagged)."""
import json, numpy as np
from pathlib import Path
R = Path(__file__).resolve().parents[1] / 'output' / 'reports'
d = json.load(open(R / 'candidates.json'))
RHO = 1.27e-3
pad = (np.pi * (10 ** 2 - 4 ** 2) - 4 * np.pi * 1.15 ** 2) * 3.0 * RHO * 4          # 4 D20 pads
boss = 8 * np.pi * (3.5 ** 2 - 1.15 ** 2) * 5.0 * RHO                                # 8 bosses D7 x 5 (PLACEHOLDER height)
ring_h, ring_t, spoke_a, n_sp = 6.0, 1.68, 10.0, 5                                    # v3 guard PLACEHOLDERS (v2 rev B: 8.9, 3.0, ~13)
arc = 2 * np.pi * (43 + ring_t / 2) * 240 / 360
guard = (arc * ring_t * ring_h + n_sp * 33 * spoke_a + n_sp * 1.2 * ring_h * 4 + np.pi * (10 ** 2 - 4 ** 2 - 4 * 1.15 ** 2 / 1) * 3.0) * RHO
plate, feet = 5.0, 4.7                                                                # plate: ESTIMATE; feet: v2 value 4 x 1.17
HW = 158.0; CT = 2.9625e-7; W = 3232.0
tw = lambda m: 4 * CT * W ** 2 / (m / 1000 * 9.81)
out = dict(pads_g=round(pad, 2), bosses_g=round(boss, 2), guard_each_g_est=round(guard, 2), plate_g_est=plate, feet_g=feet, hardware_g=HW, rows={})
def row(label, frame):
    s = frame + 4 * guard + plate + feet
    out['rows'][label] = dict(frame_g=round(frame, 1), printed_set_g=round(s, 1), all_up_g=round(s + HW, 1), TW=round(tw(s + HW), 2))
row('stock (measured: frame 25.9 + panel 7.9 + guards 25.7 + legs 8.9)', 25.9)
out['rows']['stock (measured)'] = dict(frame_g=25.9, printed_set_g=68.5, all_up_g=226.5, TW=round(tw(226.5), 2))
del out['rows']['stock (measured: frame 25.9 + panel 7.9 + guards 25.7 + legs 8.9)']
out['rows']['stock_plus (reported)'] = dict(frame_g=28.7, printed_set_g=79.9, all_up_g=237.9, TW=round(tw(237.9), 2))
for k, v in d['picks'].items():
    if v: row(k, v['structure_mass_g'] + pad + boss)
print(json.dumps(out, indent=1)); json.dump(out, open(R / 'budget.json', 'w'), indent=1)
