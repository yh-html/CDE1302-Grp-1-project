"""Re-run the beam model on the FINAL v3 arm parameters (no front/rear cross link) against the calibrated stock arm."""
import json, numpy as np
from pathlib import Path
import topology as T
ROOT = Path(__file__).resolve().parents[1]
p = {k: v['value'] for k, v in json.load(open(ROOT / 'config/parameters.json'))['parameters'].items()}
stock_P = dict(d_tip=9, d_root=9, e_tip=9.2, e_root=9.2, wc=1.5, wd=1.2, cells=3)
fS, tS = T.build('S', stock_P, dict(sec=T.rect(20, 9), w=20)); S = T.stiff(fS, *tS[0])
res = {}
for label, cross in (('with_cross_link', True), ('final_no_cross_link', False)):
    P = dict(d_root=p['arm_d_root'], d_tip=p['arm_d_tip'], w_root=p['arm_w_root'], w_tip=p['arm_w_tip'], t=p['arm_wall'])
    f, tips = T.build('A', P, dict(sec=T.box(p['body_w'], p['body_d'], p['arm_wall']), w=p['body_w'], cross=cross))
    k = T.stiff(f, *tips[0]); bd = T.breakdown(f)
    res[label] = dict(ratio_vs_stock={a: round(float(k[a] / S[a]), 2) for a in k}, model_structure_mass_g=round(sum(v for a, v in bd.items() if a != 'padstub'), 2))
res['criterion_X'] = dict(k_vert_min=0.75, k_tors_min=0.75, k_lat_min=0.25)
r = res['final_no_cross_link']['ratio_vs_stock']
res['criterion_X_met'] = bool(r['k_vert'] >= .75 and r['k_tors'] >= .75 and r['k_lat'] >= .25)
print(json.dumps(res, indent=1)); (ROOT / 'output/reports/fem_final.json').write_text(json.dumps(res, indent=1))
