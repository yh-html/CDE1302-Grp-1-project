"""Run: ..\..\drone_cad\.venv\Scripts\python.exe compare_candidates.py
Sizes each candidate to the lightest design whose tip stiffness (vertical, lateral, torsion) is >= RATIO x the
calibrated stock arm, then reports mass, stiffness ratios and plan area under the prop discs."""
import itertools, json, sys
from pathlib import Path
import numpy as np
from PIL import Image
import topology as T

OUT = Path(__file__).resolve().parents[1] / 'output'
CRITERIA = {'X_vert0.75_tors0.75_lat0.25': (0.75, 0.75, 0.25), 'Y_vert1.0_tors1.0_lat0.5': (1.0, 1.0, 0.5), 'Z_vert0.5_tors0.5_lat0.15': (0.5, 0.5, 0.15)}
q = lambda n: round(n * T.LINE, 3)    # n print lines
PAD_G = (np.pi * (10 ** 2 - 4 ** 2) - 4 * np.pi * 1.15 ** 2) * 3.0 * T.RHO   # D20 pad, D8 hole, 4x D2.3, 3 mm thick (per motor)
BOSS_G = 8 * np.pi * (3.5 ** 2 - 1.15 ** 2) * 5.0 * T.RHO                      # 8 bosses D7 x 5 mm, PLACEHOLDER height
MINWALL = 1.6

# ---- stock reference (calibrated to the measured arm volume, see README of the report) ----
stock_P = dict(d_tip=9, d_root=9, e_tip=9.2, e_root=9.2, wc=1.5, wd=1.2, cells=3)
stock_body = dict(sec=T.rect(20, 9), w=20)       # stock body is a stiff plate truss: modelled as a stiff beam
fS, tipsS = T.build('S', stock_P, stock_body)
S = T.stiff(fS, *tipsS[0])
armS = T.breakdown(fS)
areaS, _ = T.plan_area(fS)


def evaluate(kind, P, body):
    f, tips = T.build(kind, P, body)
    k = [T.stiff(f, *t) for t in tips[:1]][0]
    bd = T.breakdown(f); mass = sum(v for kk, v in bd.items() if kk != 'padstub')
    area, arr = T.plan_area(f)
    return f, dict(mass=mass, **{kk: k[kk] / S[kk] for kk in ('k_vert', 'k_lat', 'k_axial', 'k_tors')}, area=area), arr





import pickle
allres = {'A': [], 'B': [], 'C': []}
def rec(kind, P, bs, bw):
    f, r, arr = evaluate(kind, P, dict(sec=bs, w=bw))
    allres[kind].append((P, bs, bw, r, f, arr))
BODIES = [(T.box(6, 6, q(4)), 6), (T.box(8, 8, q(4)), 8), (T.box(8, 8, q(5)), 8), (T.rect(q(8), 6), q(8)), (T.twin(q(4), 6, 4.5), 6)]
for dr, wr, wt, dt, tl in itertools.product((9,), (9, 12), (4.5, 6), (6, 9), (4, 5)):
    for bs, bw in BODIES: rec('A', dict(d_root=dr, w_root=wr, w_tip=wt, d_tip=dt, t=q(tl)), bs, bw)
for dr, dt, er, et, wc, wd, nc in itertools.product((9,), (6, 9), (6, 9.2), (5, 6, 9.2), (q(4), q(5)), (q(4), q(5)), (2, 3, 4)):
    if et > er: continue
    for bs, bw in BODIES: rec('B', dict(d_root=dr, d_tip=dt, e_root=er, e_tip=et, wc=wc, wd=wd, cells=nc), bs, bw)
for dr, dt, wb, wx, dx, rn in itertools.product((9,), (6, 9), (q(5), q(8)), (q(4), q(6)), (4.5, 9), (31.8, 41.3)):
    for bs, bw in BODIES: rec('C', dict(d_root=dr, d_tip=dt, wb=wb, wx=wx, dx=dx, r_far=50.9, r_near=rn), bs, bw)
summary = dict(assumptions=dict(E_MPa=T.E, G_MPa=round(T.G, 1), rho_g_mm3=T.RHO, line_mm=T.LINE),
               stock_model=dict(stiffness={k: round(float(v), 3) for k, v in S.items()}, arm_mass_breakdown_g=armS,
                                plan_area_under_props_excl_pads_mm2=round(areaS)),
               pad_g_each=round(PAD_G, 3), boss_g_total=round(BOSS_G, 3), picks={})
for cname, (rv, rt, rl) in CRITERIA.items():
    for name, lst in allres.items():
        okl = [x for x in lst if x[3]['k_vert'] >= rv and x[3]['k_tors'] >= rt and x[3]['k_lat'] >= rl]
        key = f'{name}|{cname}'
        if not okl: summary['picks'][key] = None; continue
        P, bs, bw, r, f, arr = min(okl, key=lambda x: x[3]['mass'])
        bd = {k: float(v) for k, v in T.breakdown(f).items() if k != 'padstub'}
        summary['picks'][key] = dict(params={k: round(float(v), 3) for k, v in P.items()}, body_section={k: round(v, 2) for k, v in bs.items()},
            structure_mass_g=round(r['mass'], 2), mass_breakdown_g=bd,
            stiffness_vs_stock={k: round(float(r[k]), 2) for k in ('k_vert', 'k_lat', 'k_axial', 'k_tors')}, plan_area_under_props_mm2=round(r['area']))
        if cname.startswith('X'): Image.fromarray((arr * 255).astype('uint8')).save(Path(__file__).parent / f'_plan_{name}.png')
# also the max-min-ratio design per candidate
summary['best_balance'] = {}
for name, lst in allres.items():
    P, bs, bw, r, f, arr = max(lst, key=lambda x: min(x[3]['k_vert'], x[3]['k_lat'], x[3]['k_tors']) / x[3]['mass'])
    summary['best_balance'][name] = dict(params={k: round(float(v), 3) for k, v in P.items()}, structure_mass_g=round(r['mass'], 2),
        stiffness_vs_stock={k: round(float(r[k]), 2) for k in ('k_vert', 'k_lat', 'k_axial', 'k_tors')}, plan_area_under_props_mm2=round(r['area']))
print(json.dumps(summary, indent=1))
(OUT / 'reports').mkdir(parents=True, exist_ok=True)
json.dump(summary, open(OUT / 'reports' / 'candidates.json', 'w'), indent=1)
