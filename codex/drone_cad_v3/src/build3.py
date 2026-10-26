"""v3 build: geometry, per-part exports (STEP/STL/3MF), renders, validation report.
Run: ..\\..\\drone_cad\\.venv\\Scripts\\python.exe -u build3.py   (add --geometry-only to skip validation)"""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np
import cadquery as cq
import trimesh
from PIL import Image, ImageDraw
from geometry import *
import parts3
from build import export_3mf, log
from validate import common_volume, exact_distance, clearance_distance, view_frustum, overlap_bb
from render import render, color
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

ROOT = Path(__file__).resolve().parents[1]; OUT = ROOT / 'output'
PETG = 'PETG'; TPU = 'TPU'
GUARD_NAMES = [f'Guard_{i}_PETG' for i in range(1, 5)]
FOOT_NAMES = [f'Foot_{i}_TPU' for i in range(1, 5)]


def cyl_faces(shape):
    out = []
    for f in shape.Faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() == GeomAbs_Cylinder:
            c = s.Cylinder(); ax = c.Axis(); d = ax.Direction()
            if abs(abs(d.Z()) - 1) < 1e-6:
                loc = ax.Location(); out.append((loc.X(), loc.Y(), c.Radius()))
    return out


def match_holes(shape, expected, tol=.05):
    """expected: [(x,y,diameter)]; returns list of (expected, best found dx,dy,dd) and ok flag."""
    found = cyl_faces(shape); rows = []; ok = True
    for x, y, dia in expected:
        best = None
        for fx, fy, fr in found:
            e = max(abs(fx - x), abs(fy - y), abs(2 * fr - dia))
            if best is None or e < best[0]: best = (e, fx, fy, 2 * fr)
        good = best is not None and best[0] <= tol
        ok &= good; rows.append(dict(expected=[x, y, dia], found=None if best is None else [round(best[1], 4), round(best[2], 4), round(best[3], 4)],
                                    max_error=None if best is None else round(best[0], 4), ok=good))
    return ok, rows


def raster(mesh, ext=130.0, res=.25):
    n = int(2 * ext / res); img = Image.new('L', (n, n), 0); d = ImageDraw.Draw(img)
    v = mesh.vertices[:, :2]
    for f in mesh.faces:
        d.polygon([((v[i][0] + ext) / res, (ext - v[i][1]) / res) for i in f], fill=1)
    return np.array(img, bool)


def support_area(mesh):
    """downward-facing area steeper than 45 deg from vertical i.e. normal.z < -cos45, above 0.25 mm."""
    n = mesh.face_normals; a = mesh.area_faces; zc = mesh.triangles_center[:, 2]
    sel = (n[:, 2] < -math.cos(math.radians(45)) - 0.01) & (n[:, 2] > -0.99) & (zc > .25)
    return float(a[sel].sum())


def bridge_area(mesh):
    """flat downward faces (bridges) above 0.25 mm: need bridging, not supports, if narrow"""
    n = mesh.face_normals; a = mesh.area_faces; zc = mesh.triangles_center[:, 2]
    return float(a[(n[:, 2] <= -0.99) & (zc > .25)].sum())


def overhang_by_threshold(mesh, centre=None, rmin=None):
    """downward-facing area (mm2) whose face is less than T degrees from horizontal, T = 30/40/45/50 (what a Bambu Studio
    'threshold angle' of T would support); optionally only faces further than rmin from centre (xy). z > 0.25 mm."""
    n = mesh.face_normals; a = mesh.area_faces; c = mesh.triangles_center
    ang = np.degrees(np.arccos(np.clip(-n[:, 2], -1, 1)))          # 0 = flat underside, 90 = vertical
    sel = (n[:, 2] < 0) & (c[:, 2] > mesh.bounds[0, 2] + .25)      # exclude the bed face (pad bottom)
    if centre is not None: sel &= np.hypot(c[:, 0] - centre[0], c[:, 1] - centre[1]) >= rmin
    return {str(T): round(float(a[sel & (ang < T - 0.3)].sum()), 1) for T in (30, 40, 45, 50, 55)}


def box_inertia(m, sx, sy, sz): return np.diag([m * (sy ** 2 + sz ** 2) / 12, m * (sx ** 2 + sz ** 2) / 12, m * (sx ** 2 + sy ** 2) / 12])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--geometry-only', action='store_true'); args = ap.parse_args()
    for d in ['step', 'stl', '3mf', 'renders', 'reports']: (OUT / d).mkdir(exist_ok=True)
    (OUT / 'reports/error.txt').unlink(missing_ok=True)
    data = json.loads((ROOT / 'config/parameters.json').read_text()); p = {k: v['value'] for k, v in data['parameters'].items()}
    shapes = {}
    log('Build Frame'); shapes['Frame_PETG'] = parts3.frame(p)
    for i, (x, y, a) in enumerate(motor_positions(p), 1):
        log(f'Build Guard_{i}'); g, _, _ = parts3.guard(p, a); shapes[f'Guard_{i}_PETG'] = g.translate((x, y, 0))
    log('Build plate'); shapes['BottomPlate_PETG'] = parts3.bottom_plate(p)
    shapes['BatteryPad_TPU'] = parts3.battery_pad(p)
    shapes['FlowCover_TPU'] = parts3.flow_cover(p)
    cl = parts3.cradle(p); shapes['CameraCradle_TPU'] = tilt(cl, p)
    ft = parts3.foot(p)
    for i, (sx, sy) in enumerate([(sx, sy) for sx in (-1, 1) for sy in (-1, 1)], 1): shapes[f'Foot_{i}_TPU'] = ft.translate((sx * p['foot_x'], sy * p['foot_y'], 0))
    env = parts3.envelopes(p)
    meshes = {}; prints = {}; assembly = cq.Assembly(name='CDE1302_Astra_v3')
    for name, s in shapes.items():
        if not s.isValid() or len(s.Solids()) != 1: raise ValueError(f'{name}: invalid or disconnected solid')
        log('Export ' + name)
        cq.exporters.export(s, str(OUT / 'step' / f'{name}.step'))
        m = mesh_of(s, p['mesh_tolerance'], p['mesh_angle']); meshes[name] = m
        pr = m.copy()
        if name.startswith('Frame'): pr.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]))   # flat motor face on the bed
        if name.startswith('Foot'): pr.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]))
        if name.startswith('CameraCradle'):
            pr = mesh_of(cl, p['mesh_tolerance'], p['mesh_angle']); pr.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [0, 1, 0]))
        pr.apply_translation([-pr.bounds[:, 0].mean(), -pr.bounds[:, 1].mean(), -pr.bounds[0, 2]])
        pr.export(OUT / 'stl' / f'{name}.stl'); export_3mf(pr, OUT / '3mf' / f'{name}.3mf', name); prints[name] = pr
        assembly.add(s, name=name, color=cq.Color(*color(name)))
    assembly.save(str(OUT / 'step/Assembly_printed.step'))
    coupons = {}
    for name, s in parts3.fit_coupons(p).items():
        cq.exporters.export(s, str(OUT / 'step' / f'{name}.step'))
        m = mesh_of(s, p['mesh_tolerance'], p['mesh_angle'])
        if name.endswith('ArmBox_PETG'): m.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]))
        m.apply_translation([-m.bounds[:, 0].mean(), -m.bounds[:, 1].mean(), -m.bounds[0, 2]])
        m.export(OUT / 'stl' / f'{name}.stl'); export_3mf(m, OUT / '3mf' / f'{name}.3mf', name); coupons[name] = (s, m)
    for name, s in env.items(): assembly.add(s, name=name, color=cq.Color(*color(name), .5 if name.startswith('Prop') else 1))
    assembly.save(str(OUT / 'step/Assembly_with_hardware.step'))
    trimesh.util.concatenate(list(meshes.values())).export(OUT / 'stl/Assembly_VISUAL_ONLY.stl')
    scene = trimesh.Scene()
    for name, m in meshes.items():
        mc = m.copy(); mc.visual.face_colors = [int(255 * v) for v in color(name)] + [255]; scene.add_geometry(mc, node_name=name)
    scene.export(OUT / 'Assembly.glb')
    log('Render')
    em = {k: mesh_of(v, .12, .22) for k, v in env.items()}
    try:
        for view in ['iso', 'top', 'side', 'underside', 'front']: render(meshes, OUT / 'renders' / f'assembly_{view}.png', 'v3 hollow-box arms / ' + view, view)
        render(meshes, OUT / 'renders/assembly_exploded.png', 'v3 / exploded', explode=True)
        render(meshes, OUT / 'renders/clearance_iso.png', 'v3 / hardware envelopes', hardware=em)
        render(meshes, OUT / 'renders/clearance_side.png', 'v3 / side with envelopes', view='side', hardware=em)
        render({'Frame_PETG': meshes['Frame_PETG']}, OUT / 'renders/frame_detail.png', 'v3 frame / closed hollow-box arms')
        render({'Guard_1': meshes['Guard_1_PETG']}, OUT / 'renders/guard_detail.png', 'v3 guard / five keel spokes')
        fc = (p['flow_x'], 0, p['plate_bottom_z'] - 6)
        sub = {k: meshes[k] for k in ['FlowCover_TPU', 'BottomPlate_PETG'] + FOOT_NAMES}
        render(sub, OUT / 'renders/flow_cover_underside.png', 'MTF-02P flow cover (TPU) / underside', 'underside', hardware={'MTF02P': em['MTF02P']}, center=fc, zoom=2.6)
        render(sub, OUT / 'renders/flow_cover_side.png', 'MTF-02P flow cover (TPU) / side', 'side', hardware={'MTF02P': em['MTF02P']}, center=fc, zoom=2.6)
    except Exception as e:
        log('render failed: ' + repr(e))
    if not args.geometry_only:
        validate(p, data, shapes, meshes, prints, env, cl, coupons)
    log('Build complete')


def validate(p, data, shapes, meshes, prints, env, cl, coupons):
    checks = []; details = {}
    def chk(name, status, value=None, limit=None, note=''): checks.append(dict(name=name, status=status, value=value, limit=limit, note=note))
    m_ = p['motor_side'] / 2; TOL = .05
    # ---- validity ----
    for n, s in shapes.items():
        chk(f'{n}/brep_valid_single_solid', 'pass' if s.isValid() and len(s.Solids()) == 1 and s.Volume() > 0 else 'fail', len(s.Solids()))
        mm = meshes[n]
        chk(f'{n}/mesh_watertight', 'pass' if mm.is_watertight and mm.is_winding_consistent and mm.volume > 0 else 'fail', dict(watertight=bool(mm.is_watertight), winding=bool(mm.is_winding_consistent)),
            note=f'mesh volume {mm.volume:.1f} vs B-rep {s.Volume():.1f}')
    # ---- interface holes (tolerance 0.05 mm on centre and diameter) ----
    fcr = p['fc_hole_spacing'] / 2 * math.sqrt(2)
    fcp = [(fcr, 0), (0, fcr), (-fcr, 0), (0, -fcr)]; bm = [tuple(map(float, q)) for q in p['bottom_mount_points']]
    exp = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            mx, my = sx * m_, sy * m_
            exp += [(mx + 6, my, 2.3), (mx - 6, my, 2.3), (mx, my + 6, 2.3), (mx, my - 6, 2.3), (mx, my, 8.0), (mx, my, 20.0)]
    exp += [(x, y, 2.3) for x, y in fcp] + [(x, y, 2.8) for x, y in bm]
    ok, rows = match_holes(shapes['Frame_PETG'], exp); details['frame_holes'] = rows
    chk('Frame/fixed_interface_holes', 'pass' if ok else 'fail', f'{len(exp)} features, worst error {max(r["max_error"] for r in rows):.4f} mm', f'<= {TOL} mm', 'motor pattern, centre D8, pad D20, FC pattern, bottom-mount holes')
    for i, (x, y, a) in enumerate(motor_positions(p), 1):
        e = [(x + 6, y, 2.3), (x - 6, y, 2.3), (x, y + 6, 2.3), (x, y - 6, 2.3), (x, y, 8.0), (x, y, 20.0)]
        ok, rows = match_holes(shapes[f'Guard_{i}_PETG'], e); details[f'guard_{i}_holes'] = rows
        chk(f'Guard_{i}/fixed_interface_holes', 'pass' if ok else 'fail', f'worst error {max(r["max_error"] for r in rows):.4f} mm', f'<= {TOL} mm')
    ok, rows = match_holes(shapes['BottomPlate_PETG'], [(x, y, 2.75) for x, y in bm]); details['plate_holes'] = rows
    chk('BottomPlate/mount_holes', 'pass' if ok else 'fail', f'worst error {max(r["max_error"] for r in rows):.4f} mm', f'<= {TOL} mm', 'D2.75 as bottom_panel.stl')
    # pad thickness (vertical probe 8.5 mm from the motor centre, away from the arm)
    def vthick(shape, x, y): return line_len(shape, (x, y, 0), (0, 0, 1), 60)
    def line_len(shape, pt, d, span):
        a = np.array(pt) - span * np.array(d); b = np.array(pt) + span * np.array(d)
        cut = shape.intersect(cq.Edge.makeLine(V(*a), V(*b))); return sum(e.Length() for e in cut.Edges())
    th = [line_len(shapes['Frame_PETG'], (sx * (m_ + 6), sy * (m_ + 6), 0), (0, 0, 1), 40) for sx in (-1, 1) for sy in (-1, 1)]
    chk('Frame/motor_pad_thickness', 'pass' if all(abs(t - 3.0) <= TOL for t in th) else 'fail', [round(t, 3) for t in th], '3.0 +- 0.05')
    th = [line_len(shapes[f'Guard_{i}_PETG'], (x + 6 * np.sign(x), y + 6 * np.sign(y), 0), (0, 0, 1), 40) for i, (x, y, a) in enumerate(motor_positions(p), 1)]
    chk('Guard/pad_thickness', 'pass' if all(abs(t - 3.0) <= TOL for t in th) else 'fail', [round(t, 3) for t in th], '3.0 +- 0.05')
    gz = [shapes[n].BoundingBox() for n in GUARD_NAMES]; fz = shapes['Frame_PETG'].BoundingBox()
    ft_ = float(meshes['Frame_PETG'].bounds[1, 2]); gb_ = [float(meshes[n].bounds[0, 2]) for n in GUARD_NAMES]
    chk('stack_order', 'pass' if abs(ft_ - 3.0) < .01 and all(abs(b - 3.0) < .01 for b in gb_) else 'fail', dict(frame_top=round(ft_, 3), guard_bottoms=[round(b, 3) for b in gb_]), 'guard bottom = frame top = 3.0 (mesh bounds; B-rep bounding boxes are padded)')
    # ---- guard geometry ----
    ring_id = p['guard_od'] - 2 * p['guard_wall'] - 2 * p['guard_bead_w']; tipgap = (ring_id - p['prop_d']) / 2
    chk('Guard/prop_tip_gap', 'pass' if tipgap >= 3 else 'fail', round(tipgap, 2), '>= 3 mm')
    centres = [(x, y) for x, y, a in motor_positions(p)]
    gaps = []
    for i in range(4):
        for j in range(i + 1, 4):
            d = math.dist(centres[i], centres[j]) - (p['guard_od'] + 2 * p['guard_lip'])
            gaps.append(d)
    chk('Guard/neighbour_gap', 'pass' if min(gaps) >= 10 else 'fail', round(min(gaps), 2), '>= 10 mm', 'analytic: centre spacing minus OD incl. lip (the C rings only reduce it)')
    # ---- prop-disc clearance ----
    pc = {}
    for n in ['Frame_PETG', 'BottomPlate_PETG'] + GUARD_NAMES + FOOT_NAMES + ['CameraCradle_TPU']:
        dmin = 1e9
        for k in range(1, 5):
            d, how = clearance_distance(shapes[n], env[f'Prop_{k}'], 4.0); dmin = min(dmin, d)
        pc[n] = round(dmin, 2)
    chk('prop_disc_clearance', 'pass' if all(v >= 4.0 - 1e-6 for v in pc.values()) else 'fail', pc, '>= 4 mm to the 2 mm swept prop disc', 'ring/spoke bounds checked exactly where the AABB bound is below 4 mm')
    # ---- bed fit and supports ----
    rows = []; vol = {}
    for n, pr in prints.items():
        ext = pr.extents; fit_a1 = bool(max(ext[:2]) <= 256 and ext[2] <= 256); fit_mini = bool(max(ext[:2]) <= 180 and ext[2] <= 180)
        sup = support_area(pr); br = bridge_area(pr); mat = PETG if n.endswith('PETG') else TPU; dens = p['petg_density'] if mat == PETG else p['tpu_density']
        mass = shapes[n].Volume() * dens / 1000
        rows.append(dict(name=n, material=mat, print_bbox_mm=[round(float(v), 1) for v in ext], fits_A1_mini=fit_mini, fits_A1=fit_a1, volume_mm3=round(shapes[n].Volume(), 1),
                         mass_g=round(mass, 3), support_area_mm2=round(sup, 1), flat_bridge_area_mm2=round(br, 1)))
    details['parts'] = rows
    chk('bed_fit_A1', 'pass' if all(r['fits_A1'] for r in rows) else 'fail', {r['name']: r['print_bbox_mm'] for r in rows if not r['fits_A1']}, '<= 256 mm')
    chk('bed_fit_A1_mini', 'pass' if all(r['fits_A1_mini'] for r in rows) else 'fail', [r['name'] for r in rows if not r['fits_A1_mini']], '<= 180 mm',
        'parts listed do not fit the A1 mini (frame diagonal span is 133 mm; check)')
    # ---- wall checks ----
    t = p['arm_wall']; walls = {}
    fr = shapes['Frame_PETG']; m0 = m_
    u = np.array([-1., -1.]) / math.sqrt(2); mot = np.array([m0, m0])
    for dist in (30, 40, 50):                       # stations along the +x+y arm, measured from the motor
        c = mot + u * dist
        walls[f'vertical_at_{dist}mm'] = round(line_len(fr, (c[0], c[1], 0), (0, 0, 1), 20), 3)
    nrm = np.array([-u[1], u[0]])
    for dist in (30, 40, 50):
        c = mot + u * dist; walls[f'lateral_at_{dist}mm_mid_depth'] = round(line_len(fr, (c[0], c[1], 3 - p['arm_d_tip'] / 2 - 1), (nrm[0], nrm[1], 0), 20), 3)
    details['arm_wall_probes_mm'] = walls
    ok = all(abs(v - 2 * t) <= .1 for k, v in walls.items() if k.startswith('vertical')) and all(v >= 2 * t - .1 for k, v in walls.items() if k.startswith('lateral'))
    chk('Frame/arm_wall_thickness', 'pass' if ok else 'fail', walls, f'total along a probe = 2 x {t} (vertical), >= 2 x {t} (lateral)', 'the vertical probe crosses top wall + roof wall; lateral crosses two side walls')
    rr = (p['guard_od'] / 2 - p['guard_wall'] / 2) * 1.0
    ang = math.radians(28); gp = (math.cos(ang) * (p['guard_od'] / 2 - 1), math.sin(ang) * (p['guard_od'] / 2 - 1), (p['guard_ring_bottom_z'] + p['guard_top_z']) / 2)
    gw = line_len(shapes['Guard_1_PETG'].translate((-motor_positions(p)[0][0], -motor_positions(p)[0][1], 0)).rotate((0, 0, 0), (0, 0, 1), -motor_positions(p)[0][2]), gp, (math.cos(ang), math.sin(ang), 0), 6)
    chk('Guard/ring_wall', 'pass' if gw >= p['min_wall'] - .01 else 'fail', round(gw, 3), f'>= {p["min_wall"]}', 'radial probe between spokes at band mid-height')
    members = dict(spoke_w=p['guard_spoke_w'], rib_t=p['guard_rib_t'], ring_wall=p['guard_wall'], arm_wall=t, body_w=p['body_w'], cheek_t=p['cheek_t'], plate_t=p['plate_size'][2])
    chk('min_wall_nominal_members', 'pass' if min(members.values()) >= p['min_wall'] - 1e-9 else 'fail', members, f'>= {p["min_wall"]}', 'nominal parameters, not a medial-axis measurement; guard lip (1.0 mm) is an outward add-on to the 1.68 mm ring')
    # ---- supports ----
    sup = {r['name']: r['support_area_mm2'] for r in rows}; details['support_area_mm2'] = sup
    brd = {r['name']: r['flat_bridge_area_mm2'] for r in rows}; details['flat_bridge_area_mm2'] = brd
    chk('overhang_report', 'pass', dict(sloped_overhang_mm2=sup, flat_bridges_mm2=brd), note='sloped = faces leaning past 45 deg from vertical; flat bridges = horizontal downward faces. Frame bridges are the cavity roofs, at most ~5.6 mm wide (12 mm root box: 8.64 mm inner width less two 1.5 mm chamfers); the outer roofs are 45 deg chamfers. Informational: the slicer decides.')
    # ---- guard ring: section stiffness vs v2 rev B and v3 rev D, overhang thresholds ----
    import parts as _gp
    ro_ = p['guard_od'] / 2
    revB = _gp.ring_section_props([(ro_ - 2.4, 19.1), (ro_, 19.1), (ro_, 28.0 - 1.4 * math.tan(math.radians(50))), (ro_ + 1.4, 28.0), (ro_ - 2.4, 28.0)])
    revD = _gp.ring_section_props(_gp.ring_profile(ro_, 1.68, 20.1, 26.1, 1.0, 0.0, 63.4))
    revE = _gp.ring_section_props(_gp.ring_profile(ro_, p['guard_wall'], p['guard_ring_bottom_z'], p['guard_top_z'], p['guard_lip'], p['guard_ring_land'], p['guard_ring_bevel_deg'], p['guard_bead_w'], p['guard_bead_h']))
    rs = dict(rev_B_v2={k: round(float(v), 1) for k, v in revB.items()}, rev_D_v3_printed_and_broke={k: round(float(v), 1) for k, v in revD.items()}, rev_E={k: round(float(v), 1) for k, v in revE.items()},
              E_over_B={k: round(float(revE[k] / revB[k]), 2) for k in revE}, E_over_D={k: round(float(revE[k] / revD[k]), 2) for k in revE},
              note='A = area mm2, Iv = second moment for vertical bending (about the tangential axis), Ir = for radial bending (about the vertical axis), mm4; beam-section model of the ring cross-section, same E; midspan stiffness between spokes scales with I')
    details['guard_ring_section'] = rs
    chk('Guard/ring_radial_stiffness_vs_revB', 'pass' if rs['E_over_B']['Ir'] >= 1.0 else 'fail', rs['E_over_B'], 'radial Ir >= 1.0 x v2 rev B (the guards that survived support removal)', 'vertical Iv ratio is lower because the band is 8.0 mm vs 8.9 mm; reported, not gated')
    gov = {}
    for gname, (gx, gy, ga) in zip(GUARD_NAMES, motor_positions(p)):
        gm = meshes[gname]
        gov[gname] = dict(all=overhang_by_threshold(gm), ring_zone_r_ge_41=overhang_by_threshold(gm, (gx, gy), 41.0))
    details['guard_overhang_mm2_by_threshold_deg'] = gov
    conc = {}
    for gname, (gx, gy, ga) in zip(GUARD_NAMES, motor_positions(p)):
        V_ = meshes[gname].vertices; rr = np.hypot(V_[:, 0] - gx, V_[:, 1] - gy); zz = V_[:, 2]
        loc = np.degrees(np.arctan2(V_[:, 1] - gy, V_[:, 0] - gx)) - ga          # angle in the guard's own frame (opening toward 180)
        sel = (zz > 20.0) & (zz < 25.0) & (rr > 42.5) & (rr < 44.6)                    # mid band, below the inner bead
        inner = [];
        for a0 in range(-105, 106, 15):
            s_ = sel & (np.abs(((loc - a0 + 180) % 360) - 180) < 10)
            if s_.any(): inner.append(round(float(rr[s_].min()), 3))
        bead = (zz > 25.0) & (rr > 40) & (rr < 44.6)
        conc[gname] = dict(inner_radius_mid_band_min=min(v for v in inner if v), inner_radius_mid_band_max=max(v for v in inner if v),
                           bead_inner_radius_min=round(float(rr[bead].min()), 3), prop_tip_gap_min=round(float(rr[bead].min()) - p['prop_d'] / 2, 3))
    details['guard_ring_concentricity'] = conc
    c1 = conc['Guard_1_PETG']; spread = max(max(v['inner_radius_mid_band_max'] - v['inner_radius_mid_band_min'] for v in conc.values()), 0)
    chk('Guard/ring_concentric_with_pad', 'pass' if spread < 0.1 and min(v['prop_tip_gap_min'] for v in conc.values()) >= 4.0 else 'fail', conc, 'inner radius spread < 0.1 mm over 15 deg sectors, prop-tip gap >= 4 mm in every sector', 'measured on the assembly meshes about each motor centre; measuring about the bounding-box centre of a C-shaped print mesh gives a false 1 mm offset')
    g1 = gov['Guard_1_PETG']
    chk('Guard/overhang_report', 'pass', g1, 'informational: area a slicer threshold of T degrees would support; ring_zone = faces at r >= 41 mm from the motor (ring, lip, ribs)', 'the ring floats 16 mm above the pad plane between the spokes, so the first layers of every arc are over air: some support is unavoidable there (see PRINT_SHEET.md)')
    # ---- plan area under props ----
    ext = 130.0; res = .25; n = int(2 * ext / res); yy, xx = np.mgrid[0:n, 0:n]; X = xx * res - ext; Y = ext - yy * res
    disc = np.zeros((n, n), bool)
    for sx in (-1, 1):
        for sy in (-1, 1): disc |= (X - sx * m_) ** 2 + (Y - sy * m_) ** 2 <= (p['prop_d'] / 2) ** 2
    rf = raster(meshes['Frame_PETG']); rg = np.zeros_like(rf)
    for gname in GUARD_NAMES: rg |= raster(meshes[gname])
    plan = dict(frame_mm2=round(float((rf & disc).sum() * res ** 2)), guards_mm2=round(float((rg & disc).sum() * res ** 2)), combined_mm2=round(float(((rf | rg) & disc).sum() * res ** 2)),
                disc_area_mm2=round(float(disc.sum() * res ** 2)))
    base = json.loads((OUT / 'reports/baseline_stock.json').read_text())
    plan['stock'] = dict(frame_mm2=base['main_frame']['area_under_props_mm2'], guards_mm2=base['guards_x4']['area_under_props_mm2'], combined_mm2=base['frame_plus_guards_area_under_props_mm2'])
    plan['change_combined_pct'] = round(100 * (plan['combined_mm2'] / plan['stock']['combined_mm2'] - 1), 1)
    details['projected_area_under_props'] = plan
    chk('projected_area_under_props', 'pass' if plan['combined_mm2'] < plan['stock']['combined_mm2'] else 'fail', plan, 'less than stock 4504 mm2 (frame + guards)')
    # ---- assembly interference ----
    names = list(shapes); coll = []
    skip = {('BatteryPad_TPU', 'BottomPlate_PETG')}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if (a, b) in skip or (b, a) in skip: continue
            v = common_volume(shapes[a], shapes[b])
            if v > 1e-4: coll.append(dict(a=a, b=b, volume_mm3=round(v, 3)))
    for a in names:
        for e in env:
            if e.startswith(('Prop', 'Shaft')): continue
            if a in ('BatteryPad_TPU',) and e == 'Battery': continue
            v = common_volume(shapes[a], env[e])
            if v > 1e-4 and not (e.startswith('Motor') and a.startswith(('Guard', 'Frame'))) and not (e.startswith('Standoff') and a in ('BottomPlate_PETG', 'Frame_PETG')) and not (e.startswith('Isolator') and a == 'Frame_PETG'):
                coll.append(dict(a=a, b=e, volume_mm3=round(v, 3)))
    chk('assembly_collisions', 'pass' if not coll else 'fail', coll, 'no unintended common volume > 1e-4 mm3', 'motor/pad mating faces, standoff/isolator seats excluded')
    # clearances worth reporting
    gaps = {}
    for a, e in [('Frame_PETG', 'Battery'), ('Frame_PETG', 'FC'), ('Frame_PETG', 'Standoff_1'), ('CameraCradle_TPU', 'Standoff_1'), ('BottomPlate_PETG', 'Battery')]:
        gaps[f'{a} to {e}'] = round(exact_distance(shapes[a], env[e]), 2)
    details['key_clearances_mm'] = gaps
    chk('frame_to_battery_clearance', 'pass' if gaps['Frame_PETG to Battery'] >= 0.3 else 'fail', gaps['Frame_PETG to Battery'], '>= 0.3 mm', 'tight; depends on the UNKNOWN standoff length (placeholder). More standoff length gives room.')
    # ---- camera ----
    rigid = ['Frame_PETG', 'BottomPlate_PETG'] + GUARD_NAMES + FOOT_NAMES
    worst = {}; sweep = {}
    for tilt_a in range(30, 76, 3):
        fr_ = view_frustum(p, tilt_a)
        for n_ in rigid + [f'Prop_{k}' for k in range(1, 5)]:
            s = shapes.get(n_) or env[n_]
            v = common_volume(fr_, s)
            if v > 1e-3: worst[n_] = max(worst.get(n_, 0), v)
        cm = tilt(cl, p, tilt_a)
        for n_ in rigid + ['Battery', 'FC'] + [f'Standoff_{k}' for k in range(1, 5)] + [f'Isolator_{k}' for k in range(1, 5)]:
            s = shapes.get(n_) or env[n_]; v = common_volume(cm, s)
            if v > 1e-3: sweep[f'{n_}@{tilt_a}'] = round(v, 3)
    chk('camera_fov_clear_30_75', 'pass' if not worst else 'fail', {k: round(v, 2) for k, v in worst.items()}, 'no rigid part or prop inside the 91 x 82 deg frustum, 3 deg steps, 240 mm', 'FOV from main/at_detect.c calibration; lens position placeholder')
    chk('camera_cradle_swing_clear', 'pass' if not sweep else 'fail', sweep, 'no overlap while tilting 30-75 deg (3 deg steps)')
    # ---- guard spoke roots: section slice, bell keep-out, front C-ends vs camera cradle ----
    gl, _, _ = parts3.guard(p, 0)
    spoke_area = []
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN
    def section_area(shape, r, step=.25):
        # point-in-solid sampling: a B-rep slab intersection returned 0 mm2 at r 13.5-14.6 on this filleted solid (OCC boolean artefact)
        n = 0
        for y in np.arange(-3.5, 3.5, step) + step / 2:
            for z in np.arange(2.8, 10, step) + step / 2:
                if BRepClass3d_SolidClassifier(shape.wrapped, gp_Pnt(r, y, z), 1e-6).State() == TopAbs_IN: n += 1
        return round(n * step * step, 1)
    for r in (10.07, 11.07, 12.07, 13.07, 14.07, 15.07, 16.07): spoke_area.append(section_area(gl, r))
    nominal = 11.4   # measured spoke body section
    details['guard_root_section_area_mm2_r10_to_16'] = spoke_area
    import parts as _v2parts
    details['guard_root_fillets'] = [list(x) for x in _v2parts.FILLET_LOG]
    chk('Guard/spoke_root_section', 'pass' if min(spoke_area) >= nominal else 'fail', spoke_area, f'>= ~{nominal:.1f} mm2 (80% of w x h) from r 10 to 16 mm', 'rev D (filleted); rev C (sharp) was 15-17 mm2, the original whisker 1.4 mm2 at r 11-13')
    _, sp0, _ = parts3.guard(p, 0)
    above = sp0.intersect(box((120, 120, 60), (0, 0, 6 + 30)))
    bell0 = cylinder(p['motor_bell_d'] / 2, p['motor_h'], (0, 0, 6))
    dbell = exact_distance(above, bell0)
    chk('Guard/bell_to_spoke_above_pad', 'pass' if dbell >= p['bell_spoke_clearance'] - 1e-6 else 'fail', round(dbell, 3), f'>= {p["bell_spoke_clearance"]} mm', 'spoke material above the pad top (z > 6); the pad-level foot is coplanar with the pad top like the pad itself')
    front = [n for n, (x, y, a) in zip(GUARD_NAMES, motor_positions(p)) if x > 0]
    gaps_c = {}
    for tilt_a in range(30, 76, 3):
        cm = tilt(cl, p, tilt_a)
        for n_ in front: gaps_c[(n_, tilt_a)] = exact_distance(cm, shapes[n_])
    (kn, kt) = min(gaps_c, key=gaps_c.get)
    details['cradle_to_front_guard_min_gap_mm'] = dict(value=round(gaps_c[(kn, kt)], 3), guard=kn, tilt_deg=kt, per_tilt={str(t): round(min(v for (n_, t2), v in gaps_c.items() if t2 == t), 2) for t in range(30, 76, 3)})
    chk('camera_cradle_to_front_guard_gap', 'pass' if gaps_c[(kn, kt)] >= 3.0 else 'fail', details['cradle_to_front_guard_min_gap_mm'], '>= 3 mm (my threshold, not from the brief)', 'exact B-rep distance, cradle at 30-75 deg in 3 deg steps vs both front guards (C ends, spokes, ring)')
    # ---- MTF-02P flow cover: optical cone clearance, landed heights, clearances ----
    fh = p['flow_size'][2]; z_l = p['plate_bottom_z'] - fh; r_l = p['flow_lens_d'] / 2; ground = p['plate_bottom_z'] - p['foot_h']
    pts_all = {}
    for n_ in ['FlowCover_TPU', 'BottomPlate_PETG', 'CameraCradle_TPU'] + FOOT_NAMES + ['BatteryPad_TPU']:
        m_ = meshes[n_]; samp = m_.sample(30000) if n_ in ('FlowCover_TPU', 'BottomPlate_PETG') else m_.sample(6000)
        P = np.vstack([m_.vertices, samp]); pts_all[n_] = P[P[:, 2] < z_l - 1e-6]
    def min_angle(lx, ly):
        out = {}
        for n_, P in pts_all.items():
            if len(P) == 0: out[n_] = None; continue
            h = z_l - P[:, 2]; dd = np.hypot(P[:, 0] - lx, P[:, 1] - ly)
            ang = np.where(dd <= r_l, 0.0, np.degrees(np.arctan2(dd - r_l, h)))
            out[n_] = round(float(ang.min()), 1)
        return out
    nominal = min_angle(p['flow_x'] + p['flow_lens_offset'][0], p['flow_lens_offset'][1])
    rg = p['flow_lens_offset_range']; worst = {}
    for dx in (-rg, 0, rg):
        for dy in (-rg, 0, rg):
            for n_, a_ in min_angle(p['flow_x'] + p['flow_lens_offset'][0] + dx, p['flow_lens_offset'][1] + dy).items():
                if a_ is not None: worst[n_] = min(worst.get(n_, 999), a_)
    need = p['flow_fov'] / 2 + p['flow_cone_margin_deg']
    nom_min = min(v for v in nominal.values() if v is not None); worst_min = min(worst.values())
    details['flow_cone_min_off_axis_deg'] = dict(required_half_angle=need, nominal_by_part=nominal, worst_case_lens_pm=rg, worst_case_by_part=worst, lens_plane_z=z_l, aperture_d_placeholder=p['flow_lens_d'])
    chk('flow_cone_clearance_nominal', 'pass' if nom_min > need else 'fail', nominal, f'> {need:.0f} deg half-angle (21 deg + 10 deg margin) from the aperture edge', 'lens centre and aperture D are PLACEHOLDERS; points below the lens plane only; vertices + 30k surface samples')
    chk('flow_cone_clearance_lens_pm3mm', 'pass' if worst_min > need else 'fail', worst, f'> {need:.0f} deg for any lens position within +-{rg} mm of the board centre (placeholder range)')
    mb = float(meshes['FlowCover_TPU'].bounds[0, 2])
    details['flow_heights_mm'] = dict(lens_to_ground_landed=round(z_l - ground, 2), skirt_bottom_above_ground=round(mb - ground, 2), lens_recess_behind_skirt=round(z_l - mb, 2), flow_working_distance_min_mm=80)
    chk('flow_skirt_above_feet_ground_plane', 'pass' if mb - ground >= 3.0 else 'fail', round(mb - ground, 2), '>= 3 mm above the feet ground plane (z %.2f)' % ground)
    chk('flow_lens_recessed_behind_skirt', 'pass' if z_l - mb >= 2.0 else 'fail', round(z_l - mb, 2), '>= 2.0 mm')
    fcl = {'sensor to BottomPlate': exact_distance(env['MTF02P'], shapes['BottomPlate_PETG']), 'sensor to BatteryPad': exact_distance(env['MTF02P'], shapes['BatteryPad_TPU']),
           'sensor to FlowCover': exact_distance(env['MTF02P'], shapes['FlowCover_TPU']), 'FlowCover to BatteryPad': exact_distance(shapes['FlowCover_TPU'], shapes['BatteryPad_TPU']),
           'FlowCover to Battery': exact_distance(shapes['FlowCover_TPU'], env['Battery'])}
    details['flow_clearances_mm'] = {k: round(v, 3) for k, v in fcl.items()}
    chk('flow_sensor_to_plate_contact', 'pass' if fcl['sensor to BottomPlate'] < 0.01 else 'fail', round(fcl['sensor to BottomPlate'], 3), 'board top sits on the plate underside (0 mm by design)')
    chk('flow_sensor_to_battery_pad', 'pass' if fcl['sensor to BatteryPad'] >= 1.5 else 'fail', round(fcl['sensor to BatteryPad'], 3), '>= 1.5 mm (the plate is between)')
    chk('flow_cover_to_battery', 'pass' if fcl['FlowCover to Battery'] >= .3 else 'fail', round(fcl['FlowCover to Battery'], 3), '>= 0.3 mm (snap hooks stand in pad pockets)')
    cm_ = shapes['FlowCover_TPU'].Volume() * p['tpu_density'] / 1000
    chk('flow_cover_mass', 'pass' if cm_ <= 2.0 else 'fail', round(cm_, 3), '<= 2.0 g')
    # ---- mass / CG / inertia ----
    parts_m = []
    for n, s in shapes.items():
        mm = meshes[n]; dens = (p['petg_density'] if n.endswith('PETG') else p['tpu_density'])
        mm2 = mm.copy(); mm2.density = dens / 1000.0
        parts_m.append((n, mm2.mass, mm2.center_mass, mm2.moment_inertia))   # g, mm, g mm^2
    hw = []
    mz = p['frame_t'] + p['guard_pad_t']
    for i, (x, y, a) in enumerate(motor_positions(p), 1):
        hw.append((f'Motor_{i}', p['motor_mass'], np.array([x, y, mz + p['motor_h'] / 2]), np.diag([p['motor_mass'] * (3 * (p['motor_bell_d'] / 2) ** 2 + p['motor_h'] ** 2) / 12] * 2 + [p['motor_mass'] * (p['motor_bell_d'] / 2) ** 2 / 2])))
        pz = mz + p['motor_h'] + p['prop_plane_above_bell']
        hw.append((f'Prop_{i}', p['prop_mass'], np.array([x, y, pz]), np.diag([p['prop_mass'] * (p['prop_d'] / 2) ** 2 / 4] * 2 + [p['prop_mass'] * (p['prop_d'] / 2) ** 2 / 2])))
    bz = p['plate_bottom_z'] + p['plate_size'][2] + p['battery_pad_t']
    hw.append(('Battery', p['battery_mass'], np.array([0, 0, bz + 11]), box_inertia(p['battery_mass'], *p['batt_size'])))
    hw.append(('FC', p['fc_mass'], np.array([0, 0, TOP_Z + p['fc_isolator_h'] + p['fc_size'][2] / 2]), box_inertia(p['fc_mass'], *p['fc_size'])))
    cpos = np.array(p['camera_pivot'], float)
    hw.append(('Camera', p['camera_mass'], cpos, box_inertia(p['camera_mass'], *p['camera_size'])))
    hw.append(('MTF02P', p['flow_mass'], np.array([p['flow_x'], 0, p['plate_bottom_z'] - p['flow_size'][2] / 2]), box_inertia(p['flow_mass'], *p['flow_size'])))
    hw.append(('ELRS', p['elrs_mass'], np.array([-42.5, 0, p['plate_bottom_z'] + p['plate_size'][2] + 2.2]), box_inertia(p['elrs_mass'], *p['elrs_size'])))
    hw.append(('Misc', p['misc_mass'], np.array([0., 0, 0]), box_inertia(p['misc_mass'], 30, 30, 10)))
    allp = [(n, m, np.array(c), np.array(I)) for n, m, c, I in parts_m] + hw
    M = sum(m for _, m, _, _ in allp); cg = sum(m * c for _, m, c, _ in allp) / M
    Jm = np.zeros((3, 3))
    for _, m, c, I in allp:
        d = c - cg; Jm += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    Jm = Jm * 1e-3 * 1e-6          # g mm^2 -> kg m^2
    T = np.diag([1, 1, -1.0]); Jf = T @ Jm @ T.T; cgf = (T @ cg)
    printed = sum(m for n, m, _, _ in parts_m)
    petg_set = sum(m for n, m, _, _ in parts_m if n != 'BatteryPad_TPU' and n != 'CameraCradle_TPU')
    tw = 4 * p['Ct'] * p['omega_max'] ** 2 / (M / 1000 * p['gravity'])
    partmass = {n: round(m, 2) for n, m, _, _ in parts_m}
    coup = {k: round(s.Volume() * (1.27 if k.endswith('PETG') else 1.27) / 1000, 2) for k, (s, _) in coupons.items()}
    mass = dict(parts_g=partmass, printed_total_g=round(printed, 2), printed_set_comparable_to_stock_g=round(printed - partmass['BatteryPad_TPU'] - partmass['CameraCradle_TPU'], 2),
                structure_without_tpu_pad_cradle_g=round(petg_set, 2), all_up_g=round(M, 2), cg_flight_mm=[round(float(v), 2) for v in cgf], J_diag_kg_m2=[float(Jf[i, i]) for i in range(3)],
                J_sim_kg_m2=[3.92e-4, 4.05e-4, 6.39e-4], thrust_to_weight=round(float(tw), 3), hardware_g=round(M - printed, 2),
                stock_printed_set_g=68.5, stock_plus_printed_set_g=79.9, fit_coupon_masses_g=coup,
                method='Printed parts: exact tessellation volume x density (zero-infill walls would be lower; solid CAD volume is what the slicer would fill with walls only if every region is wall-thin; real prints also differ by seams and flow). Hardware: placeholders from the brief, boxes/discs about their own centres; misc 20 g as a 30x30x10 box at the origin.')
    details['mass'] = mass
    chk('mass_target_55g_printed_set', 'pass' if mass['printed_set_comparable_to_stock_g'] <= 55 else 'fail', mass['printed_set_comparable_to_stock_g'], '<= 55 g (frame + guards + plate + feet; excludes TPU battery pad and camera cradle)',
        'reported honestly; see TOPOLOGY_DECISION.md for why this is not reached')
    chk('mass_stretch_45g', 'fail' if mass['printed_set_comparable_to_stock_g'] > 45 else 'pass', mass['printed_set_comparable_to_stock_g'], '<= 45 g', 'stretch target flagged not credible')
    chk('lighter_than_stock', 'pass' if mass['printed_set_comparable_to_stock_g'] < 68.5 else 'fail', mass['printed_set_comparable_to_stock_g'], '< 68.5 g stock')
    # ---- tip-over about the front feet ----
    zg = p['plate_bottom_z'] - p['foot_h']; xp = p['foot_x'] + p['foot_od'] / 2
    tip = {}
    for n, mm in meshes.items():
        if n.startswith('Foot') or mm is None: continue
        v = mm.vertices - np.array([xp, 0, zg])
        # nose-down rotation about the y axis through the front-foot edge: point (x,z) -> (x cos t + z sin t..)
        best = None
        for deg in np.arange(0, 90, .25):
            th_ = math.radians(deg); z_new = -v[:, 0] * math.sin(th_) + v[:, 2] * math.cos(th_)
            if z_new.min() <= 0: best = float(deg); break
        tip[n] = best
    details['nose_down_strike_deg'] = {k: v for k, v in tip.items() if v is not None}
    first = min(tip, key=lambda k: 999 if tip[k] is None else tip[k])
    chk('nose_down_first_contact_is_TPU', 'pass' if first == 'CameraCradle_TPU' or first.endswith('TPU') else 'fail', {first: tip[first]}, 'first ground contact on a TPU part', note='geometric tilt about the front foot edge only')
    # ---- FEM ----
    fem = json.loads((OUT / 'reports/fem_final.json').read_text()) if (OUT / 'reports/fem_final.json').exists() else None
    if fem:
        chk('beam_model_stiffness_criterion_X', 'pass' if fem['criterion_X_met'] else 'fail', fem['final_no_cross_link']['ratio_vs_stock'], 'vertical >= 0.75, torsion >= 0.75, lateral >= 0.25 of stock', 'concept beam model, not FEA; E and density are placeholders')
    # ---- indeterminate ----
    for nm, note in [
        ('safe_motor_thread_engagement', 'M2x10 through 3 + 3 mm pads; safe motor thread depth UNKNOWN; do not use longer screws.'),
        ('standoff_length', 'UNKNOWN. v3 assumes 25 mm between the plate top (z -30) and the body boss bottoms (z -5); the arm roots clear the battery by only ~0.4 mm, so a longer standoff is preferable.'),
        ('fc_isolator_and_screw_length', 'FC sits ABOVE the frame on 5 mm isolators (v2 convention); stock_plus describes it hanging below. Verify which side, and the screw engagement in the 8 mm bosses.'),
        ('arm_depth_cap_9mm', 'Depth capped at the stock 9 mm envelope; the true limit is FC and battery clearance, unknown.'),
        ('guard_band_height', 'v3 band is 6.0 mm (v2 8.9) centred on the ASSUMED prop plane (z 23.1); verify the real prop height.'),
        ('flow_sensor_lens_connector_holes', 'VERIFIED: 21.6 x 16 x 6.5 mm and 42 deg FOV (MicoAir, Rotor Village, Rotorama pages). NOT verifiable: lens position and aperture, connector side/pinout, mounting holes (page images only). The cover assumes a centred D8 lens, a rear cable notch and no holes; it grips the board (no screws). Measure the real board and re-run.'),
        ('flow_cover_fit', 'FlowCover snap tabs (0.6 mm hooks, 0.21 mm slot clearance each side) and the 0.3 mm board clearance are untested; print FitCoupon_FlowCover_TPU against FitCoupon_FlowSlot_PETG first.'),
        ('ELRS_fit', '20 x 12 x 4 mm placeholder pocket on the rear tab.'),
        ('strap_fit', '12 x 2 mm placeholder strap.'),
        ('material_density', 'PETG 1.27 and TPU 1.21 g/cm3 placeholders; weigh the first print.'),
        ('physical_mass_CG', 'Hardware masses are placeholders; weigh the build.'),
        ('physical_strength', 'No FEA, fatigue, crash or layer-adhesion test. Print the FitCoupons, break-test the arm box and the guard spoke first.'),
        ('arm_closed_box_printing', 'Closed hollow arms with 45 deg chamfered roofs; slicer must keep the walls continuous. Check the thin-wall preview; hole-to-wall ligaments near the screw seats are unmeasured.')]:
        chk(nm, 'indeterminate', note=note)
    result = {'schema': 1, 'axes': 'CAD x forward, y right, z up (motor stack +z); flight report x forward, y right, z down', 'checks': checks, 'details': details,
              'summary': {s: sum(c['status'] == s for c in checks) for s in ['pass', 'fail', 'indeterminate']}}
    (OUT / 'reports/validation.json').write_text(json.dumps(result, indent=1, default=float))
    write_reports(result, p)
    log('Validation: ' + json.dumps(result['summary']))


TOP_Z = 3.0


def stop_dist(p): return p['arm_tip_stop']


def write_reports(result, p):
    d = result['details']; ms = d['mass']; fails = [c for c in result['checks'] if c['status'] == 'fail']; unk = [c for c in result['checks'] if c['status'] == 'indeterminate']
    t = '# Engineering validation - v3 (tapered closed hollow-box arms)\n\n**Prototype; not flight-qualified.** Geometry checks do not establish crash survivability.\n\n'
    t += f"Automated checks: **{result['summary']['pass']} pass, {len(fails)} fail, {len(unk)} indeterminate**. See `validation.json` for every number.\n\n## Mass, CG, T/W\n\n"
    t += f"- Printed set comparable to stock (frame + 4 guards + plate + 4 feet): **{ms['printed_set_comparable_to_stock_g']} g** (stock 68.5 g, stock_plus 79.9 g). Including the TPU battery pad and camera cradle: {ms['printed_total_g']} g.\n"
    t += f"- Part masses (g): {json.dumps(ms['parts_g'])}\n"
    t += f"- All-up (with the brief's placeholder hardware, {ms['hardware_g']} g): **{ms['all_up_g']} g**. Thrust-to-weight **{ms['thrust_to_weight']}** (4 Ct wmax^2 / m g; stock 5.57, stock_plus 5.30).\n"
    t += f"- CG (x forward, y right, z down): **{ms['cg_flight_mm']} mm**. Jxx/Jyy/Jzz about CG: **{' / '.join(f'{v:.4g}' for v in ms['J_diag_kg_m2'])} kg m^2** (simulation {' / '.join(f'{v:.3g}' for v in ms['J_sim_kg_m2'])}).\n"
    t += f"- {ms['method']}\n\n## Failures\n\n"
    if not fails: t += 'None.\n'
    for c in fails: t += f"- **{c['name']}**: {c.get('value')}; required {c.get('limit')}. {c.get('note', '')}\n"
    t += '\n## Indeterminate / physical verification\n\n'
    for c in unk: t += f"- **{c['name']}**: {c['note']}\n"
    t += '\n## Methods\n\n- B-rep validity and hole positions read from the exact B-rep cylinder faces (tolerance 0.05 mm on centre and diameter).\n- Wall checks are line probes plus nominal parameters, not a medial-axis analysis. Support area = downward-facing area beyond 45 deg in the suggested print orientation; the slicer decides.\n- Camera frusta at 3 deg steps, 30-75 deg, 91 x 82 deg, 240 mm.\n- Stiffness comes from a concept beam model with placeholder E; ratios only.\n'
    (OUT / 'reports/VALIDATION.md').write_text(t, encoding='utf-8')
    s = '# Print sheet - v3\n\n0.4 mm nozzle, 0.2 mm layers; wall counts are multiples of the 0.42 mm line (4 perimeters = 1.68 mm). Starting settings only; masses are solid CAD volume at placeholder densities (zero-infill thin-wall parts, so infill setting barely matters).\n\n| Part | Material | Print box mm | Volume cm3 | Mass g | Overhang >45 deg mm2 | Orientation | Settings |\n|---|---|---|---:|---:|---:|---|---|\n'
    for r in d['parts']:
        n = r['name']
        o = 'Skirt down, hooks up, as exported' if n.startswith('FlowCover') else 'Flat motor face on the bed (flipped from CAD)' if n.startswith('Frame') else 'Pad down' if n.startswith('Guard') else 'Back on the bed' if n.startswith('Camera') else 'Open end up' if n.startswith('Foot') else 'Flat'
        st = 'TPU 95A, 0.2 mm layers, slow (20-25 mm/s), 4 perimeters (1.68 mm walls), 0% infill, fan 100%, no supports; 45 deg lip and hook faces; print FitCoupon_FlowCover_TPU first' if n.startswith('FlowCover') else '4 perimeters, 0-10% gyroid, 245 C, fan 40%; arm roofs are 45 deg chamfers (no supports expected)' if n.startswith('Frame') else '4-6 perimeters, ring and spokes are all wall; 245 C, fan 30-40%' if n.startswith('Guard') else '4 perimeters, 15% gyroid' if n.startswith(('Bottom')) else 'TPU 95A slow, 3 perimeters, 20% gyroid' if r['material'] == 'TPU' else ''
        s += f"| {n} | {r['material']} | {' x '.join(str(v) for v in r['print_bbox_mm'])} | {r['volume_mm3'] / 1000:.3f} | {r['mass_g']:.2f} | {r['support_area_mm2']} | {o} | {st} |\n"
    s += '\nCoupons (print before the full set): `FitCoupon_Frame_PLA` (motor pad + FC hole pattern), `FitCoupon_Guard_PLA`, `FitCoupon_GuardSpoke_PETG` (break-test the spoke and ring joint by hand), `FitCoupon_ArmBox_PETG` (40 mm closed-box arm slice: crush/bend it to see wall bonding), `FitCoupon_FlowCover_TPU` (one wall of the flow cover with lip and snap tab) with `FitCoupon_FlowSlot_PETG` (matching plate slots): the tab must click in and out by hand.\n'
    g1 = d['guard_overhang_mm2_by_threshold_deg']['Guard_1_PETG']; rs = d['guard_ring_section']
    s += f"""
## Guard (rev E) - read before printing

**Print the ring-segment coupon first** (`FitCoupon_GuardSpoke_PETG`: pad, one spoke, a 90 degree arc of ring with the lip). Break it by hand: twist and pull the ring; compare with a v2 rev B guard if you still have one.

Why rev E: the rev D ring (1.68 mm wall, 6 mm band) broke when its supports were removed. Rev E: 6-line wall 2.52 mm, 8.0 mm band (z 19.1-27.1), outward lip 1.4 mm (as v2 rev B), 0.8 x 1.5 mm inner bead, spoke joint reaching 1.8 mm into the ring, rib foot chamfered at 50 degrees. Ring section vs v2 rev B: radial bending {rs['E_over_B']['Ir']}x, vertical bending {rs['E_over_B']['Iv']}x, area {rs['E_over_B']['A']}x; vs the broken rev D: radial {rs['E_over_D']['Ir']}x, vertical {rs['E_over_D']['Iv']}x.

Slicer: PETG, 0.2 mm layers, nozzle 245-250 C, bed 75 C, part fan 30-40 percent (higher fan makes the ring layers peel), walls **6** (the ring and spokes are all wall; infill does nothing), 100 percent top/bottom not needed, line width 0.42-0.45 mm, outer wall slow (about 40 mm/s), brim 3-5 mm on the pad. Print pad-down, as exported.

**Supports (the honest answer):** the ring floats about 16 mm above the pad plane between the spokes, so the first layers of every arc are printed over air. Every ring, lip, bead and rib-foot face is 50 degrees or steeper from horizontal and the ring apex is a sharp edge (no flat land), so a threshold of 45 degrees or lower puts supports on nothing but the first layers under that apex line, and a threshold of 50 or more would load supports onto the 50 degree faces. Overhang area on one guard, by slicer threshold angle (T = 30 / 40 / 45 / 50 / 55 degrees; all guard, then ring zone r >= 41 mm only), mm2: {g1['all']} and {g1['ring_zone_r_ge_41']}. Faces between 45 and 50 degrees are the spoke keels on the tilted spokes (self-supporting by the 45 degree rule). **Unavoidable:** between the spokes (and under the two C-ends) the ring floats 16 mm above the pad, so the first layers of the apex are printed over air whatever the face angles; some support must sit under the apex line there, nowhere else.

Bambu Studio: Support ON, type **tree (slim)** or normal (grid), threshold angle **30-40**, **on build plate only**, Top Z distance **0.25-0.3 mm** (or 'Thick bridges' off), support/object XY distance 0.5 mm, support interface 2 layers rectilinear, pattern spacing wide (3-4 mm) so the supports are few and light, **no supports on the spokes**: check the preview and paint-out (support blocker) anything touching the spokes or the pad. In the preview, supports must appear only as thin columns under the ring land between the spokes and under the two C-ends.
Removing supports: cut or snap each column near the bed first, then peel the interface layer sideways along the ring; do not pull the ring away from the support radially. Let the guard cool fully first. If the coupon ring snaps while you remove supports, stop and increase the wall or band before printing four guards.
"""
    (OUT / 'reports/PRINT_SHEET.md').write_text(s, encoding='utf-8')


if __name__ == '__main__':
    try: main()
    except BaseException:
        import traceback
        (OUT / 'reports/error.txt').write_text(traceback.format_exc()); raise
