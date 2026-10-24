"""v3 parts: tapered closed hollow-box (hexagonal-section) arms, light guards, light plate.
Axes: x forward, y right, z up; motor stack on +z. Frame top face (z = 3) carries guard and motor; the
arm/body boxes hang BELOW it (z < 3), so the FC sits above the frame on its isolators as in v2."""
import math
import cadquery as cq
from geometry import *
import parts as v2

TOP = 3.0   # frame top face (= pad top)


def hex_section(w, d, c, y0=0.0):
    """(y, z) outline, top flat at z=TOP, vertical sides, 45 deg chamfers on the roof corners."""
    return [(-w / 2, TOP), (w / 2, TOP), (w / 2, TOP - d + c), (w / 2 - c, TOP - d), (-w / 2 + c, TOP - d), (-w / 2, TOP - d + c)]


def chamfer(w, d): return max(0.8, min(2.5, d * .35, w / 2 - 1.2))


def hexbox(p0, p1, w0, d0, w1, d1, t, inner_trim=0.0):
    """Closed tapered hollow box from p0 to p1 (xy). Returns (outer, inner). inner_trim shortens the
    cavity at both ends (so a member end can stay closed)."""
    p0 = np.array(p0, float); p1 = np.array(p1, float)
    L = np.linalg.norm(p1 - p0); ang = math.degrees(math.atan2(*(p1 - p0)[::-1]))
    def wire(x, w, d, inner):
        if inner:
            c = chamfer(w, d); ci = max(0.4, c - .586 * t)
            wi = w / 2 - t; zt = TOP - t; zb = TOP - d + t
            pts = [(-wi, zt), (wi, zt), (wi, zb + ci), (wi - ci, zb), (-wi + ci, zb), (-wi, zb + ci)]
        else:
            pts = hex_section(w, d, chamfer(w, d))
        return cq.Wire.makePolygon([V(x, y, z) for y, z in pts + [pts[0]]])
    def solid(inner):
        a, b = (inner_trim, L - inner_trim) if inner else (0, L)
        f = lambda x: (w0 + (w1 - w0) * x / L, d0 + (d1 - d0) * x / L)
        wa, da = f(a); wb, db = f(b)
        return cq.Solid.makeLoft([wire(a, wa, da, inner), wire(b, wb, db, inner)], True)
    place = lambda s: s.rotate((0, 0, 0), (0, 0, 1), ang).translate((p0[0], p0[1], 0))
    return place(solid(False)), place(solid(True))


import numpy as np


def node_points(p):
    f = p['fc_hole_spacing'] / 2 * math.sqrt(2)          # 18.031: FC holes on a 25.5 square rotated 45 deg
    fc = [(f, 0), (0, f), (-f, 0), (0, -f)]
    bm = [tuple(map(float, q)) for q in p['bottom_mount_points']]
    return fc, bm


def frame(p):
    t = p['arm_wall']; bw = p['body_w']; bd = p['body_d']; stop = p['arm_tip_stop']; m = p['motor_side'] / 2
    fc, bm = node_points(p)
    outers = []; inners = []
    def member(a, b, w0, d0, w1, d1):
        o, i = hexbox(a, b, w0, d0, w1, d1, t, inner_trim=0.0); outers.append(o); inners.append(i)
    for i in range(4): member(fc[i], fc[(i + 1) % 4], bw, bd, bw, bd)
    for q in bm:
        nearest = min(fc, key=lambda f: (f[0] - q[0]) ** 2 + (f[1] - q[1]) ** 2)
        member(nearest, q, bw, bd, bw, bd)
    roots = []
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        mot = np.array([sx * m, sy * m]); root = np.array([sx * fc[0][0] / 2, sy * fc[0][0] / 2])
        u = (root - mot) / np.linalg.norm(root - mot); tip = mot + u * stop
        member(root, tip, p['arm_w_root'], p['arm_d_root'], p['arm_w_tip'], p['arm_d_tip'])
    body = union(outers)
    cav = union(inners)
    # bosses: solid tubes at FC and bottom-mount nodes, full body depth
    bosses = []
    for (x, y), hd in [(q, p['fc_hole_d']) for q in fc] + [(q, p.get('bottom_frame_hole_d', 2.8)) for q in bm]:
        bosses.append(cylinder(p['boss_od'] / 2, bd, (x, y, TOP - bd)))
    pads = [cylinder(p['motor_pad_d'] / 2, p['frame_t'], (sx * m, sy * m, TOP - p['frame_t'])) for sx in (-1, 1) for sy in (-1, 1)]
    # cavity is cut from the outer shell, then solid bosses and pads are added back
    shell = body.cut(cav, tol=1e-5)
    result = union([shell] + bosses + pads)
    cutters = []
    for (x, y) in fc: cutters.append(cylinder(p['fc_hole_d'] / 2, 20, (x, y, TOP - 12)))
    for (x, y) in bm: cutters.append(cylinder(p.get('bottom_frame_hole_d', 2.8) / 2, 20, (x, y, TOP - 12)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            cutters.extend(tool.translate((sx * m, sy * m, 0)) for tool in motor_tools(p, TOP - p['frame_t'] - 1, p['frame_t'] + 2))
    return result.cut(*cutters, tol=1e-5).clean().fix()


def guard(p, angle=0):
    # v2 rev B generator with the v3 numbers from parameters.json (lighter ring/spokes, 6 mm band).
    # Spoke path points are shifted with the band so the prop clearance stays what v2 had.
    return v2.guard(p, angle)


def bottom_plate(p):
    sx, sy, th = p['plate_size']; z = p['plate_bottom_z']
    fcx = p['batt_size'][0] / 2
    pts = [tuple(map(float, q)) for q in p['bottom_mount_points']]
    plate = rounded_box((sx, sy, th), (0, 0, z + th / 2), 3)
    # strap slots on both sides of the battery
    for yy in [-p['batt_size'][1] / 2 - 3, p['batt_size'][1] / 2 + 3]:
        plate = plate.cut(rounded_box((p['strap_w'] + 1, p['strap_t'] + .6, th + 2), (0, yy, z + th / 2), .4))
    # four battery end stops
    for xx in [-fcx - 1.2, fcx + 1.2]:
        for yy in [-10, 10]: plate = plate.fuse(rounded_box((2.4, 6, 4), (xx, yy, z + th + 2), .8))
    # front ears: two bars reaching the front mount holes (+-40, +-12)
    for yy in (-12, 12):
        plate = plate.fuse(rounded_box((17, 8, th), (36.5, yy, z + th / 2), 2.0))
    # rear tab with ELRS pocket (open top, antenna notch at the rear edge)
    ex, ey, ez = p['elrs_size']
    rear = rounded_box((24, 34, th), (-41, 0, z + th / 2), 3)
    plate = plate.fuse(rear)
    pr = -41 - 1.5; w = 1.68
    for yy in [-(ey + w) / 2 - .4, (ey + w) / 2 + .4]:
        plate = plate.fuse(box((ex + 2, w, 3.0), (pr, yy, z + th + 1.5)))
    plate = plate.fuse(box((w, ey + 2 * w + .8, 3.0), (pr + ex / 2 + 1 + w / 2, 0, z + th + 1.5)))   # forward lip toward the battery
    # lightening windows: flow sensor window; battery windows (the pack rides on the TPU pad)
    fx, fy, fh = p['flow_size']; xx = p['flow_x']; rec = p['flow_recess']
    if rec > 0: plate = plate.cut(box((fx + .8, fy + .8, rec + .01), (xx, 0, z + rec / 2 - .005)))
    plate = plate.cut(rounded_box((fx - 4, fy - 4, th + 2), (xx, 0, z + th / 2), 1.5))
    for yy in [-fy / 2 - 3.1, fy / 2 + 3.1]: plate = plate.cut(box((6, 1.8, 8), (xx, yy, z)))
    plate = plate.cut(rounded_box((25, 22, th + 2), (17.5, 0, z + th / 2), 3))    # front battery window
    plate = plate.cut(rounded_box((8, 22, th + 2), (-25.0, 0, z + th / 2), 3))  # rear battery window (ELRS side)
    plate = cheeks(p, plate)
    for (x, y) in pts: plate = plate.cut(cylinder(p['bottom_panel_hole_d'] / 2, th + 2, (x, y, z - 1)))
    return plate.clean()


def cheeks(p, plate):
    th = p['plate_size'][2]; zb = p['plate_bottom_z']; cp = p['camera_pivot']
    cy = p['camera_size'][1] / 2 + p['camera_cradle_wall'] + 2.2; ct = p['cheek_t']; r = 7.5
    x0 = p['batt_size'][0] / 2 + .5; x1 = cp[0] + r
    xa = x1 - (cp[2] - zb) / math.tan(math.radians(50))
    web = (cq.Workplane('XZ').moveTo(x0, zb).lineTo(xa, zb).lineTo(x1, cp[2])
           .threePointArc((cp[0], cp[2] + r), (cp[0] - r, cp[2])).lineTo(x0, zb + th + 4).close().extrude(ct).val())
    # lightening window, leaving >= 4 mm ligaments around the pivot boss and edges
    win = (cq.Workplane('XZ').polyline([(x0 + 4, zb + th + 4), (xa - 7, zb + th + 4), (cp[0] - 11, cp[2] - 3), (x0 + 4, zb + th + 11)]).close().extrude(ct + 2).val()
           .translate((0, -1, 0)))
    web = web.cut(win)
    bb = web.BoundingBox(); parts = [plate]
    for sign in [-1, 1]:
        y0 = cy - 1.6 if sign > 0 else -(cy + 1.6) + (3.2 - ct)
        parts.append(web.translate((0, y0 - bb.ymin, 0)))
        parts.append(rounded_box((xa - 2 - x0, 6.0, th), ((x0 + xa - 2) / 2, sign * (cy - 1.6 + 1.5 + ct / 2 + 0.0), zb + th / 2), 2))
    plate = union(parts)
    for sign in [-1, 1]:
        inner = sign * (cy - 1.6)
        for a in range(0, 360, 15):
            plate = plate.cut(box((2.4, .5, 1.0), (6, sign * .2, 0)).rotate((0, 0, 0), (0, 1, 0), a).translate((cp[0], inner, cp[2])))
    return plate.cut(cylinder(p['camera_pivot_d'] / 2, 60, (cp[0], -30, cp[2]), (0, 1, 0)))


def battery_pad(p): return v2.battery_pad(p)


def foot(p):
    z = p['plate_bottom_z']; h = p['foot_h']; r = p['foot_od'] / 2; wall = p['foot_wall']
    s = cylinder(r, h, (0, 0, z - h))
    s = s.cut(cylinder(r - wall, h - wall, (0, 0, z - h - .01)))
    s = s.cut(cylinder(p['body_bolt_d'] / 2, h + 2, (0, 0, z - h - 1)))
    return s.clean()


def cradle(p): return v2.cradle(p)


def envelopes(p):
    out = {}; mz = p['frame_t'] + p['guard_pad_t']; pp = mz + p['motor_h'] + p['prop_plane_above_bell']
    for i, (x, y, a) in enumerate(motor_positions(p), 1):
        out[f'Motor_{i}'] = cylinder(p['motor_bell_d'] / 2, p['motor_h'], (x, y, mz))
        out[f'Shaft_{i}'] = cylinder(p['motor_shaft_d'] / 2, p['motor_shaft_h'], (x, y, mz + p['motor_h']))
        out[f'Prop_{i}'] = cylinder(p['prop_d'] / 2, p['prop_envelope_t'], (x, y, pp - p['prop_envelope_t'] / 2))
    z = p['plate_bottom_z'] + p['plate_size'][2] + p['battery_pad_t']
    out['Battery'] = box(p['batt_size'], (0, 0, z + p['batt_size'][2] / 2))
    out['FC'] = box(p['fc_size'], (0, 0, TOP + p['fc_isolator_h'] + p['fc_size'][2] / 2))
    ex, ey, ez = p['elrs_size']
    out['ELRS'] = box(p['elrs_size'], (-42.5, 0, p['plate_bottom_z'] + p['plate_size'][2] + .2 + ez / 2))
    out['Camera'] = tilt(box(p['camera_size'], (0, 0, 0)), p)
    out['MTF02P'] = box(p['flow_size'], (p['flow_x'], 0, p['plate_bottom_z'] + p['flow_recess'] - p['flow_size'][2] / 2))
    zt = p['plate_bottom_z'] + p['plate_size'][2]
    for i, (x, y) in enumerate([tuple(map(float, q)) for q in p['bottom_mount_points']], 1):
        L = p['standoff_top_z'] - zt
        out[f'Standoff_{i}'] = cylinder(2.5, L, (x, y, zt)).cut(cylinder(p['bottom_panel_hole_d'] / 2, L, (x, y, zt)))
    fc, _ = node_points(p)
    for i, (x, y) in enumerate(fc, 1):
        out[f'Isolator_{i}'] = cylinder(2.5, p['fc_isolator_h'], (x, y, TOP)).cut(cylinder(p['fc_hole_d'] / 2, p['fc_isolator_h'], (x, y, TOP)))
    return out


def arm_coupon(p):
    """40 mm slice of the arm (root-size end) with a pad-like tab: break-test the closed hollow box walls."""
    o, i = hexbox((0, 0), (40, 0), p['arm_w_root'] * .75, p['arm_d_root'], p['arm_w_root'] * .5, p['arm_d_root'] - 2, p['arm_wall'])
    return o.cut(i).clean()


def fit_coupons(p):
    f = {}
    motor = cylinder(p['motor_pad_d'] / 2, p['frame_t'], (-25, 0, 0))
    bridge = rounded_box((46, 8, p['frame_t']), (-4, 0, p['frame_t'] / 2), 1)
    fc = rounded_box((34, 34, p['frame_t']), (16, 0, p['frame_t'] / 2), 2)
    base = union([motor, bridge, fc])
    base = holes_motor(base.translate((25, 0, 0)), p, -1, p['frame_t'] + 2).translate((-25, 0, 0))
    a = p['fc_hole_spacing'] / 2 * math.sqrt(2)
    for xx, yy in [(16 + a, 0), (16, a), (16 - a, 0), (16, -a)]: base = base.cut(cylinder(p['fc_hole_d'] / 2, 8, (xx, yy, -1)))
    f['FitCoupon_Frame_PLA'] = base.clean()
    f['FitCoupon_Guard_PLA'] = holes_motor(cylinder(p['guard_pad_d'] / 2, p['guard_pad_t']), p, -1, 4).clean()
    f['FitCoupon_GuardSpoke_PETG'] = v2.guard_spoke_coupon(p)
    f['FitCoupon_ArmBox_PETG'] = arm_coupon(p)
    return f
