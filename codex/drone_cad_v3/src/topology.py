"""v3 step (b): three topology candidates vs a calibrated stock-arm model.
Concept-level beam (3D frame FEM) + plan-area model. NOT a crash simulation. All numbers are ratios/estimates:
E, G, densities and every candidate section are assumptions (flagged in TOPOLOGY_DECISION.md).
Coordinates here: stock axes (x right, y forward, z up); mass/stiffness are invariant to the Astra axis swap.
"""
import numpy as np
from PIL import Image, ImageDraw

RHO = 1.27e-3            # g/mm3 PETG, PLACEHOLDER
E = 1800.0; G = E / 2.7  # MPa, PLACEHOLDER (flat-printed PETG); only ratios are used
M = 56.5; PROP_R = 38.1; PAD_R = 10.0; ARM_LEN_PAD = 10.0
LINE = 0.42


# ---------- section properties (w horizontal across member, d vertical) ----------
def rect(w, d):
    a, b = max(w, d), min(w, d)
    J = a * b ** 3 * (1 / 3 - 0.21 * (b / a) * (1 - b ** 4 / (12 * a ** 4)))
    return dict(A=w * d, Iv=w * d ** 3 / 12, Il=d * w ** 3 / 12, J=J)


def box(w, d, t):
    t = min(t, w / 2 - .01, d / 2 - .01)
    A = w * d - (w - 2 * t) * (d - 2 * t)
    Iv = w * d ** 3 / 12 - (w - 2 * t) * (d - 2 * t) ** 3 / 12
    Il = d * w ** 3 / 12 - (d - 2 * t) * (w - 2 * t) ** 3 / 12
    Am = (w - t) * (d - t); per = 2 * ((w - t) + (d - t))
    return dict(A=A, Iv=Iv, Il=Il, J=4 * Am * Am * t / per)


def twin(wr, d, sp):     # two vertical ribs sp apart, no skins (open section; ties ignored -> conservative)
    r = rect(wr, d)
    return dict(A=2 * r['A'], Iv=2 * r['Iv'], Il=2 * (r['Il'] + r['A'] * (sp / 2) ** 2), J=2 * r['J'])


# ---------- 3D frame FEM ----------
def kbeam(L, s):
    A, Iv, Il, J = s['A'], s['Iv'], s['Il'], s['J']   # local y bends in z (vertical): Iv ; local z bends in plan: Il
    k = np.zeros((12, 12)); EA = E * A / L; GJ = G * J / L
    k[0, 0] = k[6, 6] = EA; k[0, 6] = k[6, 0] = -EA
    k[3, 3] = k[9, 9] = GJ; k[3, 9] = k[9, 3] = -GJ
    for (v, th, I, sg) in ((1, 5, Il, 1), (2, 4, Iv, -1)):
        a = 12 * E * I / L ** 3; b = 6 * E * I / L ** 2; c = 4 * E * I / L; d = 2 * E * I / L
        idx = [v, th, v + 6, th + 6]
        m = np.array([[a, sg * b, -a, sg * b], [sg * b, c, -sg * b, d], [-a, -sg * b, a, -sg * b], [sg * b, d, -sg * b, c]])
        for i in range(4):
            for j in range(4):
                k[idx[i], idx[j]] += m[i, j]
    return k


def rot(p, q):
    e1 = np.array(q, float) - np.array(p, float); L = np.linalg.norm(e1); e1 = e1 / L
    e2 = np.cross([0, 0, 1.], e1)
    if np.linalg.norm(e2) < 1e-9: e2 = np.array([0, 1., 0])
    e2 /= np.linalg.norm(e2); e3 = np.cross(e1, e2)
    R = np.array([e1, e2, e3]); T = np.zeros((12, 12))
    for i in range(4): T[3 * i:3 * i + 3, 3 * i:3 * i + 3] = R
    return T, L


class Frame:
    def __init__(s): s.nodes = {}; s.el = []; s.fixed = set(); s.mass = 0.0; s.members = []

    def node(s, xy):
        key = (round(float(xy[0]), 1), round(float(xy[1]), 1))
        if key not in s.nodes: s.nodes[key] = len(s.nodes)
        return s.nodes[key]

    def add(s, p, q, sec, w, n=1, tag=''):
        """member p->q in n pieces; sec / w may be functions of t in [0,1] along p->q (taper)."""
        p = np.array(p, float); q = np.array(q, float)
        for i in range(n):
            t0, t1 = i / n, (i + 1) / n; tm = (t0 + t1) / 2
            a = p * (1 - t0) + q * t0; b = p * (1 - t1) + q * t1
            sc = sec(tm) if callable(sec) else sec
            ia, ib = s.node(a), s.node(b)
            s.el.append((ia, ib, sc, a, b, tag)); s.mass += sc.get('mA', sc['A']) * np.linalg.norm(b - a) * RHO
            s.members.append((a, b, w(tm) if callable(w) else w, tag))

    def fix(s, xy): s.fixed.add(s.node(xy))

    def solve(s, load_node, F):
        return s.solve_multi(load_node, [F])[0]

    def solve_multi(s, load_node, Fs):
        N = len(s.nodes); K = np.zeros((6 * N, 6 * N))
        par = list(range(N))
        def fnd(i):
            while par[i] != i: par[i] = par[par[i]]; i = par[i]
            return i
        for ia, ib, *_ in s.el: par[fnd(ia)] = fnd(ib)
        if len({fnd(i) for i in range(N)}) != 1: raise ValueError('frame model is not connected (floating members)')
        for ia, ib, sc, a, b, _ in s.el:
            T, L = rot(np.r_[a, 0], np.r_[b, 0]); k = T.T @ kbeam(L, sc) @ T
            idx = list(range(6 * ia, 6 * ia + 6)) + list(range(6 * ib, 6 * ib + 6))
            K[np.ix_(idx, idx)] += k
        fx = {6 * i + j for i in s.fixed for j in range(6)}; free = [i for i in range(6 * N) if i not in fx]
        Kf = K[np.ix_(free, free)]; out = []
        B = np.zeros((6 * N, len(Fs)))
        for j, F in enumerate(Fs): B[6 * load_node:6 * load_node + 6, j] = F
        X = np.linalg.solve(Kf, B[free])
        for j in range(len(Fs)):
            u = np.zeros(6 * N); u[free] = X[:, j]; out.append(u)
        return out


RIGID = dict(A=1e3, Iv=1e6, Il=1e6, J=1e6, mA=10.0)   # solid joint region: root block / pad region treated as rigid (its mass is costed with pad/body)
FC = [(18.031, 0), (0, 18.031), (-18.031, 0), (0, -18.031)]
BB = [(12, 40), (-12, 40), (12, -40), (-12, -40)]
ROOT_R = 9.0155   # arm joins the FC diamond at its edge midpoint (9.0155, 9.0155)


def build(kind, P, body):
    """kind A hollow tapered box arm | B Warren lattice arm | C blade arm + X braces | S stock-calibrated lattice.
    body = dict(sec=section, w=plan width)."""
    f = Frame(); tips = []
    for q in FC + BB: f.fix(q)
    bs, bw = body['sec'], body['w']
    for i in range(4): f.add(FC[i], FC[(i + 1) % 4], bs, bw, 2, 'body')
    for b in BB: f.add(b, (0, 18.031 * np.sign(b[1])), bs, bw, 2, 'body')
    if body.get('cross', True): f.add(BB[0], BB[1], bs, bw, 2, 'body'); f.add(BB[2], BB[3], bs, bw, 2, 'body')
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        mot = np.array([sx * M, sy * M]); root = np.array([sx * ROOT_R, sy * ROOT_R])
        u = (root - mot) / np.linalg.norm(root - mot); nrm = np.array([-u[1], u[0]]); tip = mot + u * ARM_LEN_PAD
        tips.append((f.node(mot), np.arctan2(u[1], u[0])))
        # pad stub: the D20 pad is costed separately (identical for all); only its stiffness is modelled
        f.add(mot, tip, rect(10, 3.0 + 0.0), 10, 1, 'padstub')
        dt, dr = P['d_tip'], P['d_root']
        if kind == 'A':
            wt, wr_, t = P['w_tip'], P['w_root'], P['t']
            f.add(tip, root, lambda x: box(wt + (wr_ - wt) * x, dt + (dr - dt) * x, t), lambda x: wt + (wr_ - wt) * x, 6, 'arm')
        elif kind in ('B', 'S'):
            nc = P['cells']; et, er = P['e_tip'], P['e_root']
            e = lambda x: et + (er - et) * x
            dd = lambda x: dt + (dr - dt) * x
            for sgn in (1, -1):
                pos = lambda x, sgn=sgn: tip * (1 - x) + root * x + nrm * sgn * e(x)
                for i in range(nc * 2):
                    t0, t1 = i / (nc * 2), (i + 1) / (nc * 2); tm = (t0 + t1) / 2
                    f.add(pos(t0), pos(t1), rect(P['wc'], dd(tm)), P['wc'], 1, 'chord')
            for sgn in (1, -1):    # root tie: chord end -> body node (rigid-ish gusset)
                f.add(root + nrm * sgn * e(1), root, RIGID, 0.1, 1, 'gusset')
                f.add(tip + nrm * sgn * e(0), tip, RIGID, 0.1, 1, 'gusset')
            for i in range(nc):
                t0, t1 = i / nc, (i + 1) / nc; tm = (t0 + t1) / 2; s1 = 1 if i % 2 == 0 else -1
                pa = tip * (1 - t0) + root * t0 + nrm * s1 * e(t0); pb = tip * (1 - t1) + root * t1 - nrm * s1 * e(t1)
                f.add(pa, pb, rect(P['wd'], dd(tm)), P['wd'], 1, 'diag')
        elif kind == 'C':
            wb = P['wb']
            f.add(tip, root, lambda x: rect(wb, dt + (dr - dt) * x), wb, 6, 'arm')
    if kind == 'C':
        for (a1, a2) in (((1, 1), (1, -1)), ((-1, 1), (-1, -1)), ((1, 1), (-1, 1)), ((1, -1), (-1, -1))):
            for (r1, r2) in ((P['r_far'], P['r_near']), (P['r_near'], P['r_far'])):
                a = np.array(a1) * r1 / np.sqrt(2); b = np.array(a2) * r2 / np.sqrt(2)
                f.add(a, b, rect(P['wx'], P['dx']), P['wx'], 4, 'xbrace')
    return f, tips


def stiff(f, tip, ang):
    u = np.array([np.cos(ang), np.sin(ang)]); n = np.array([-u[1], u[0]])
    r = [x[6 * tip:6 * tip + 6] for x in f.solve_multi(tip, [[0, 0, 1, 0, 0, 0], [n[0], n[1], 0, 0, 0, 0], [u[0], u[1], 0, 0, 0, 0], [0, 0, 0, u[0], u[1], 0]])]
    return dict(k_vert=1 / r[0][2], k_lat=1 / (r[1][:2] @ n), k_axial=1 / (r[2][:2] @ u), k_tors=1 / (r[3][3:5] @ u))


RES = 0.25; EXT = 100.0


def plan_area(f):
    """Plan area (mm2) of members inside the four prop discs, excluding the D20 pads (fixed, same for all)."""
    n = int(2 * EXT / RES); img = Image.new('L', (n, n), 0); d = ImageDraw.Draw(img)
    px = lambda p: ((p[0] + EXT) / RES, (EXT - p[1]) / RES)
    for a, b, w, tag in f.members:
        v = b - a; L = np.linalg.norm(v)
        if L < 1e-6 or tag == 'padstub': continue
        nv = np.array([-v[1], v[0]]) / L * w / 2
        d.polygon([px(a + nv), px(b + nv), px(b - nv), px(a - nv)], fill=1)
    yy, xx = np.mgrid[0:n, 0:n]; X = xx * RES - EXT; Y = EXT - yy * RES
    disc = np.zeros((n, n), bool); pad = np.zeros((n, n), bool)
    for sx in (-1, 1):
        for sy in (-1, 1):
            disc |= (X - sx * M) ** 2 + (Y - sy * M) ** 2 <= PROP_R ** 2
            pad |= (X - sx * M) ** 2 + (Y - sy * M) ** 2 <= PAD_R ** 2
    arr = np.array(img, bool)
    return float((arr & disc & ~pad).sum() * RES ** 2), arr


def breakdown(f):
    out = {}
    for ia, ib, sc, a, b, tag in f.el:
        out[tag] = out.get(tag, 0) + sc.get('mA', sc['A']) * np.linalg.norm(b - a) * RHO
    return {k: round(v, 2) for k, v in out.items()}
