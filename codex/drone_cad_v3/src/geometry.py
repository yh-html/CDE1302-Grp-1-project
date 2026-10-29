"""Canonical OpenCascade solids. All coordinates mm; local CAD z points up."""
import math
import cadquery as cq
import numpy as np

V = cq.Vector

def box(size, center):
    return cq.Workplane('XY').box(*size).val().translate(center)

def cylinder(r, h, base=(0,0,0), direction=(0,0,1)):
    return cq.Solid.makeCylinder(r,h,V(*base),V(*direction))

def union(shapes):
    shapes=list(shapes)
    # A 10 nm fuzzy tolerance resolves numerically coincident tangent joins;
    # it is five thousand times smaller than the manufacturing mesh tolerance.
    # Do not unify same-domain faces here: OCC 7.8 can invalidate these tangent
    # multi-loft junctions during that optional simplification step.
    if len(shapes)==1: return shapes[0]
    # v2: a fuse that silently drops inputs still returns a "valid" shape, so also require
    # the result to keep the largest input and at least half the summed input volume
    # (overlaps at tube joints are far below 50 %).
    vols=[s.Volume() for s in shapes]
    def sane(r): return r.isValid() and r.Volume()>=.98*max(vols) and r.Volume()>=.5*sum(vols)
    try:
        r=shapes[0].fuse(*shapes[1:],tol=1e-5)
        if sane(r): return r
    except ValueError:
        pass
    # Fallback for near-tangent converging tubes: pairwise fuse with a slightly larger
    # fuzzy tolerance (still <= 1 micron, far below the 0.06 mm mesh tolerance).
    for tol in (1e-5,1e-4,1e-3):
        try:
            r=shapes[0]
            for s in shapes[1:]: r=r.fuse(s,tol=tol)
            if sane(r): return r
        except ValueError:
            continue
    raise ValueError(f'union failed at every tolerance (inputs {[round(v,1) for v in vols]})')

def tube(points, r0, r1=None, tangents=None, bezier=False):
    """Smooth interpolating spine and normal circular loft sections, tapered at roots."""
    pts=[V(*p) for p in points]
    edge=(cq.Edge.makeBezier(pts) if bezier else cq.Edge.makeSpline(pts,tangents=[V(*t) for t in tangents] if tangents else None)) if len(pts)>2 else cq.Edge.makeLine(*pts)
    if r1 is None or abs(r0-r1)<1e-8:
        profile=cq.Wire.makeCircle(r0,pts[0],edge.tangentAt(0))
        return cq.Solid.sweep(profile,[],edge,True,False)
    wires=[]
    for t in np.linspace(0,1,9):
        radius=r0+(r1-r0)*(3*t*t-2*t*t*t)
        wires.append(cq.Wire.makeCircle(radius,edge.positionAt(float(t)),edge.tangentAt(float(t))))
    return cq.Solid.makeLoft(wires)

def tube_oval(points, w, h, n=13):
    """Elliptical-section sweep: w across (horizontal), h up (vertical major axis), smooth spline spine.

    The section x-axis is kept horizontal (tangent x world Z), so the tall axis stays as
    close to vertical as the path allows: stiffer against landing and vertical crash loads
    than a round tube of similar area, and still thin in the prop wash.
    """
    pts=[V(*p) for p in points]
    edge=cq.Edge.makeSpline(pts) if len(pts)>2 else cq.Edge.makeLine(*pts)
    z=V(0,0,1)
    tan=edge.tangentAt(0).normalized(); xd=tan.cross(z)
    xd=V(1,0,0) if xd.Length<1e-6 else xd.normalized()
    profile=cq.Wire.makeEllipse(w/2,h/2,edge.positionAt(0),tan,xd)
    # Default (corrected-Frenet) sweep: cleaner faces than a loft, which broke the booleans
    # where two branches leave the same fork. The spoke paths lie almost in vertical planes,
    # so the profile keeps its tall axis close to vertical. (A fixed-binormal sweep was tried
    # and gave wrong, even negative, volumes in OCC 7.8.)
    return cq.Solid.sweep(profile,[],cq.Wire.assembleEdges([edge]),True,False)

def tube_keel(points, w, h, keel_deg=50):
    """Spline sweep with a 'keel' section: round top (radius w/2), vertical sides, and a V
    underneath whose faces are keel_deg from horizontal; total height h.

    Printed with the section's round top up, every face is at least keel_deg from horizontal
    at any spine slope, so the member needs no support. The section's up axis is kept
    perpendicular to the spine within its vertical plane (spines here are planar, radial).
    """
    pts=[V(*p) for p in points]
    edge=cq.Edge.makeSpline(pts) if len(pts)>2 else cq.Edge.makeLine(*pts)
    o=edge.positionAt(0); tan=edge.tangentAt(0).normalized()
    u=tan.cross(V(0,0,1)); u=V(1,0,0) if u.Length<1e-6 else u.normalized()
    up=u.cross(tan).normalized()
    if up.z<0: up=-up
    r=w/2; vee=r*math.tan(math.radians(keel_deg)); side=h-r-vee
    if side<0: raise ValueError('tube_keel: h too small for the round top and keel')
    P=lambda a,b: o+u*a+up*b
    edges=[cq.Edge.makeThreePointArc(P(r,0),P(0,r),P(-r,0)),
           cq.Edge.makeLine(P(-r,0),P(-r,-side)),cq.Edge.makeLine(P(-r,-side),P(0,-side-vee)),
           cq.Edge.makeLine(P(0,-side-vee),P(r,-side)),cq.Edge.makeLine(P(r,-side),P(r,0))]
    return cq.Solid.sweep(cq.Wire.assembleEdges(edges),[],cq.Wire.assembleEdges([edge]),True,False)

def rounded_box(size, center, radius=1):
    w=cq.Workplane('XY').box(*size).edges('|Z').fillet(radius)
    return w.val().translate(center)

def motor_tools(p,z,h):
    tools=[]
    a=p['motor_hole_spacing']/2
    if p.get('motor_hole_layout','square')=='cardinal':
        points=[(a,0),(0,a),(-a,0),(0,-a)]
    else:
        points=[(a,a),(a,-a),(-a,a),(-a,-a)]
    for x,y in points:
        if p['motor_slot_travel']<=0:   # v3: plain round holes, exactly the stock interface
            tools.append(cylinder(p['motor_slot_d']/2,h,(x,y,z))); continue
        angle=math.degrees(math.atan2(y,x))
        tool=(cq.Workplane('XY').center(x,y)
              .slot2D(p['motor_slot_d']+p['motor_slot_travel'],p['motor_slot_d'],angle)
              .extrude(h).val().translate((0,0,z)))
        tools.append(tool)
    tools.append(cylinder(p['motor_center_hole_d']/2,h,(0,0,z)))
    return tools

def holes_motor(shape,p,z,h):
    return shape.cut(*motor_tools(p,z,h),tol=1e-5).clean()

def placed(shape, angle, center):
    return shape.rotate((0,0,0),(0,0,1),angle).translate(center)

def tilt(shape,p,angle=None):
    # v2 fix: a positive rotation about +y pitches local +x (the lens axis) DOWN:
    # (1,0,0) -> (cos a, 0, -sin a). Astra 01 used -angle, which aimed the camera
    # 30-75 deg UP. laptop/tag_debug.py assumes 45 deg nose-down.
    return shape.rotate((0,0,0),(0,1,0),(p['camera_tilt'] if angle is None else angle)).translate(p['camera_pivot'])

def motor_positions(p):
    a=p['motor_side']/2
    return [(a,a,45),(a,-a,-45),(-a,-a,-135),(-a,a,135)]

def mesh_of(shape,tol=.06,angle=.15):
    import trimesh
    vv,ff=shape.tessellate(tol,angle)
    m=trimesh.Trimesh(vertices=[v.toTuple() for v in vv],faces=ff,process=True)
    m.merge_vertices(digits_vertex=6)
    # OCC may emit zero-area pole triangles. Remove only degenerate/duplicate
    # facets; never fill holes or replace the canonical B-rep with a repaired mesh.
    m.update_faces(m.nondegenerate_faces(height=1e-7))
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    return m
