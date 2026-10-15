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
    return shapes[0].fuse(*shapes[1:],tol=1e-5) if len(shapes)>1 else shapes[0]

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

def rounded_box(size, center, radius=1):
    w=cq.Workplane('XY').box(*size).edges('|Z').fillet(radius)
    return w.val().translate(center)

def motor_tools(p,z,h):
    tools=[]
    a=p['motor_hole_spacing']/2
    for x,y in [(a,a),(a,-a),(-a,a),(-a,-a)]:
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
    return shape.rotate((0,0,0),(0,1,0),-(p['camera_tilt'] if angle is None else angle)).translate(p['camera_pivot'])

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
