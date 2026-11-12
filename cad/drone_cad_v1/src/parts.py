"""Part generators. Construction ratios control shape, parameters control interfaces."""
import math
import cadquery as cq
from geometry import *

def arm(p,kind='fork'):
    b=p['body_bolt_half']; m=p['motor_side']/2; d=p['arm_splay']; sag=p['arm_lower_sag']
    root=p['root_chord_d']/2; tip=p['tip_chord_d']/2
    top=[(b,b,p['deck_bottom_z']),(b,b,16),(b+6,b+6,12),(m-18,m-18,10),(m-14,m-14,2.5),(m-11,m-11,2.5)]
    upper_edge=cq.Edge.makeBezier([V(*v) for v in top])
    pieces=[tube(top,root,tip,bezier=True),
            tube([(m-14,m-14,2.5),(m,m,2.5)],tip+.2)]
    sides=[0] if kind=='arch' else [-1,1]
    for s in sides:
        low=[(b+s*d,b-s*d,2.5),(b+3+s*d,b+3-s*d,-sag),
             (b+12+s*d,b+12-s*d,-sag-2),(m-17+s*d/2,m-17-s*d/2,-sag),
             (m-14,m-14,2.5),(m-11,m-11,2.5)]
        pieces.append(tube(low,root*.85,tip,bezier=True))
        if kind!='arch':
            le=cq.Edge.makeBezier([V(*v) for v in low])
            a=le.positionAt(.42); end=upper_edge.positionAt(.73)
            ctrl=[a,a+le.tangentAt(.42)*4,end-upper_edge.tangentAt(.73)*4,end]
            pieces.append(tube([v.toTuple() for v in ctrl],p['web_d']/2+.3,p['web_d']/2,bezier=True))
    if kind=='cage':
        for s in [-1,1]:
            pieces.append(tube([(b+s*d,b-s*d,20),(b+12+s*4,b+12-s*4,11),(m-1,m-1,2.5)],tip))
    for piece in pieces:
        if not piece.isValid() or piece.Volume()<=1: raise ValueError('Invalid or empty arm sweep')
    result=union(pieces)
    if len(result.Solids())!=1 or result.Volume()<500: raise ValueError(f'Arm union lost its continuous load path: {len(result.Solids())} solids, volume {result.Volume()}, inputs {[q.Volume() for q in pieces]}')
    return result

def frame(p):
    b=p['body_bolt_half']; dz=p['deck_bottom_z']; dt=p['deck_t']
    deck=rounded_box((p['deck_outer'],p['deck_outer'],dt),(0,0,dz+dt/2),4)
    deck=deck.cut(rounded_box((p['deck_opening'],p['deck_opening'],dt+2),(0,0,dz+dt/2),2))
    parts=[deck]
    # Curved perimeter at the lower frame, with open central electronics bay.
    for angle in [0,90,180,270]:
        parts.append(placed(tube([(-b,b,2.5),(0,b+1.5,2.5),(b,b,2.5)],p['root_chord_d']/2),angle,(0,0,0)))
    sector=arm(p)
    # Curves end tangent to a horizontal neck outside the guard pad: the motor
    # seating region has no upper/lower chord bulge and needs no clipping cuts.
    for angle in [0,90,180,270]: parts.append(placed(sector,angle,(0,0,0)))
    for x,y,a in motor_positions(p):
        pad=cylinder(p['motor_pad_d']/2,p['frame_t'])
        pad=holes_motor(pad,p,-1,p['frame_t']+2)
        # Pins stay inside the guard pad and below the motor seating plane.
        for sign in [-1,1]:
            pad=pad.fuse(cylinder(p['key_pin_d']/2,p['key_pin_h'],(0,sign*p['key_offset'],p['frame_t'])))
        parts.append(pad.translate((x,y,0)))
    for x in [-b,b]:
        for y in [-b,b]:
            parts.append(cylinder(p['body_leg_d']/2,dz+dt,(x,y,0)))
    f=p['fc_hole_spacing']/2
    for x in [-f,f]:
        for y in [-f,f]:
            parts.append(cylinder(4,p['frame_t'],(x,y,0)))
            parts.append(tube([(x,y,2.5),(x*1.35,y*1.35,2.5),(math.copysign(b,x),math.copysign(b,y),2.5)],2.5))
    # Thin adjacent-arm ties resist yaw/torsion without obstructing the FC sides.
    t=p['motor_side']*.34
    for angle in [0,90,180,270]:
        parts.append(placed(tube([(-t,t,-2),(0,t+2,-2),(t,t,-2)],p['tie_d']/2),angle,(0,0,0)))
    # Integrated camera cheeks: two separated double-supported forks.
    cp=p['camera_pivot']; cy=p['camera_size'][1]/2+p['camera_cradle_wall']+2.2
    for sign in [-1,1]:
        yy=sign*cy
        cheek=union([tube([(b,sign*b,dz),(cp[0]-6,yy,cp[2]-6),(cp[0],yy,cp[2])],3.4,3),
                     tube([(b,sign*b,12),(cp[0]-5,yy,cp[2]-9),(cp[0],yy,cp[2])],2.7),
                     cylinder(7.5,3.2,(cp[0],yy-1.6,cp[2]),(0,1,0))])
        cheek=cheek.intersect(box((100,100,100),(cp[0],sign*(cy-1.6+50),cp[2])))
        # 15 degree radial face detents: TPU teeth engage after light axial clamping.
        inner=sign*(cy-1.6)
        for a in range(0,360,15):
            groove=box((2.4,.5,1.0),(6,sign*.2,0)).rotate((0,0,0),(0,1,0),a).translate((cp[0],inner,cp[2]))
            cheek=cheek.cut(groove)
        parts.append(cheek)
    # Open rear receiver pocket with positive lips and tie/antenna exits.
    ex,ey,ez=p['elrs_size']; wall=2; rear=-(p['deck_outer']+ex)/2
    pocket=rounded_box((ex+2*wall,ey+2*wall,2.2),(rear,0,dz+dt+.9),1)
    pocket=union([pocket,box((2,ey+4,ez),(rear-(ex+2)/2,0,dz+dt+2+ez/2)),
                  box((ex+4,2,ez),(rear,(ey+2)/2,dz+dt+2+ez/2)),
                  box((ex+4,2,ez),(rear,-(ey+2)/2,dz+dt+2+ez/2))])
    pocket=pocket.cut(box((4,5,ez+4),(rear-(ex+2)/2,0,dz+dt+3)))
    for yy in [-ey/2+2.4,ey/2-2.4]: pocket=pocket.cut(box((6,1.6,2.6),(rear,yy,dz+dt+.9)))
    parts.append(pocket)
    result=union(parts)
    cutters=[]
    for x in [-b,b]:
        for y in [-b,b]:
            cutters.append(cylinder(p['body_bolt_d']/2,40,(x,y,-1)))
            # Clear the standoff beneath the curved lower perimeter.
            cutters.append(cylinder(2.65,4,(x,y,-4)))
    for x in [-f,f]:
        for y in [-f,f]: cutters.append(cylinder(p['fc_hole_d']/2,8,(x,y,-1)))
    cutters.append(cylinder(p['camera_pivot_d']/2,60,(cp[0],-30,cp[2]),(0,1,0)))
    for x,y,a in motor_positions(p):
        cutters.extend(tool.translate((x,y,0)) for tool in motor_tools(p,-1,p['frame_t']+2))
    return result.cut(*cutters,tol=1e-5).fix()

def guard(p,angle=0):
    # Local +x points outboard. C opening faces -x (body).
    ro=p['guard_od']/2; ri=ro-p['guard_wall']; half=p['guard_half_sweep_deg']; z0=p['guard_ring_bottom_z']; z1=p['guard_top_z']
    def polar(r,a,z=0): return (r*math.cos(math.radians(a)),r*math.sin(math.radians(a)),z)
    outer=cq.Edge.makeCircle(ro,(0,0,z0),(0,0,1),-half,half)
    inner=cq.Edge.makeCircle(ri,(0,0,z0),(0,0,1),-half,half)
    lines=[cq.Edge.makeLine(V(*polar(ro,-half,z0)),V(*polar(ri,-half,z0))),cq.Edge.makeLine(V(*polar(ro,half,z0)),V(*polar(ri,half,z0)))]
    wire=cq.Wire.assembleEdges([outer,inner]+lines)
    ring=cq.Solid.extrudeLinear(wire,[],V(0,0,z1-z0))
    # Round upper/lower ring edges; the rounded edge still leaves >1.6 mm wall.
    ring=cq.Workplane(obj=ring).edges().fillet(.45).val()
    pad=cylinder(p['guard_pad_d']/2,p['guard_pad_t'],(0,0,p['frame_t']))
    pad=holes_motor(pad,p,p['frame_t']-1,p['guard_pad_t']+2)
    for s in [-1,1]: pad=pad.cut(cylinder(p['key_hole_d']/2,4,(0,s*p['key_offset'],p['frame_t']-1)))
    branches=[]; r=p['guard_spoke_d']/2
    for a in [-82,0,82]:
        root=polar(11.9,a,4.5); fork=polar(29,a,13.5)
        branches.append(tube([root,polar(15.5,a,6.8),polar(21,a,10),fork],r))
        for sign in [-1,1]:
            end=a+sign*25
            branches.append(tube([fork,polar(39,a+sign*15,16),polar(44.5,end,18),polar((ri+ro)/2,end,z0+1)],r))
    spokes=union(branches).intersect(box((120,120,60),(0,0,35)))
    spokes=placed(spokes,angle,(0,0,0)); ring=placed(ring,angle,(0,0,0))
    return union([pad,spokes,ring]),spokes,ring

def bottom_plate(p):
    sx,sy,th=p['plate_size']; z=p['plate_bottom_z']; b=p['body_bolt_half']
    plate=rounded_box((sx,sy,th),(0,0,z+th/2),3)
    # Strap through slots along each battery side; plenty of edge ligament.
    for yy in [-p['batt_size'][1]/2-3,p['batt_size'][1]/2+3]:
        plate=plate.cut(rounded_box((p['strap_w']+1,p['strap_t']+.6,th+2),(0,yy,z+th/2),.4))
    # Four short battery end stops leave lateral insertion and strap access.
    for xx in [-p['batt_size'][0]/2-1.2,p['batt_size'][0]/2+1.2]:
        for yy in [-10,10]: plate=plate.fuse(rounded_box((2.4,6,4),(xx,yy,z+th+2),.8))
    for x in [-b,b]:
        for y in [-b,b]: plate=plate.cut(cylinder(p['body_bolt_d']/2,th+2,(x,y,z-1)))
    # Sensor tape nest on underside, open lens face, two retention strap slots.
    fx,fy,fh=p['flow_size']; xx=p['flow_x']; wall=2
    for yy in [-(fy/2+wall/2+.4),fy/2+wall/2+.4]:
        plate=plate.fuse(box((fx+2*wall,wall,2.5),(xx,yy,z-1.25)))
    for xx2 in [xx-fx/2-wall/2-.4,xx+fx/2+wall/2+.4]:
        plate=plate.fuse(box((wall,fy+4.8,2.5),(xx2,0,z-1.25)))
    for yy in [-fy/2-3.1,fy/2+3.1]: plate=plate.cut(box((6,1.8,8),(xx,yy,z)))
    return plate.clean()

def battery_pad(p):
    x,y,_=p['batt_size']; t=p['battery_pad_t']; z=p['plate_bottom_z']+p['plate_size'][2]
    return rounded_box((x,y,t),(0,0,z+t/2),2)

def foot(p):
    z=p['plate_bottom_z']; h=p['foot_h']; r=p['foot_od']/2; wall=p['foot_wall']
    s=cylinder(r,h,(0,0,z-h))
    s=s.cut(cylinder(r-wall,h-wall,(0,0,z-h-.01)))
    s=s.cut(cylinder(p['body_bolt_d']/2,h+2,(0,0,z-h-1)))
    return s.clean()

def cradle(p):
    dx,dy,dz=p['camera_size']; w=p['camera_cradle_wall']
    back=box((w,dy+2*w,dz+2*w),(-dx/2-w/2,0,0))
    parts=[back,box((dx+w,dy+2*w,w),(-w/2,0,-dz/2-w/2))]
    for s in [-1,1]:
        yy=s*(dy/2+w/2)
        parts.append(box((dx+w,w,dz+2*w),(-w/2,yy,0)))
        parts.append(cylinder(7.5,w,(0,yy-w/2,0),(0,1,0)))
        for a in range(0,360,15):
            tooth=box((2,.55,.8),(6,s*(dy/2+w+.275),0)).rotate((0,0,0),(0,1,0),a)
            parts.append(tooth)
    result=union(parts)
    result=result.cut(cylinder(1.25,dy+2*w+4,(0,-dy/2-w-2,0),(0,1,0)))
    # Cable/heatsink window and two separate zip-tie slots retain the board.
    result=result.cut(box((w+2,dy-8,dz-8),(-dx/2-w/2,0,0)))
    for zz in [-dz/2+1.5,dz/2-1.5]: result=result.cut(box((w+2,8,1.8),(-dx/2-w/2,0,zz)))
    return result.clean()

def tilt_wedge(p,angle):
    # Removable keyed spacer on M3 pivot; flats index against cradle back and cheek.
    # This is an angle witness/assembly jig, not the flight load-bearing clamp.
    r=9; thick=1.6
    points=[(0,0),(r,0),(r*math.cos(math.radians(angle)),r*math.sin(math.radians(angle)))]
    s=cq.Workplane('XZ').polyline(points).close().extrude(thick).val()
    return s.cut(cylinder(1.65,5,(0,-3,0),(0,1,0)))

def envelopes(p):
    out={}; mz=p['frame_t']+p['guard_pad_t']; pp=mz+p['motor_h']+p['prop_plane_above_bell']
    for i,(x,y,a) in enumerate(motor_positions(p),1):
        out[f'Motor_{i}']=cylinder(p['motor_bell_d']/2,p['motor_h'],(x,y,mz))
        out[f'Shaft_{i}']=cylinder(p['motor_shaft_d']/2,p['motor_shaft_h'],(x,y,mz+p['motor_h']))
        out[f'Prop_{i}']=cylinder(p['prop_d']/2,p['prop_envelope_t'],(x,y,pp-p['prop_envelope_t']/2))
    z=p['plate_bottom_z']+p['plate_size'][2]+p['battery_pad_t']
    out['Battery']=box(p['batt_size'],(0,0,z+p['batt_size'][2]/2))
    out['FC']=box(p['fc_size'],(0,0,p['frame_t']+p['fc_isolator_h']+p['fc_size'][2]/2))
    b=p['body_bolt_half']; dz=p['deck_bottom_z']+p['deck_t']; ex,ey,ez=p['elrs_size']
    out['ELRS']=box(p['elrs_size'],(-(p['deck_outer']+ex)/2,0,dz+2+ez/2))
    out['Camera']=tilt(box(p['camera_size'],(0,0,0)),p)
    out['MTF02P']=box(p['flow_size'],(p['flow_x'],0,p['plate_bottom_z']-.6-p['flow_size'][2]/2))
    # Nonprinted hardware interfaces are explicit solid standoff/isolator envelopes.
    for i,(x,y) in enumerate([(x,y) for x in [-b,b] for y in [-b,b]],1):
        zz=p['plate_bottom_z']+p['plate_size'][2]
        out[f'Standoff_{i}']=cylinder(2.5,-zz,(x,y,zz)).cut(cylinder(p['body_bolt_d']/2,-zz,(x,y,zz)))
    f=p['fc_hole_spacing']/2
    for i,(x,y) in enumerate([(x,y) for x in [-f,f] for y in [-f,f]],1):
        out[f'Isolator_{i}']=cylinder(2.5,p['fc_isolator_h'],(x,y,p['frame_t'])).cut(cylinder(p['fc_hole_d']/2,p['fc_isolator_h'],(x,y,p['frame_t'])))
    return out

def fit_coupons(p):
    """Optional PLA interface tests; not included in aircraft mass or assembly."""
    motor=cylinder(p['motor_pad_d']/2,p['frame_t'],(-25,0,0))
    bridge=rounded_box((46,8,p['frame_t']),(-4,0,p['frame_t']/2),1)
    fc=rounded_box((34,34,p['frame_t']),(16,0,p['frame_t']/2),2)
    base=union([motor,bridge,fc])
    base=holes_motor(base.translate((25,0,0)),p,-1,p['frame_t']+2).translate((-25,0,0))
    for yy in [-p['key_offset'],p['key_offset']]: base=base.fuse(cylinder(p['key_pin_d']/2,p['key_pin_h'],(-25,yy,p['frame_t'])))
    f=p['fc_hole_spacing']/2
    for xx,yy in [(16+x,y) for x in [-f,f] for y in [-f,f]]: base=base.cut(cylinder(p['fc_hole_d']/2,8,(xx,yy,-1)))
    base=base.cut(rounded_box((p['strap_w']+1,p['strap_t']+.6,8),(16,0,2.5),.4))
    gp=holes_motor(cylinder(p['guard_pad_d']/2,p['guard_pad_t']),p,-1,4)
    for yy in [-p['key_offset'],p['key_offset']]: gp=gp.cut(cylinder(p['key_hole_d']/2,4,(0,yy,-1)))
    return {'FitCoupon_Frame_PLA':base.clean(),'FitCoupon_Guard_PLA':gp.clean()}
