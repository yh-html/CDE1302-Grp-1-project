"""Part generators. Construction ratios control shape, parameters control interfaces."""
import math
import cadquery as cq
from geometry import *

def arm(p,kind='fork'):
    b=p['body_bolt_half']; m=p['motor_side']/2; d=p['arm_splay']; sag=p['arm_lower_sag']
    root=p['root_chord_d']/2; tip=p['tip_chord_d']/2
    # v2: upper chord raised 2 mm mid-span to keep truss depth now the lower chord no longer sags.
    top=[(b,b,p['deck_bottom_z']),(b,b,16),(b+6,b+6,14),(m-18,m-18,12),(m-14,m-14,2.5),(m-11,m-11,2.5)]
    upper_edge=cq.Edge.makeBezier([V(*v) for v in top])
    pieces=[tube(top,root,tip,bezier=True),
            tube([(m-14,m-14,2.5),(m,m,2.5)],tip+.2)]
    sides=[0] if kind=='arch' else [-1,1]
    # v2: lower chords run along the bed instead of sagging up to 6 mm below the motor
    # pads. The axis sits at 0.6 x chord radius, so the frame's flat cut at z=0 leaves a
    # ~4 mm bed strip, and the chord meets the z=2.5 neck at an angle (a coaxial,
    # tangent overlap with the neck broke the OCC union).
    zl=root*.85*.6
    for s in sides:
        low=[(b+s*d,b-s*d,zl),(b+3+s*d,b+3-s*d,zl-sag),
             (b+12+s*d,b+12-s*d,zl-sag),(m-17+s*d/2,m-17-s*d/2,zl-sag),
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
    tz=p['tie_d']/2  # v2: ties sit on the bed (were 2 mm below it)
    for angle in [0,90,180,270]:
        parts.append(placed(tube([(-t,t,tz),(0,t+2,tz),(t,t,tz)],p['tie_d']/2),angle,(0,0,0)))
    # Integrated camera cheeks: two separated double-supported forks.
    # v2: only for the 'top' mount; the 'under_nose' cheeks belong to the bottom plate.
    cp=p['camera_pivot']; cy=p['camera_size'][1]/2+p['camera_cradle_wall']+2.2
    for sign in ([-1,1] if p['camera_mount']=='top' else []):
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
    # v2: flat underside, so pads, lower chords, ties and perimeter print on the bed
    # and the motor-screw seats are not printed on supports.
    cutters=[box((400,400,40),(0,0,-20))]
    for x in [-b,b]:
        for y in [-b,b]:
            cutters.append(cylinder(p['body_bolt_d']/2,40,(x,y,-1)))
            # Clear the standoff beneath the curved lower perimeter.
            cutters.append(cylinder(2.65,4,(x,y,-4)))
    for x in [-f,f]:
        for y in [-f,f]: cutters.append(cylinder(p['fc_hole_d']/2,8,(x,y,-1)))
    if p['camera_mount']=='top': cutters.append(cylinder(p['camera_pivot_d']/2,60,(cp[0],-30,cp[2]),(0,1,0)))
    for x,y,a in motor_positions(p):
        cutters.extend(tool.translate((x,y,0)) for tool in motor_tools(p,-1,p['frame_t']+2))
    return result.cut(*cutters,tol=1e-5).fix()

def guard(p,angle=0):
    """v2 rev B guard, after printed Y-fork guards broke at the forks and the spoke-to-ring
    joints. Five independent spokes (no forks); each passes under the band and climbs its
    outside face as a full-height rib, so the joint spans the band height instead of butting
    into its free edge. Keel-section spokes, a 50 deg outer top lip and a root collar make the
    whole part printable pad-down with supports only under the band's lower edge."""
    # Local +x points outboard. C opening faces -x (body).
    ro=p['guard_od']/2; ri=ro-p['guard_wall']; half=p['guard_half_sweep_deg']; z0=p['guard_ring_bottom_z']; z1=p['guard_top_z']
    def polar(r,a,z=0): return (r*math.cos(math.radians(a)),r*math.sin(math.radians(a)),z)
    # Band plus an outward lip on the top edge (hoop stiffness; prop-tip gap unchanged),
    # lip underside 50 deg from horizontal so it prints unsupported.
    lip=p['guard_lip']; lz=lip*math.tan(math.radians(50))
    ring=(cq.Workplane('XZ').polyline([(ri,z0),(ro,z0),(ro,z1-lz),(ro+lip,z1),(ri,z1)]).close()
          .revolve(2*half,(0,0,0),(0,1,0)).val().rotate((0,0,0),(0,0,1),-half))
    # Round only the two open C ends (no snag points). Rounding every ring edge left slivers
    # where the ribs cross the lip's inside corner, and the mesh was no longer watertight.
    ends=[f for f in ring.Faces() if f.geomType()=='PLANE' and abs(f.normalAt().z)<.1]
    ring=ring.fillet(.5,[e for f in ends for e in f.Edges()])
    pad=cylinder(p['guard_pad_d']/2,p['guard_pad_t'],(0,0,p['frame_t']))
    # Root collar on the motor side, outside the bell's keep-out, deepens the spoke roots.
    rc=p['motor_bell_d']/2+p['bell_spoke_clearance']+.3; zt=p['frame_t']+p['guard_pad_t']
    pad=pad.fuse(cylinder(p['guard_pad_d']/2,p['guard_collar_h'],(0,0,zt)).cut(cylinder(rc,p['guard_collar_h']+1,(0,0,zt-.5))))
    pad=holes_motor(pad,p,p['frame_t']-1,p['guard_pad_t']+p['guard_collar_h']+2)
    for s in [-1,1]: pad=pad.cut(cylinder(p['key_hole_d']/2,p['guard_pad_t']+p['guard_collar_h']+2,(0,s*p['key_offset'],p['frame_t']-1)))
    sw,sh,rt=p['guard_spoke_w'],p['guard_spoke_h'],p['guard_rib_t']
    ze=z0-sw/2+.6  # spoke end centre: its round top overlaps the band's lower edge by 0.6 mm
    each=[]
    for a in p['guard_spoke_angles_deg']:
        # Spine stays >= 5 mm under the prop disc inside the tip radius, then runs flat under
        # the band and ends flush with the rib's outer face.
        sp=tube_keel([polar(12.6,a,6.2),polar(18,a,8.6),polar(26,a,12.2),polar(34,a,15.0),
                      polar(40,a,ze-2.3),polar(44,a,ze-.5),polar(ro+rt,a,ze)],sw,sh,p['guard_keel_deg'])
        rib=rounded_box((rt,sw,z1-ze),(ro+rt/2-.01,0,(z1+ze)/2),.5).rotate((0,0,0),(0,0,1),a)
        each.append(union([sp,rib]))
    spokes=union(each).intersect(box((120,120,60),(0,0,35)))
    spokes=placed(spokes,angle,(0,0,0)); ring=placed(ring,angle,(0,0,0))
    # The pad is not rotated: its motor slots and key holes match the frame pad, which uses the same pattern at every motor.
    return union([pad,spokes,ring]),spokes,ring

def guard_spoke_coupon(p):
    """One spoke of the guard with its pad and a 44 deg piece of ring: a hand break-test of
    the new joints before printing four guards. Print pad-down like the guard."""
    g,_,_=guard(p,0)
    wedge=cq.Workplane('XY').polyline([(0,0),(70*math.cos(math.radians(22)),-70*math.sin(math.radians(22))),
                                       (70*math.cos(math.radians(22)),70*math.sin(math.radians(22)))]).close().extrude(40).val()
    return g.intersect(union([wedge,cylinder(p['guard_pad_d']/2+.01,40,(0,0,0))])).clean()

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
    # v2: single-sided plate. The MTF-02P sits in a shallow underside recess (was a
    # 2.5 mm downward wall nest that forced supports), with a cable/connector window
    # through the plate and the two retention-strap slots kept.
    fx,fy,fh=p['flow_size']; xx=p['flow_x']; rec=p['flow_recess']
    plate=plate.cut(box((fx+.8,fy+.8,rec+.01),(xx,0,z+rec/2-.005)))
    plate=plate.cut(rounded_box((fx-4,fy-4,th+2),(xx,0,z+th/2),1.5))
    for yy in [-fy/2-3.1,fy/2+3.1]: plate=plate.cut(box((6,1.8,8),(xx,yy,z)))
    # Lightening window under the battery's front half (the pack is a rigid brick).
    bx=p['batt_size'][0]/2
    plate=plate.cut(rounded_box((bx-(xx+fx/2)-4,p['batt_size'][1]-16,th+2),((bx+xx+fx/2)/2,0,z+th/2),3))
    if p['camera_mount']=='under_nose': plate=nose_cheeks(p,plate)
    return plate.clean()

def nose_cheeks(p,plate):
    """v2 under-nose camera: two upright webs rise from the plate's front corners and carry
    the pivot ahead of the battery and below the frame; the cradle swings in the open slot
    between them. The webs print standing on the bed with the plate. Their front-lower edge
    is cut back at 50 deg (self-supporting) so the TPU cradle, not PETG, meets the ground
    first in a nose-down tip-over."""
    th=p['plate_size'][2]; zb=p['plate_bottom_z']; cp=p['camera_pivot']
    cy=p['camera_size'][1]/2+p['camera_cradle_wall']+2.2; ct=3.2; r=7.5
    x0=p['batt_size'][0]/2+.5      # just ahead of the battery's front face, behind the stops' outer face
    x1=cp[0]+r
    xa=x1-(cp[2]-zb)/math.tan(math.radians(50))
    web=(cq.Workplane('XZ').moveTo(x0,zb).lineTo(xa,zb).lineTo(x1,cp[2])
         .threePointArc((cp[0],cp[2]+r),(cp[0]-r,cp[2])).lineTo(x0,zb+th+4).close()
         .extrude(ct).val())
    bb=web.BoundingBox(); parts=[plate]
    for sign in [-1,1]:
        y0=cy-1.6 if sign>0 else -(cy+1.6)
        parts.append(web.translate((0,y0-bb.ymin,0)))
        # Outboard bed flange: print adhesion and lateral stiffness; nothing inboard of the cheek.
        parts.append(rounded_box((xa-2-x0,7.2,th),((x0+xa-2)/2,sign*(cy-1.6+3.6),zb+th/2),2))
    plate=union(parts)
    for sign in [-1,1]:
        # 15 degree radial face detents on the inner faces, as on the Astra 01 frame cheeks.
        inner=sign*(cy-1.6)
        for a in range(0,360,15):
            plate=plate.cut(box((2.4,.5,1.0),(6,sign*.2,0)).rotate((0,0,0),(0,1,0),a).translate((cp[0],inner,cp[2])))
    return plate.cut(cylinder(p['camera_pivot_d']/2,60,(cp[0],-30,cp[2]),(0,1,0)))

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
    # v2: TPU lens hood - side walls and bottom run camera_hood_margin past the lens tip,
    # so a flat impact lands on TPU, not on the lens; edges stay outside the field of view.
    hood=p['camera_lens_x']+p['camera_hood_margin']-dx/2
    parts.append(box((hood+w,dy+2*w,w),(dx/2+hood/2-w/2,0,-dz/2-w/2)))
    for s in [-1,1]:
        parts.append(box((hood+w,w,dz+2*w),(dx/2+hood/2-w/2,s*(dy/2+w/2),0)))
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
    # v2: sensor top sits in the plate's underside recess.
    out['MTF02P']=box(p['flow_size'],(p['flow_x'],0,p['plate_bottom_z']+p['flow_recess']-p['flow_size'][2]/2))
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
    return {'FitCoupon_Frame_PLA':base.clean(),'FitCoupon_Guard_PLA':gp.clean(),'FitCoupon_GuardSpoke_PETG':guard_spoke_coupon(p)}
