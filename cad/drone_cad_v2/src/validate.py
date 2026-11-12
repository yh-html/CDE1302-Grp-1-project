"""Numerical checks of final solids, interfaces, meshes and conservative envelopes."""
import hashlib,itertools,json,math,time
from pathlib import Path
import numpy as np
import cadquery as cq
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from geometry import *
from render import render

def overlap_bb(a,b,tol=0):
    aa=a.BoundingBox(); bb=b.BoundingBox()
    return all(getattr(aa,k+'max')>getattr(bb,k+'min')+tol and getattr(bb,k+'max')>getattr(aa,k+'min')+tol for k in 'xyz')

def common_volume(a,b):
    if not overlap_bb(a,b,1e-5): return 0.
    return abs(a.intersect(b).Volume())

def exact_distance(a,b):
    calc=BRepExtrema_DistShapeShape()
    calc.SetMultiThread(True)
    calc.LoadS1(a.wrapped); calc.LoadS2(b.wrapped); calc.Perform()
    if not calc.IsDone(): raise RuntimeError('OpenCascade distance did not finish')
    return calc.Value()

def clearance_distance(a,b,required):
    aa=a.BoundingBox(); bb=b.BoundingBox()
    gaps=[max(getattr(aa,k+'min')-getattr(bb,k+'max'),getattr(bb,k+'min')-getattr(aa,k+'max'),0) for k in 'xyz']
    lower=float(np.linalg.norm(gaps))
    if lower>=required: return lower,'conservative AABB lower bound'
    return exact_distance(a,b),'exact B-rep minimum'

def line_thickness(shape,point,direction,span=20):
    a=np.array(point)-span*np.array(direction); b=np.array(point)+span*np.array(direction)
    e=cq.Edge.makeLine(V(*a),V(*b)); cut=shape.intersect(e)
    return sum(x.Length() for x in cut.Edges())

def view_frustum(p,tilt_angle,length=240):
    # Conservative envelope between 3-degree samples: +/-1.5 degree pitch,
    # plus a lens-origin translation margin. Horizontal widening accounts for
    # x changing under pitch rotation at the vertical field edge.
    lens=p['camera_lens_x']; delta=math.radians(1.5)
    v=math.radians(p['camera_vfov']/2)
    hv=math.tan(math.radians(p['camera_hfov']/2))/(math.cos(delta)-math.tan(v)*math.sin(delta))
    vv=math.tan(v+delta); margin=abs(lens)*2*math.sin(delta/2)+.05
    wires=[]
    for dist in [-margin,length]:
        halfy=max(dist,0)*hv+margin; halfz=max(dist,0)*vv+margin
        pts=[V(lens+dist,y*halfy,z*halfz) for y,z in [(-1,-1),(1,-1),(1,1),(-1,1)]]
        wires.append(cq.Wire.makePolygon(pts+[pts[0]]))
    return tilt(cq.Solid.makeLoft(wires,True),p,tilt_angle)

def validate(root,p,data,shapes,meshes,prints,env,spokes,rings,digest,cradle_local):
    out=root/'output'; checks=[]; details={}
    def log(s):
        with (out/'reports/progress.log').open('a') as f: f.write(s+'\n')
        try: print(s,flush=True)
        except OSError: pass
    def check(name,ok,value=None,limit=None,note=''):
        checks.append(dict(name=name,status='pass' if ok else 'fail',value=value,limit=limit,note=note))
        if not ok: log('FAIL '+name+': '+str(value))
    def pending(name,note): checks.append(dict(name=name,status='indeterminate',note=note))
    log('Validate topology and printable meshes')
    rows=[]
    for name,s in shapes.items():
        m=meshes[name]; pr=prints[name]; bounds=[s.BoundingBox().xlen,s.BoundingBox().ylen,s.BoundingBox().zlen]
        check(name+'/valid',s.isValid())
        check(name+'/one_solid',len(s.Solids())==1,len(s.Solids()),1)
        check(name+'/positive_volume',s.Volume()>1,s.Volume(),'>1 mm3')
        check(name+'/watertight',bool(pr.is_watertight and pr.is_winding_consistent),dict(watertight=bool(pr.is_watertight),winding=bool(pr.is_winding_consistent)))
        check(name+'/mesh_components',len(pr.split(only_watertight=False))==1,len(pr.split(only_watertight=False)),1)
        check(name+'/mesh_volume',abs(pr.volume-s.Volume())/s.Volume()<.01,float(pr.volume/s.Volume()-1),'+/-1%')
        check(name+'/A1_fit',bool(np.all(pr.extents<=256)),pr.extents.tolist(),[256]*3)
        check(name+'/A1_mini_fit',bool(np.all(pr.extents<=180)),pr.extents.tolist(),[180]*3)
        mat='TPU' if name.endswith('TPU') else 'PETG'; density=p[mat.lower()+'_density']; mass=s.Volume()/1000*density
        # Fraction of downward surface steeper than 45 degrees, excluding bed contact.
        below=pr.face_normals[:,2]<-math.sqrt(.5)
        below &= pr.triangles_center[:,2]>.25
        support_area=float(pr.area_faces[below].sum())
        rows.append(dict(name=name,material=mat,quantity=1,bbox_mm=bounds,print_bbox_mm=pr.extents.tolist(),volume_mm3=s.Volume(),mass_g=mass,support_candidate_area_mm2=support_area))
    details['parts']=rows
    # Four actual central slot probes in each pad: through-passing cylindrical tool.
    h=p['motor_hole_spacing']/2; probe=[]
    for i,(x,y,a) in enumerate(motor_positions(p),1):
        for dx,dy in itertools.product([-h,h],repeat=2):
            tool=cylinder(.95,20,(x+dx,y+dy,-12))
            vol=common_volume(tool,shapes['Frame_PETG'])+common_volume(tool,shapes[f'Guard_{i}_PETG'])
            probe.append(vol)
    check('motor_16_through_holes',max(probe)<1e-5,max(probe),'zero intersection mm3')
    # Slot endpoint probes confirm the full radial adjustment, not only nominal centers.
    slots=[]
    for i,(x,y,a) in enumerate(motor_positions(p),1):
        for dx,dy in itertools.product([-h,h],repeat=2):
            for shift in [-.5,.5]:
                v=np.array([dx,dy]); v=v/np.linalg.norm(v)*shift
                tool=cylinder(.95,9,(x+dx+v[0],y+dy+v[1],-1))
                slots.append(common_volume(tool,shapes['Frame_PETG'])+common_volume(tool,shapes[f'Guard_{i}_PETG']))
    check('motor_radial_slot_travel',max(slots)<1e-5,max(slots),'zero intersection mm3')
    fh=p['fc_hole_spacing']/2; vols=[]
    for x,y in itertools.product([-fh,fh],repeat=2): vols.append(common_volume(shapes['Frame_PETG'],cylinder(p['fc_hole_d']/2-.01,7,(x,y,-1))))
    check('FC_25p5_pattern_through_holes',max(vols)<1e-5,max(vols))
    plate_top=p['plate_bottom_z']+p['plate_size'][2]; b=p['body_bolt_half']; vols=[]
    for x,y in itertools.product([-b,b],repeat=2):
        tool=cylinder(p['body_bolt_d']/2-.01,70,(x,y,p['plate_bottom_z']-1))
        vols.append(common_volume(tool,shapes['Frame_PETG'])+common_volume(tool,shapes['BottomPlate_PETG']))
    check('plate_and_hollow_leg_hole_alignment',max(vols)<1e-5,max(vols))
    key_clear=(p['key_hole_d']-p['key_pin_d'])/2
    check('key_radial_clearance',key_clear>=.15,key_clear,'>=0.15 mm')
    check('motor_radius_rounding',abs(p['motor_side']/math.sqrt(2)-p['arm_r'])<.05,p['motor_side']/math.sqrt(2),'arm_r 77.8 +/-0.05')
    log('Validate prop and bell clearances')
    prop_details=[]
    for i in range(1,5):
        disc=env[f'Prop_{i}']
        for name,s in shapes.items():
            required=3 if name.startswith('Guard') else p['prop_clearance']
            dist,method=clearance_distance(s,disc,required)
            check(f'Prop_{i}/{name}',dist>=required-1e-5,dist,required,method)
            prop_details.append(dict(prop=i,part=name,distance_mm=dist,method=method))
        for name in ['FC','Battery','Camera','MTF02P','ELRS']:
            dist=exact_distance(env[name],disc); check(f'Prop_{i}/{name}',dist>=p['prop_clearance'],dist,p['prop_clearance'])
        dist=exact_distance(spokes[i],env[f'Motor_{i}'])
        check(f'Guard_{i}/bell_spoke_clearance',dist>=p['bell_spoke_clearance']-1e-5,dist,p['bell_spoke_clearance'])
        # Crop to the prop cylinder: within its footprint the uppermost spoke is below the disc.
        x,y,_=motor_positions(p)[i-1]
        below=spokes[i].intersect(cylinder(p['prop_d']/2,60,(x,y,-10)))
        gap=disc.BoundingBox().zmin-below.BoundingBox().zmax
        check(f'Guard_{i}/vertical_spoke_gap',gap>=p['guard_spoke_clearance'],gap,p['guard_spoke_clearance'])
    details['prop_clearances']=prop_details
    guard_gaps=[]
    for i,j in itertools.combinations(range(1,5),2):
        d,method=clearance_distance(shapes[f'Guard_{i}_PETG'],shapes[f'Guard_{j}_PETG'],p['guard_gap']); guard_gaps.append(d)
        check(f'Guard_{i}_{j}/separation',d>=p['guard_gap']-1e-5,d,p['guard_gap'],method)
    check('guard_min_ID',p['guard_od']-2*p['guard_wall']>=82.2,p['guard_od']-2*p['guard_wall'],'>=82.2 mm')
    # v2: the band must cover the prop envelope with 3 mm above and below (replaces the
    # literal 30 mm stock height, which included the stock guard's landing legs).
    pp=p['frame_t']+p['guard_pad_t']+p['motor_h']+p['prop_plane_above_bell']; pe=p['prop_envelope_t']/2
    band_margin=min(pp-pe-p['guard_ring_bottom_z'],p['guard_top_z']-(pp+pe))
    check('guard_ring_covers_prop_plane',band_margin>=3-1e-6,band_margin,'>=3 mm above and below the prop envelope')
    # Explicit interface accessibility corridors.
    fx,fy,fz=p['fc_size']; drop=box((fx+.4,fy+.4,35),(0,0,p['deck_bottom_z']+17))
    check('FC_insertion_through_deck',common_volume(drop,shapes['Frame_PETG'])<1e-5,common_volume(drop,shapes['Frame_PETG']),'zero mm3; 0.2 mm perimeter allowance')
    ux,uy,uz=p['usb_size']; usb=box((ux,uy,uz),(0,fy/2+uy/2,p['frame_t']+p['fc_isolator_h']+fz/2))
    check('USB_side_access',common_volume(usb,shapes['Frame_PETG'])<1e-5,common_volume(usb,shapes['Frame_PETG']),'zero mm3; +y connector assumption')
    # Flow cone includes finite lens aperture, expands to well past the feet/ground.
    lens_z=p['plate_bottom_z']+p['flow_recess']-p['flow_size'][2]; length=100  # v2: sensor sits in the recess
    cone=cq.Solid.makeCone(p['flow_lens_d']/2,p['flow_lens_d']/2+length*math.tan(math.radians(p['flow_fov']/2)),length,V(p['flow_x'],0,lens_z-.01),V(0,0,-1))
    cone_hits=[]
    for name,s in shapes.items():
        v=common_volume(cone,s)
        if v>1e-5: cone_hits.append(dict(part=name,mm3=v))
    check('flow_42deg_cone',not cone_hits,cone_hits,'zero obstacles, 100 mm long solid cone including aperture')
    ground=p['plate_bottom_z']-p['foot_h']
    check('flow_ground_clearance',lens_z-ground>=5,lens_z-ground,'>=5 mm')
    check('battery_ground_clearance',env['Battery'].BoundingBox().zmin-ground>=5,env['Battery'].BoundingBox().zmin-ground,'>=5 mm')
    log('Validate camera FOV and moving cradle at 3-degree intervals')
    cam_hits=[]; moving_hits=[]; moving_prop_gaps=[]
    angles=list(np.arange(p['camera_tilt_min'],p['camera_tilt_max']+.001,3))
    if p['camera_tilt_max'] not in angles: angles.append(p['camera_tilt_max'])
    obstacles={**shapes,**{k:s for k,s in env.items() if k!='Camera'}}
    obstacles.pop('CameraCradle_TPU')
    cb=cradle_local.BoundingBox()
    radius=math.hypot(max(abs(cb.xmin),abs(cb.xmax)),max(abs(cb.zmin),abs(cb.zmax)))
    sweep_margin=radius*2*math.sin(math.radians(1.5)/2)
    for angle in angles:
        frustum=view_frustum(p,float(angle))
        for name,s in obstacles.items():
            v=common_volume(frustum,s)
            if v>1e-4: cam_hits.append(dict(angle=float(angle),part=name,volume_mm3=v))
        moving=tilt(cradle_local,p,float(angle))
        v=common_volume(frustum,moving)
        if v>1e-4: cam_hits.append(dict(angle=float(angle),part='moving_cradle',volume_mm3=v))
        cam_body=tilt(box(p['camera_size'],(0,0,0)),p,float(angle))
        for i in range(1,5):
            for name,s in [('cradle',moving),('camera',cam_body)]:
                gap,method=clearance_distance(s,env[f'Prop_{i}'],p['prop_clearance']+sweep_margin)
                moving_prop_gaps.append(dict(angle=float(angle),part=name,prop=i,gap_mm=gap,method=method))
        for name,s in shapes.items():
            if name=='CameraCradle_TPU': continue
            if overlap_bb(moving,s):
                v=common_volume(moving,s)
                if v>1e-5: moving_hits.append(dict(angle=float(angle),part=name,volume_mm3=v))
    check('camera_FOV_sampled',not cam_hits,cam_hits,'full rectangular frustum at 3 deg tilt increments, 240 mm range')
    check('camera_cradle_motion_sampled',not moving_hits,moving_hits,'zero overlap at 3 deg increments')
    cb=cradle_local.BoundingBox()
    radius=math.hypot(max(abs(cb.xmin),abs(cb.xmax)),max(abs(cb.zmin),abs(cb.zmax)))
    sweep_margin=radius*2*math.sin(math.radians(1.5)/2)
    check('camera_motion_prop_clearance',all(g['gap_mm']>=p['prop_clearance']+sweep_margin for g in moving_prop_gaps),min(g['gap_mm'] for g in moving_prop_gaps),p['prop_clearance']+sweep_margin,'4 mm plus conservative maximum between-sample rotational travel; camera and cradle every 3 degrees')
    details['camera_motion_prop_gaps']=moving_prop_gaps
    details['camera']={'sample_angles_deg':[float(a) for a in angles],'nominal_detent_axial_gap_mm':.05,'hfov':p['camera_hfov'],'vfov':p['camera_vfov'],'unresolved':'Actual lens FOV/optical center; sampled tilt does not prove unsampled intermediate states. The axial gap is the design value, not a global minimum-distance measurement.'}
    # v2: ground clearance of the swept cradle, and the nose-down tip-over angle (about the
    # front feet's outer edge) at which each part would first touch flat ground.
    cam_low=min(tilt(cradle_local,p,float(a)).BoundingBox().zmin for a in angles)
    check('camera_ground_clearance',cam_low-ground>=5,cam_low-ground,'>=5 mm, lowest cradle point over the tilt range')
    edge_x=p['body_bolt_half']+p['foot_od']/2
    def strike(v):
        v=v[v[:,0]>edge_x+1e-6]
        return float(np.degrees(np.arctan2(v[:,2]-ground,v[:,0]-edge_x)).min()) if len(v) else 90.0
    cm=mesh_of(cradle_local,.2,.3).vertices; sweep=[]
    for a in angles:
        t=math.radians(a); x=cm[:,0]*math.cos(t)+cm[:,2]*math.sin(t); z=-cm[:,0]*math.sin(t)+cm[:,2]*math.cos(t)
        sweep.append(strike(np.c_[x+p['camera_pivot'][0],cm[:,1],z+p['camera_pivot'][2]]))
    details['nose_down_strike_deg']={'CameraCradle_TPU (worst tilt)':min(sweep),
                                     **{n:strike(meshes[n].vertices) for n in ['BottomPlate_PETG','Frame_PETG','Guard_1_PETG','Guard_2_PETG']}}
    log('Nose-down strike angles: '+', '.join(f'{k} {v:.1f}' for k,v in details['nose_down_strike_deg'].items()))
    pending('camera_actual_optics',f"Field {p['camera_hfov']:.0f} x {p['camera_vfov']:.0f} deg derived from the firmware calibration (main/at_detect.c, QVGA); lens protrusion and board orientation are still placeholders. Mechanical sweep sampled every 3 deg, not continuous proof.")
    render(meshes,out/'renders/clearance_camera_flow.png','Sight envelopes / 30 and 75 deg + downward 42 deg',hardware={k:mesh_of(v,.2,.3) for k,v in env.items()},diagnostics={'flow':mesh_of(cone,.3,.3),'camera30':mesh_of(view_frustum(p,30,85),.3,.3),'camera75':mesh_of(view_frustum(p,75,85),.3,.3)})
    log('Validate assembly collisions with B-rep intersections')
    allshapes={**shapes,**env}; collisions=[]; intended=[]
    for (an,a),(bn,bshape) in itertools.combinations(allshapes.items(),2):
        # The shaft physically enters the prop hub; rotational disc deliberately includes the hub.
        if (an.startswith('Shaft_') and bn=='Prop_'+an.split('_')[1]) or (bn.startswith('Shaft_') and an=='Prop_'+bn.split('_')[1]):
            intended.append([an,bn,'shaft/hub engagement']); continue
        v=common_volume(a,bshape)
        if v>1e-4: collisions.append(dict(a=an,b=bn,volume_mm3=v))
    check('assembly_collisions',not collisions,collisions,'zero unintended common volume > 1e-4 mm3')
    details['intentional_engagements']=intended
    log('Measure critical wall sections')
    sample_specs=[('deck',shapes['Frame_PETG'],(0,21,p['deck_bottom_z']+p['deck_t']/2),(0,0,1),4),
                  ('motor_pad',shapes['Frame_PETG'],(65,55,2.5),(0,0,1),4),
                  # v2: y 11, beside the battery lightening window (|y| <= 7) and inside the strap slots.
                  ('plate',shapes['BottomPlate_PETG'],(25,11,p['plate_bottom_z']+1.25),(0,0,1),4),
                  ('battery_pad',shapes['BatteryPad_TPU'],(0,0,plate_top+.8),(0,0,1),4),
                  ('cradle_back',cradle_local,(-p['camera_size'][0]/2-p['camera_cradle_wall']/2,10,0),(1,0,0),3)]
    if p['camera_mount']=='under_nose':
        # v2: mid-thickness of the under-nose cheek web, 9.8 mm from the pivot (clear of the detents and the 50 deg front cut).
        cy=p['camera_size'][1]/2+p['camera_cradle_wall']+2.2
        sample_specs.append(('nose_cheek_web',shapes['BottomPlate_PETG'],(p['camera_pivot'][0]-4,cy,p['camera_pivot'][2]-9),(0,1,0),4))
    sample_results=[]
    for name,s,pt,direction,span in sample_specs:
        thickness=line_thickness(s,pt,direction,span); check('wall/'+name,thickness>=p['min_wall']-1e-5,thickness,p['min_wall'])
        sample_results.append(dict(name=name,point=list(pt),direction=list(direction),thickness_mm=thickness))
    # Analytic lower bounds for circular members and intentionally constant walls.
    wall_bounds={'web':p['web_d'],'tip_chord':p['tip_chord_d'],'tie':p['tie_d'],'guard_spoke':p['guard_spoke_d'],'guard_rib':p['guard_rib_t'],'ring_midwall':p['guard_wall'],'foot':p['foot_wall'],'hollow_leg':(p['body_leg_d']-p['body_bolt_d'])/2,
                 'motor_slot_edge':p['motor_pad_d']/2-(p['motor_hole_spacing']/math.sqrt(2)+p['motor_slot_travel']/2+p['motor_slot_d']/2),
                 'guard_key_edge':p.get('guard_pad_d',p['motor_pad_d'])/2-p['key_offset']-p['key_hole_d']/2}
    wall_bounds['camera_window_to_tie_slot']=p['camera_size'][2]/2-1.5-.9-(p['camera_size'][2]-8)/2
    wall_bounds['receiver_floor_slot_to_wall']=2.4-.8
    for name,th in wall_bounds.items(): check('wall/'+name,th>=p['min_wall']-1e-5,th,p['min_wall'])
    details['wall_samples']=sample_results
    pending('continuous_minimum_wall','Critical line sections and analytic nominal member bounds checked; no certified global medial-axis wall analysis. Rounded free edges taper by construction; inspect slicer thin-wall preview and print coupons.')
    log('Compute mass, CG and inertias')
    mass_items=[]
    for row in rows:
        mass_items.append((row['name'],row['mass_g'],meshes[row['name']]))
    for name in ['Battery','FC','Camera','MTF02P','ELRS']:
        key={'Battery':'battery_mass','FC':'fc_mass','Camera':'camera_mass','MTF02P':'flow_mass','ELRS':'elrs_mass'}[name]
        mass_items.append((name,p[key],mesh_of(env[name],.12,.25)))
    for i in range(1,5):
        for prefix,key in [('Motor','motor_mass'),('Prop','prop_mass')]: mass_items.append((f'{prefix}_{i}',p[key],mesh_of(env[f'{prefix}_{i}'],.12,.25)))
    mass_items.append(('Misc combined hardware',p['misc_mass'],mesh_of(box((30,30,10),(0,0,0)))))
    total=sum(m for _,m,_ in mass_items); printed=sum(r['mass_g'] for r in rows)
    cg=sum(m*me.center_mass for _,m,me in mass_items)/total
    inertia=np.zeros((3,3)); entries=[]
    for name,m,me in mass_items:
        d=me.center_mass-cg; intrinsic=me.moment_inertia*(m/me.volume)
        inertia+=intrinsic+m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
        entries.append(dict(name=name,mass_g=m,cg_cad_mm=me.center_mass.tolist()))
    flip=np.diag([1,1,-1]); flight_cg=flip@cg; flight_inertia=flip@inertia@flip*1e-9
    tw=4*p['Ct']*p['omega_max']**2/(total/1000*p['gravity'])
    mass=dict(printed_structure_g=printed,all_up_g=total,cg_flight_mm=flight_cg.tolist(),inertia_CG_kg_m2=flight_inertia.tolist(),J_diagonal_kg_m2=np.diag(flight_inertia).tolist(),thrust_to_weight_simulation=tw,items=entries,
              method='Printed masses from exact B-rep volumes and placeholder solid densities; intrinsic inertia from the export tessellation. Hardware mass overrides. Prop uniform-disc inertia approximation. Misc mass distributed over a 30 x 30 x 10 mm central box. No slicer infill deduction.',
              simulation_mass_g=302,simulation_J=[3.92e-4,4.05e-4,6.39e-4],legacy_mass_g=234.38404596154608)
    details['mass']=mass
    legacy=json.loads((root/'config/legacy_mass_reference.json').read_text())
    mass['legacy_mass_g']=legacy['M_kg']*1000
    mass['legacy_reference_note']='Comparison uses the captured legacy_mass_reference.json snapshot (244.68 g), obtained after external export changes. The initial project summary cited 234.38 g; its original mass_raw.json hash is no longer available, so that earlier model cannot be fully recomputed.'
    legacy_printed=sum(r['g'] for r in legacy['rows'] if r['comp']!='Placeholders')
    legacy_cg=np.array(legacy['cg_cm'])*.01
    legacy_J=np.array(legacy['I_origin_kgcm2'][:3])*1e-4-legacy['M_kg']*(np.dot(legacy_cg,legacy_cg)-legacy_cg**2)
    mass['legacy_printed_g']=legacy_printed
    mass['legacy_J_CG_kg_m2']=legacy_J.tolist()
    mass['printed_mass_change_vs_legacy_g']=printed-legacy_printed
    mass['all_up_change_vs_legacy_pct']=(total/(legacy['M_kg']*1000)-1)*100
    mass['all_up_change_vs_simulation_pct']=(total/302-1)*100
    mass['J_change_vs_simulation_pct']=((np.diag(flight_inertia)/np.array(mass['simulation_J'])-1)*100).tolist()
    check('inertia_positive_definite',bool(np.all(np.linalg.eigvalsh(flight_inertia)>0)),np.linalg.eigvalsh(flight_inertia).tolist())
    # Read-only preservation check; hashes in the manifest are relative when rebuilt elsewhere.
    hashes=json.loads((root/'config/legacy_hashes.json').read_text(encoding='utf-8-sig')); legacy=[]; legacy_details=[]
    for item in hashes:
        path=root.parents[1]/'CAD'/'exports'/Path(item['Path']).name
        candidates=[path,path.parent/'rev1'/path.name]
        found=[q for q in candidates if q.exists()]
        matched=[str(q) for q in found if hashlib.sha256(q.read_bytes()).hexdigest().upper()==item['Hash']]
        if found: legacy.append(bool(matched))
        legacy_details.append(dict(name=path.name,matching_paths=matched,baseline_found=bool(matched)))
    details['legacy_preservation']=legacy_details
    if legacy:
        check('legacy_exports_untouched',all(legacy) and len(legacy)==len(hashes),sum(legacy),len(hashes),'Read-only comparison with initial SHA256 manifest, searching original paths and rev1. See details.legacy_preservation for missing baseline hashes. This project writes only under codex/drone_cad and does not restore or overwrite external exports.')
    else:
        pending('legacy_exports_untouched','Reference CAD/exports directory not present in this standalone checkout; preservation cannot be rechecked here.')
    engagement=p['motor_screw_length']-p['frame_t']-p['guard_pad_t']
    details['motor_screw_engagement_mm']=engagement
    for name,note in {
        'safe_motor_thread_engagement':f'M2x10 nominal engagement {engagement:.1f} mm, without washer allowance. Maximum safe motor thread depth unknown. Verify engagement and winding clearance; do not fit longer unverified screws.',
        'MTF_mount_holes':'Unknown pitch: retention nest uses foam tape plus a nonoptical retaining strap; verify actual board and lens/connector offsets.',
        'ELRS_fit':'20 x 12 x 4 mm placeholder with antenna notch and ties. Measure real board/antenna bend radius.',
        'strap_fit':'12 x 2 mm placeholder strap, 1 mm width / 0.6 mm thickness allowance. Check threading and secure battery under inversion.',
        'material_density':'PETG 1.27 and TPU 1.21 g/cm3 placeholders; weigh actual prints. Solid-volume mass is not slicer mass.',
        'physical_mass_CG':'Current drone actual all-up mass and CG not supplied; compare measured assembled prototype with estimates.',
        'physical_strength':'No FEA, fatigue, crash, layer adhesion, motor vibration or flight qualification. Geometry validity is not proof of structural safety.',
        'motor_prop_envelope':'Hub seating and full blade axial swept envelope assumed. Measure physical prop height/sweep and rerun clearances.',
        'electrical_routing':'USB corridor and antenna notch modeled; solder joints, flex cables, wire bend radii and screw-tool sweeps require dry assembly.'
    }.items(): pending(name,note)
    result={'schema':1,'geometry_digest':digest,'axes':'CAD x forward, y right, z up; flight mass report x forward, y right, z down','checks':checks,'details':details,'summary':{status:sum(c['status']==status for c in checks) for status in ['pass','fail','indeterminate']}}
    (out/'reports/validation.json').write_text(json.dumps(result,indent=2))
    write_reports(out,result,p)
    log('Validation: '+json.dumps(result['summary']))

def write_reports(out,result,p):
    d=result['details']; mass=d['mass']; fails=[c for c in result['checks'] if c['status']=='fail']; unknowns=[c for c in result['checks'] if c['status']=='indeterminate']
    text='# Engineering validation — Astra 01 v2\n\n**Prototype; not flight-qualified.** Geometry checks do not establish crash survivability or structural safety.\n\n'
    text+=f"Automated checks: **{result['summary']['pass']} pass, {len(fails)} fail, {len(unknowns)} indeterminate**. See `validation.json` for every numeric result and collision pair.\n\n"
    text+='## Computed geometry and mass\n\n'
    text+=f"- Printed solid-volume structure: **{mass['printed_structure_g']:.2f} g**.\n- Estimated all-up mass: **{mass['all_up_g']:.2f} g**, vs captured legacy {mass['legacy_mass_g']:.2f} g and simulation 302 g.\n- CG, x forward/y right/z down: **({', '.join(f'{v:.2f}' for v in mass['cg_flight_mm'])}) mm**.\n- Jxx/Jyy/Jzz about CG: **{' / '.join(f'{v:.6g}' for v in mass['J_diagonal_kg_m2'])} kg m²**.\n- Simulation inertia: 3.92 / 4.05 / 6.39 × 10⁻⁴ kg m².\n- Estimated thrust-to-weight: **{mass['thrust_to_weight_simulation']:.2f}**, using 4 Ct ωmax²/(m g), Ct=2.9625e-7 and ωmax=3232 rad/s. This simulation estimate is not a measured 3S motor thrust result.\n\n"
    text+=mass['legacy_reference_note']+'\n\n'
    ring_id=p['guard_od']-2*p['guard_wall']
    text+=mass['method']+f"\n\nThe 110 mm motor side spacing implies a 77.7817 mm radius; the supplied arm_r=77.8 is rounded. Guard OD={p['guard_od']:.0f} mm, ring ID={ring_id:.0f} mm (required >= 82.2 mm, prop-tip gap {(ring_id-p['prop_d'])/2:.1f} mm); ring band z {p['guard_ring_bottom_z']:.1f}-{p['guard_top_z']:.1f} mm, centred on the prop plane.\n\n"
    text+=f"Legacy printed structure was {mass['legacy_printed_g']:.2f} g; change {mass['printed_mass_change_vs_legacy_g']:+.2f} g. All-up estimate changes {mass['all_up_change_vs_legacy_pct']:+.1f}% vs the legacy draft and {mass['all_up_change_vs_simulation_pct']:+.1f}% vs simulation. Legacy CG-corrected Jxx/Jyy/Jzz: {' / '.join(f'{v:.6g}' for v in mass['legacy_J_CG_kg_m2'])} kg m² (converted from supplied origin inertias). Simulation inertia changes: {' / '.join(f'{v:+.1f}%' for v in mass['J_change_vs_simulation_pct'])}. Comparisons inherit the legacy placeholder masses and density assumptions.\n\n"
    if 'nose_down_strike_deg' in d:
        text+='Nose-down tip-over about the front feet: first ground contact at '+', '.join(f'{k} {v:.1f} deg' for k,v in d['nose_down_strike_deg'].items())+'. Lower angle = touches first.\n\n'
    text+='## Failures\n\n'
    if not fails: text+='No failures in implemented checks for the configured placeholder envelopes.\n\n'
    for c in fails: text+=f"- **{c['name']}**: {c.get('value')}; required {c.get('limit')}. {c.get('note','')}\n"
    text+='\n## Indeterminate and physical verification\n\n'
    for c in unknowns: text+=f"- **{c['name']}**: {c['note']}\n"
    text+='\n## Methods and limitations\n\n- B-rep validity, one-solid counts and positive volume from OpenCascade. STL winding, watertightness, connectivity and volume agreement from independently processed triangle meshes.\n- Conservative bounding-box separation bounds, with exact B-rep minimum distances wherever a bound does not establish the required clearance, to 2 mm thick prop discs; intentional motor-shaft/prop-hub engagement excluded from collisions. Guard pad is an intentional motor mounting surface; bell-to-spoke check excludes that pad.\n- Assembly common-volume intersections use bounding-box broad phase and a 1e-4 mm³ reporting threshold. Touching mating faces are allowed; penetration is not.\n- Camera FOV uses full rectangular solid frusta at 3° tilt increments from 30–75°, inflated to cover the intervening ±1.5° pitch and lens-origin motion. Moving camera/prop clearance includes the maximum between-sample rotational travel. Camera/frame mechanical overlap remains a sampled check; actual optics are unverified. MTF analysis uses a full expanding solid cone from a finite assumed lens aperture.\n- Wall checks combine local line intersections and analytic nominal diameters/ligaments, not exhaustive minimum wall measurement. Support area is the downward-facing surface beyond 45°, excluding the first 0.25 mm above the bed. It is not a slicer prediction.\n- All assembly STEP coordinates use z up; CG/inertia are transformed to requested flight z down. Individual print meshes are transformed to the bed; STEP parts retain assembly coordinates.\n- Original exports are checked against the preserved SHA256 manifest.\n'
    (out/'reports/VALIDATION.md').write_text(text,encoding='utf-8')
    sheet='# Print sheet — Astra 01 v2\n\nOne part per manufacturing file; each listed file quantity 1. Four separate guard and foot files are included. 0.4 mm nozzle, 0.2 mm layers. These are starting settings requiring slicer review and fit coupons. Masses assume solid CAD volume and placeholder densities.\n\n| Part | Qty | Material | Print box mm | Volume cm³ | Solid mass g | Orientation | Walls / infill | Supports |\n|---|---:|---|---|---:|---:|---|---|---|\n'
    for row in d['parts']:
        name=row['name']; orientation='As exported, feet base up' if name.startswith('Foot') else 'Pad down, as exported' if name.startswith('Guard') else 'Cradle back on bed' if name.startswith('Camera') else 'Flat, as exported'
        support='Tree, build plate only; inspect interfaces' if name.startswith(('Frame','Guard','Bottom')) else 'Inspect overhangs; avoid TPU supports'
        if name.startswith('Bottom'): support='None: flat, battery stops up; the 0.8 mm sensor recess bridges'+('; the camera cheeks stand on the bed, front edge 50 deg, M3 pivot holes bridge' if p['camera_mount']=='under_nose' else '')
        if name.startswith('Guard'): support='Tree, build plate only, under the ring lower edge between spokes; none on the spokes or pad'
        if name.startswith('Frame'): support='Tree, build plate only, for the deck and upper chords'+(' and camera cheeks' if p['camera_mount']=='top' else '')+'; pads, lower chords and ties are on the bed'
        settings='4 walls / 35% gyroid' if row['material']=='PETG' else '4 walls / 30% gyroid, slow TPU profile'
        if name.startswith('Guard'): settings='6 walls (spokes and ring are all wall; infill does nothing); 240-250 C, part fan 30-40% so layers fuse'
        if name.startswith('BatteryPad'): settings='Solid, 100%'
        sheet+=f"| {name} | 1 | {row['material']} | {' × '.join(f'{v:.1f}' for v in row['print_bbox_mm'])} | {row['volume_mm3']/1000:.3f} | {row['mass_g']:.2f} | {orientation} | {settings} | {support} |\n"
    sheet+='\nBoth printers: check reported individual-part bed fit; allow brim/support margin. Whole assembled mesh is for inspection, never slice it as a single part. STEP parts retain assembly positions; STL/3MF parts are already placed/oriented at z=0. A 3MF is generic Core 3MF, not a Bambu slicer project.\n\nUse PETG for the rigid parts and TPU 95A for the cradle/feet/pad. PLA is only for interface coupons. Keep supports out of motor slots and pin bores, remove all debris, drill/ream only to the verified nominal interface and inspect layer bonding. v2: the frame underside is flat, so the motor pads and their screw seats print directly on the bed.\n'
    sheet+='\nOptional fit coupons: print one `FitCoupon_Frame_PLA` and one `FitCoupon_Guard_PLA` flat as exported, 4 walls, 100% infill, no support. These test the four motor slots and paired locating pins/bores; they are excluded from assembly mass.\n'
    sheet+='\n`FitCoupon_GuardSpoke_PETG`: one guard spoke with its pad and a 44 deg piece of ring. Print it in PETG, pad down, with the guard settings, then break-test the spoke and ring joint by hand before printing four guards.\n'
    (out/'reports/PRINT_SHEET.md').write_text(sheet,encoding='utf-8')
