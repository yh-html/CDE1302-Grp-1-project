"""One-command geometry, manufacturing export, rendering and engineering validation."""
import argparse,hashlib,json,sys,time,zipfile
from pathlib import Path
import cadquery as cq
import trimesh
from geometry import *
import parts
from render import render,topology_render,color

ROOT=Path(__file__).resolve().parents[1]; OUT=ROOT/'output'
def log(message):
    with (OUT/'reports/progress.log').open('a',encoding='utf-8') as f: f.write(message+'\n')
    try: print(message,flush=True)
    except OSError: pass

def export_3mf(mesh,path,name):
    # Standards-compliant core 3MF, in mm, one named printable object, no slicer presets.
    import xml.etree.ElementTree as E
    ns='http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
    E.register_namespace('',ns)
    model=E.Element('{'+ns+'}model',unit='millimeter',attrib={'xml:lang':'en-US'})
    E.SubElement(model,'metadata',name='Title').text=name
    res=E.SubElement(model,'resources'); obj=E.SubElement(res,'object',id='1',type='model',name=name)
    me=E.SubElement(obj,'mesh'); verts=E.SubElement(me,'vertices'); tris=E.SubElement(me,'triangles')
    for x,y,z in mesh.vertices: E.SubElement(verts,'vertex',x=f'{x:.7f}',y=f'{y:.7f}',z=f'{z:.7f}')
    for a,b,c in mesh.faces: E.SubElement(tris,'triangle',v1=str(a),v2=str(b),v3=str(c))
    bu=E.SubElement(model,'build'); E.SubElement(bu,'item',objectid='1')
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml','<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr('_rels/.rels','<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rel0" Target="/3D/3dmodel.model" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr('3D/3dmodel.model',E.tostring(model,encoding='utf-8',xml_declaration=True))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--force',action='store_true'); ap.add_argument('--geometry-only',action='store_true'); args=ap.parse_args()
    (OUT/'reports/error.txt').unlink(missing_ok=True)
    data=json.loads((ROOT/'config/parameters.json').read_text()); p={k:v['value'] for k,v in data['parameters'].items()}
    digest=hashlib.sha256((ROOT/'config/parameters.json').read_bytes()+(ROOT/'src/parts.py').read_bytes()+(ROOT/'src/geometry.py').read_bytes()).hexdigest()
    cache=OUT/'cache'/digest[:16]; cache.mkdir(parents=True,exist_ok=True)
    for d in ['step','stl','3mf','renders','reports']: (OUT/d).mkdir(exist_ok=True)
    def get(name,fn):
        f=cache/(name+'.brep')
        if f.exists() and not args.force: log('Read '+name); return cq.Shape.importBrep(str(f))
        log('Build '+name); start=time.monotonic(); s=fn()
        if not s.isValid() or len(s.Solids())!=1 or s.Volume()<1:
            raise ValueError(f'{name}: invalid or disconnected solid; refusing export')
        s.exportBrep(str(f)); log(f'  {time.monotonic()-start:.1f}s, {len(s.Solids())} solids, valid={s.isValid()}'); return s
    # Three real low-detail solid candidates, each with the same end coordinates.
    candidates={name:get('candidate_'+name,lambda name=name:parts.arm(p,name)) for name in ['arch','fork','cage']}
    cm={k:mesh_of(s,.15,.3) for k,s in candidates.items()}; topology_render(cm,OUT/'renders/topology_candidates.png')
    scores={'arch':[5,2,4,4,3],'fork':[4,5,3,4,5],'cage':[2,4,2,3,4]}
    text='# Topology decision\n\nSelected **fork**: paired splayed lower chords, a single curved upper chord and two branching webs.\n\nAll three candidates were built as low-detail B-rep solids and rendered before the detailed assembly. Scores are ordinal engineering judgment (5 best), not FEA or measured strength.\n\n| Candidate | Arm volume mm3 | Mass tendency | Crash load path | Printability | Interface risk (5 low) | Visual fit | Total |\n|---|---:|---:|---:|---:|---:|---:|---:|\n'
    for name,s in candidates.items(): text+=f'| {name} | {s.Volume():.1f} | '+ ' | '.join(map(str,scores[name]))+f' | {sum(scores[name])} |\n'
    text+='\nArch has the least material but lacks the forked torsional load path. Cage adds tip material and crowded supports. Fork spreads the lower root in plan, keeps visible deep lens cells and preserves a single unobstructed electronics well. Adjacent-arm ties close a torsional path. No load test or buckling analysis is implied.\n'
    (OUT/'reports/TOPOLOGY_DECISION.md').write_text(text)
    shapes={'Frame_PETG':get('Frame',lambda:parts.frame(p))}
    spokes={}; rings={}
    for i,(x,y,a) in enumerate(motor_positions(p),1):
        def build_guard(a=a):
            g,sp,ri=parts.guard(p,a); sp.exportBrep(str(cache/f'spokes_{a}.brep')); ri.exportBrep(str(cache/f'ring_{a}.brep')); return g
        g=get(f'Guard_{i}',build_guard)
        shapes[f'Guard_{i}_PETG']=g.translate((x,y,0))
        spokes[i]=cq.Shape.importBrep(str(cache/f'spokes_{a}.brep')).translate((x,y,0))
        rings[i]=cq.Shape.importBrep(str(cache/f'ring_{a}.brep')).translate((x,y,0))
    shapes['BottomPlate_PETG']=get('BottomPlate',lambda:parts.bottom_plate(p))
    shapes['BatteryPad_TPU']=get('BatteryPad',lambda:parts.battery_pad(p))
    cl=get('CameraCradle',lambda:parts.cradle(p)); shapes['CameraCradle_TPU']=tilt(cl,p)
    ft=get('Foot',lambda:parts.foot(p)); b=p['body_bolt_half']
    for i,(x,y) in enumerate([(x,y) for x in [-b,b] for y in [-b,b]],1): shapes[f'Foot_{i}_TPU']=ft.translate((x,y,0))
    env=parts.envelopes(p)
    meshes={}; prints={}; assembly=cq.Assembly(name='CDE1302_Astra_01_v2')
    for name,s in shapes.items():
        log('Export '+name)
        cq.exporters.export(s,str(OUT/'step'/f'{name}.step'))
        m=mesh_of(s,p['mesh_tolerance'],p['mesh_angle']); meshes[name]=m
        # Printable exports are separate parts normalized to the bed, with explicit transforms.
        pr=m.copy()
        # v2 rev B: guards print pad-down, as modelled (Astra 01 flipped them ring-down).
        if name.startswith('Foot'): pr.apply_transform(trimesh.transformations.rotation_matrix(math.pi,[1,0,0]))
        if name.startswith('CameraCradle'):
            pr=mesh_of(cl,p['mesh_tolerance'],p['mesh_angle'])
            pr.apply_transform(trimesh.transformations.rotation_matrix(-math.pi/2,[0,1,0]))
        pr.apply_translation([-pr.bounds[:,0].mean(),-pr.bounds[:,1].mean(),-pr.bounds[0,2]])
        pr.export(OUT/'stl'/f'{name}.stl'); export_3mf(pr,OUT/'3mf'/f'{name}.3mf',name); prints[name]=pr
        assembly.add(s,name=name,color=cq.Color(*color(name)))
    assembly.save(str(OUT/'step/Assembly_printed.step'))
    for name,s in parts.fit_coupons(p).items():
        cq.exporters.export(s,str(OUT/'step'/f'{name}.step'))
        m=mesh_of(s,p['mesh_tolerance'],p['mesh_angle'])
        m.apply_translation([-m.bounds[:,0].mean(),-m.bounds[:,1].mean(),-m.bounds[0,2]])
        m.export(OUT/'stl'/f'{name}.stl'); export_3mf(m,OUT/'3mf'/f'{name}.3mf',name)
    for name,s in env.items(): assembly.add(s,name=name,color=cq.Color(*color(name),.5 if name.startswith('Prop') else 1))
    assembly.save(str(OUT/'step/Assembly_with_hardware.step'))
    trimesh.util.concatenate(list(meshes.values())).export(OUT/'stl/Assembly_VISUAL_ONLY.stl')
    scene=trimesh.Scene()
    for name,m in meshes.items():
        mc=m.copy(); mc.visual.face_colors=[int(255*v) for v in color(name)]+[255]; scene.add_geometry(mc,node_name=name)
    scene.export(OUT/'Assembly.glb')
    log('Render assembly views')
    em={k:mesh_of(v,.12,.22) for k,v in env.items()}
    for view in ['iso','top','side','underside','front']:
        render(meshes,OUT/'renders'/f'assembly_{view}.png','Organic deep-truss / '+view,view)
    render(meshes,OUT/'renders/assembly_exploded.png','Assembly / exploded',explode=True)
    render(meshes,OUT/'renders/clearance_iso.png','Hardware envelopes / clearance',hardware=em)
    render(meshes,OUT/'renders/clearance_side.png','Hardware envelopes / side',view='side',hardware=em)
    render({'Frame_PETG':meshes['Frame_PETG']},OUT/'renders/frame_detail.png','Frame / integrated deck'+(' and camera cheeks' if p['camera_mount']=='top' else ''))
    render({'Guard_1':meshes['Guard_1_PETG']},OUT/'renders/guard_detail.png','Guard / C ring with five ribbed keel spokes')
    if not args.geometry_only:
        from validate import validate
        validate(ROOT,p,data,shapes,meshes,prints,env,spokes,rings,digest,cl)
    log('Build complete')

if __name__=='__main__':
    try: main()
    except BaseException:
        import traceback
        (OUT/'reports/error.txt').write_text(traceback.format_exc())
        raise
