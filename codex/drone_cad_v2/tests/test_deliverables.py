import json,sys,zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
import trimesh
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from parts import battery_pad,guard
from geometry import motor_positions

@pytest.fixture(scope='module')
def report(): return json.loads((ROOT/'output/reports/validation.json').read_text())

def test_machine_checkable_requirements(report):
    failures=[c for c in report['checks'] if c['status']=='fail']
    assert not failures, json.dumps(failures,indent=2)

def test_every_part_has_manufacturing_exports(report):
    for row in report['details']['parts']:
        for folder,ext in [('step','step'),('stl','stl'),('3mf','3mf')]:
            f=ROOT/'output'/folder/(row['name']+'.'+ext)
            assert f.exists() and f.stat().st_size>100

def test_3mf_round_trip_matches_stl(report):
    ns={'m':'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
    names=[row['name'] for row in report['details']['parts']]+['FitCoupon_Frame_PLA','FitCoupon_Guard_PLA']
    for name in names:
        with zipfile.ZipFile(ROOT/'output/3mf'/f'{name}.3mf') as z:
            model=ET.fromstring(z.read('3D/3dmodel.model'))
        assert model.attrib['unit']=='millimeter'
        vs=[[float(v.attrib[k]) for k in ['x','y','z']] for v in model.findall('.//m:vertex',ns)]
        fs=[[int(v.attrib[k]) for k in ['v1','v2','v3']] for v in model.findall('.//m:triangle',ns)]
        mesh=trimesh.Trimesh(vs,fs,process=True)
        stl=trimesh.load_mesh(ROOT/'output/stl'/f'{name}.stl')
        assert mesh.is_watertight and mesh.is_winding_consistent
        assert stl.is_watertight and stl.is_winding_consistent
        assert abs(mesh.volume/stl.volume-1)<1e-5
        assert mesh.bounds[0,2]>=-1e-5

def test_parameter_change_rebuilds_battery_pad():
    p={k:v['value'] for k,v in json.loads((ROOT/'config/parameters.json').read_text())['parameters'].items()}
    before=battery_pad(p)
    p['batt_size']=[p['batt_size'][0]+4,p['batt_size'][1]+2,p['batt_size'][2]]
    after=battery_pad(p)
    assert after.isValid() and len(after.Solids())==1
    assert abs(after.BoundingBox().xlen-before.BoundingBox().xlen-4)<1e-5
    assert abs(after.BoundingBox().ylen-before.BoundingBox().ylen-2)<1e-5
    assert after.Volume()>before.Volume()

def test_required_views_and_assemblies():
    for name in ['assembly_iso','assembly_top','assembly_side','assembly_underside','assembly_exploded','clearance_iso','clearance_camera_flow','topology_candidates']:
        assert (ROOT/'output/renders'/f'{name}.png').stat().st_size>10000
    for name in ['Assembly_printed','Assembly_with_hardware']:
        assert (ROOT/'output/step'/f'{name}.step').stat().st_size>100000

def test_unknowns_are_explicit(report):
    pending={c['name'] for c in report['checks'] if c['status']=='indeterminate'}
    assert {'safe_motor_thread_engagement','MTF_mount_holes','ELRS_fit','strap_fit','material_density','physical_mass_CG','physical_strength'}.issubset(pending)
