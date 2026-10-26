import json,sys,zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
import trimesh
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import parts3

@pytest.fixture(scope='module')
def report(): return json.loads((ROOT/'output/reports/validation.json').read_text())

def params(): return {k:v['value'] for k,v in json.loads((ROOT/'config/parameters.json').read_text())['parameters'].items()}

# checks that are targets reported honestly, not machine-verifiable requirements
TARGET_ONLY={'mass_target_55g_printed_set','mass_stretch_45g','lighter_than_stock'}

def test_machine_checkable_requirements(report):
    failures=[c for c in report['checks'] if c['status']=='fail' and c['name'] not in TARGET_ONLY]
    assert not failures, json.dumps(failures,indent=2)

def test_mass_targets_reported_honestly(report):
    # the 55 g and 45 g targets are NOT met in v3; the test pins that the report says so
    m=report['details']['mass']['printed_set_comparable_to_stock_g']
    st={c['name']:c['status'] for c in report['checks']}
    assert st['mass_target_55g_printed_set']==('pass' if m<=55 else 'fail')
    assert st['mass_stretch_45g']==('pass' if m<=45 else 'fail')
    assert st['lighter_than_stock']==('pass' if m<68.5 else 'fail')   # rev E guards: the printed set is HEAVIER than stock; the report must say so

def test_fixed_interface_holes_within_0p05(report):
    for key in ('frame_holes','plate_holes')+tuple(f'guard_{i}_holes' for i in range(1,5)):
        for row in report['details'][key]: assert row['ok'] and row['max_error']<=0.05,(key,row)

def test_every_part_has_manufacturing_exports(report):
    for row in report['details']['parts']:
        for folder,ext in [('step','step'),('stl','stl'),('3mf','3mf')]:
            f=ROOT/'output'/folder/(row['name']+'.'+ext)
            assert f.exists() and f.stat().st_size>100

def test_3mf_round_trip_matches_stl(report):
    ns={'m':'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
    names=[row['name'] for row in report['details']['parts']]+['FitCoupon_Frame_PLA','FitCoupon_Guard_PLA','FitCoupon_GuardSpoke_PETG','FitCoupon_ArmBox_PETG','FitCoupon_FlowCover_TPU','FitCoupon_FlowSlot_PETG']
    for name in names:
        with zipfile.ZipFile(ROOT/'output/3mf'/f'{name}.3mf') as z: model=ET.fromstring(z.read('3D/3dmodel.model'))
        assert model.attrib['unit']=='millimeter'
        vs=[[float(v.attrib[k]) for k in ['x','y','z']] for v in model.findall('.//m:vertex',ns)]
        fs=[[int(v.attrib[k]) for k in ['v1','v2','v3']] for v in model.findall('.//m:triangle',ns)]
        mesh=trimesh.Trimesh(vs,fs,process=True); stl=trimesh.load_mesh(ROOT/'output/stl'/f'{name}.stl')
        assert mesh.is_watertight and mesh.is_winding_consistent and stl.is_watertight
        assert abs(mesh.volume/stl.volume-1)<1e-5
        assert mesh.bounds[0,2]>=-1e-5

def test_parameter_change_rebuilds_battery_pad():
    p=params(); before=parts3.battery_pad(p)
    p['batt_size']=[p['batt_size'][0]+4,p['batt_size'][1]+2,p['batt_size'][2]]
    after=parts3.battery_pad(p)
    assert after.isValid() and len(after.Solids())==1 and after.Volume()>before.Volume()

def test_parameter_change_rebuilds_frame():
    p=params(); a=parts3.frame(p); p['arm_wall']=2.1; b=parts3.frame(p)
    assert b.isValid() and len(b.Solids())==1 and b.Volume()>a.Volume()

def test_required_views_and_assemblies():
    for name in ['assembly_iso','assembly_top','assembly_side','assembly_underside','assembly_exploded','clearance_iso','frame_detail','guard_detail','flow_cover_underside','flow_cover_side']:
        assert (ROOT/'output/renders'/f'{name}.png').stat().st_size>10000
    for name in ['Assembly_printed','Assembly_with_hardware']: assert (ROOT/'output/step'/f'{name}.step').stat().st_size>100000

def test_unknowns_are_explicit(report):
    pending={c['name'] for c in report['checks'] if c['status']=='indeterminate'}
    assert {'safe_motor_thread_engagement','standoff_length','guard_band_height','arm_depth_cap_9mm','flow_sensor_lens_connector_holes','ELRS_fit','strap_fit','material_density','physical_mass_CG','physical_strength'}.issubset(pending)


def test_guard_stl_and_3mf_are_watertight():
    for i in range(1,5):
        for ext in ('stl',):
            m=trimesh.load_mesh(ROOT/'output/stl'/f'Guard_{i}_PETG.stl')
            assert m.is_watertight and m.is_winding_consistent and m.volume>0, f'Guard_{i}_PETG not watertight'
    for name in ('Frame_PETG','BottomPlate_PETG','FitCoupon_GuardSpoke_PETG','FitCoupon_Guard_PLA','FlowCover_TPU'):
        m=trimesh.load_mesh(ROOT/'output/stl'/f'{name}.stl'); assert m.is_watertight, name
