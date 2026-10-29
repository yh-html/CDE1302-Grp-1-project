"""v3 step (a): measure the stock STLs: volume, mass, bbox, plan area under the 4 prop discs."""
import json, sys, numpy as np, trimesh
from pathlib import Path
from PIL import Image, ImageDraw
ROOT=Path(__file__).resolve().parents[3]
S=ROOT/'Actual Mod STL of drone frame'
RHO=1.27; PROP_R=38.1; RES=0.25  # mm/px
out={}
def load(n): return trimesh.load(S/n)
def raster(mesh, xy_extent=130):
    """plan-view occupancy: a pixel is 'covered' if any triangle projects onto it (z-ignorant silhouette)."""
    n=int(2*xy_extent/RES); img=Image.new('L',(n,n),0); d=ImageDraw.Draw(img)
    v=mesh.vertices[:,:2]; 
    for f in mesh.faces:
        p=[((v[i][0]+xy_extent)/RES,(xy_extent-v[i][1])/RES) for i in f]
        d.polygon(p,fill=1)
    return np.array(img,bool)
def disc_mask(cx,cy,r,xy_extent=130):
    n=int(2*xy_extent/RES); yy,xx=np.mgrid[0:n,0:n]
    X=xx*RES-xy_extent; Y=xy_extent-yy*RES
    return (X-cx)**2+(Y-cy)**2<=r*r
discs=[disc_mask(sx*56.5,sy*56.5,PROP_R) for sx in(-1,1) for sy in(-1,1)]
disc_union=np.any(discs,axis=0); disc_area=disc_union.sum()*RES**2
rep={}
m=load('main_frame.stl'); g=load('prop_guard(x4).stl')
for name,mesh in(('main_frame',m),('prop_guard(x4)',g),('bottom_panel',load('bottom_panel.stl')),('landing_leg(x2)',load('landing_leg(x2).stl'))):
    rep[name]=dict(watertight=bool(mesh.is_watertight),volume_mm3=round(float(mesh.volume),1),mass_g_at_1p27=round(float(mesh.volume)*RHO/1000,2),
                   bbox_min=[round(float(x),2) for x in mesh.bounds[0]],bbox_max=[round(float(x),2) for x in mesh.bounds[1]])
# guard file is ONE guard with ring centre/pad at local origin, C opening toward local (-1,+1).
# Rotate so the opening faces the body centre at each motor, then translate to the motor.
rm=raster(m); rep['main_frame']['area_under_props_mm2']=round(float((rm&disc_union).sum()*RES**2),0)
rep['prop_disc_union_mm2']=round(float(disc_area),0)
gall=np.zeros_like(rm)
a0=np.arctan2(1,-1)
for sx in(-1,1):
    for sy in(-1,1):
        a=np.arctan2(-sy,-sx)-a0; c,s_=np.cos(a),np.sin(a)
        gm=g.copy(); v=gm.vertices.copy()
        gm.vertices=np.c_[c*v[:,0]-s_*v[:,1]+sx*56.5, s_*v[:,0]+c*v[:,1]+sy*56.5, v[:,2]]
        gall|=raster(gm)
rep['guards_x4']={'area_under_props_mm2':round(float((gall&disc_union).sum()*RES**2),0)}
rep['frame_plus_guards_area_under_props_mm2']=round(float(((gall|rm)&disc_union).sum()*RES**2),0)
Image.fromarray(((rm|gall)*255).astype('uint8')).save(Path(__file__).parent/'_baseline_plan.png')
print(json.dumps(rep,indent=1))
json.dump(rep,open(ROOT/'codex/drone_cad_v3/output/reports/baseline_stock.json','w'),indent=1)
