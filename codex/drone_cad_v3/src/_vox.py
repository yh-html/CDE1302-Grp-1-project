import trimesh, numpy as np
m=trimesh.load(r'C:\cde1302\Actual Mod STL of drone frame\main_frame.stl')
p=0.3
v=m.voxelized(p).fill()
pts=v.points; vol=p**3
x,y=pts[:,0],pts[:,1]
tot=len(pts)*vol
print('total vox vol',tot,'mesh',m.volume)
# pads: disc r=10 about each motor
pad=np.zeros(len(pts),bool)
for sx in(-1,1):
  for sy in(-1,1): pad|=(x-sx*56.5)**2+(y-sy*56.5)**2<=100
body=(np.abs(x)<=17)&(~pad)
arm=(~body)&(~pad)
print('pads',pad.sum()*vol,'body(|x|<=17)',body.sum()*vol,'arms',arm.sum()*vol, 'per arm',arm.sum()*vol/4)
# slab sections: along arm u from motor, for quadrant ++ perpendicular slabs; area & depth
c=np.array([1,1])/np.sqrt(2); n=np.array([-1,1])/np.sqrt(2)
sel=(x>17)&(y>17)&(~pad)
U=(pts[sel,0]*c[0]+pts[sel,1]*c[1]); V=(pts[sel,0]*n[0]+pts[sel,1]*n[1]); Z=pts[sel,2]
for u0 in range(34,76,4):
    s=(U>=u0)&(U<u0+2)
    a=s.sum()*p**3/2
    print('r=%d..%d: area %.1f mm2, width extent %.1f, z max %.1f'%(u0,u0+2,a,np.ptp(V[s]) if s.any() else 0,Z[s].max() if s.any() else 0))
