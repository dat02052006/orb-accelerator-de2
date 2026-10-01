from pathlib import Path
import json,numpy as np
from reference import dense,nms,pack_key,v1_keys,border_point
out=Path(__file__).resolve().parent/'vectors';out.mkdir(exist_ok=True)
rng=np.random.default_rng(4319)
def hexfile(name,data,digits):
 (out/name).write_text(''.join(f'{int(v):0{digits}x}\n' for v in data) or '0\n')
frames=[]
def make(w,h,kind,threshold,mode,level=0):
 image=np.full((h,w),40,dtype=np.uint8)
 if kind=='flat': pass
 elif kind in ('corners','sparse'):
  image[:2,:2]=220;image[:2,-2:]=220;image[-2:,:2]=220;image[-2:,-2:]=220
  if kind=='sparse':
   for y in range(8,h-6,24):
    for x in range(8,w-6,24):image[y,x]=255 if level%2 else 0
 elif kind=='random':image=rng.integers(0,256,(h,w),dtype=np.uint8)
 elif kind in ('bright_point','dark_point'):
  for x,y in [(0,0),(1,0),(2,0),(w-1,0),(0,h-1),(w-1,h-1),(w//2,h//2)]: image[y,x]=255 if kind=='bright_point' else 0
 else:
  yy,xx=np.indices((h,w))
  if kind=='horizontal':mask=yy>h//2
  elif kind=='vertical':mask=xx>w//2
  elif kind=='diagonal_plus':mask=xx+yy>w//2
  elif kind=='diagonal_minus':mask=xx-yy>w//2
  elif kind=='checker':mask=((xx//2+yy//2)%2)>0
  elif kind=='L_edge':mask=(xx>w//2)|(yy>1)
  else:raise ValueError(kind)
  image=np.where(mask,220,40).astype(np.uint8)
 c,s,src=dense(image,threshold);keys=nms(c,s);v1=v1_keys(c,s)
 core=[k for k in keys if 4<=k[0]<=w-5 and 4<=k[1]<=h-5]
 assert core==v1
 if kind in ('flat','horizontal','vertical','diagonal_plus','diagonal_minus'):
  assert not np.any(c[src==1]),kind
 if kind in ('corners','sparse'):
  assert all((x,y,179) in keys for x,y in [(0,0),(w-1,0),(0,h-1),(w-1,h-1)])
 idx=len(frames)
 hexfile(f'f{idx}_pixels.hex',image.ravel(),2)
 hexfile(f'f{idx}_map.hex',((src.astype(np.uint16)<<9)|(c.astype(np.uint16)<<8)|s).ravel(),3)
 hexfile(f'f{idx}_keys.hex',[pack_key(*k) for k in keys],7)
 hexfile(f'f{idx}_v1keys.hex',[pack_key(*k) for k in v1],7)
 frames.append(dict(width=w,height=h,level=level,threshold=threshold,mode=mode,kind=kind,keys=len(keys),v1keys=len(v1)))
make(7,7,'flat',20,0)
make(9,9,'corners',20,2)
for kind in ['horizontal','vertical','diagonal_plus','diagonal_minus','L_edge','checker','bright_point','dark_point','random']:
 make(23,19,kind,20,1 if kind!='corners' else 2)
make(25,21,'random',0,2)
make(17,15,'random',255,1)
make(640,480,'sparse',20,0,0)
make(320,240,'sparse',20,1,1)
make(160,120,'sparse',20,2,2)
# Border-only unit vectors with arbitrary bytes in invalid outside-image positions.
windows=[];params=[];expected=[]
positions=[(0,0),(1,0),(2,0),(3,0),(6,0),(12,0),(0,1),(0,2),(0,3),(0,6),(12,6),(6,12),(0,12),(12,12),(2,3),(3,2),(10,3),(3,10)]
for threshold in (0,1,20,21,254,255):
 for x,y in positions:
  for kind in range(8):
   image=rng.integers(0,256,(13,13),dtype=np.uint8)
   if kind<4:
    image[:]=0;image[:2,:2]=[1,20,21,255][kind]
   win=[]
   for dy in range(-3,4):
    for dx in range(-3,4):
     yy,xx=y+dy,x+dx
     win.append(int(image[yy,xx]) if 0<=xx<13 and 0<=yy<13 else int(rng.integers(0,256)))
   windows.append(sum(p<<(8*i) for i,p in enumerate(win)))
   params.append((threshold<<19)|(x<<9)|y)
   c,s=border_point(image,x,y,threshold);expected.append((int(c)<<8)|s)
# Ideal single-edge angles, including mirrored one-sided neighborhoods.
for angle in range(0,180,5):
 yy,xx=np.indices((13,13));a=np.deg2rad(angle)
 image=np.where((xx-6)*np.cos(a)+(yy-6)*np.sin(a)>0,220,40).astype(np.uint8)
 for x,y in [(0,0),(6,0),(0,6),(12,6),(6,12),(12,12)]:
  win=[int(image[y+dy,x+dx]) if 0<=x+dx<13 and 0<=y+dy<13 else 167 for dy in range(-3,4) for dx in range(-3,4)]
  windows.append(sum(p<<(8*i) for i,p in enumerate(win)));params.append((20<<19)|(x<<9)|y)
  c,s=border_point(image,x,y,20);assert not c;expected.append((int(c)<<8)|s)
hexfile('border_windows.hex',windows,98);hexfile('border_params.hex',params,7);hexfile('border_expected.hex',expected,3)
# NMS maps put local maxima at every real edge/corner, ties, cross-source positions.
maps=[]
for kind in ('boundary','plateau','random'):
 w,h=11,9;c=np.zeros((h,w),bool);s=np.zeros((h,w),np.uint8)
 if kind=='boundary':
  for x,y,v in [(0,0,120),(w-1,0,121),(0,h-1,122),(w-1,h-1,123),(0,4,100),(w-1,4,101),(5,0,102),(5,h-1,103),(5,4,254)]:c[y,x]=1;s[y,x]=v
  c[3,2:4]=1;s[3,2]=60;s[3,3]=61 # FAST neighbor can suppress Border.
  c[6,2:4]=1;s[6,2]=71;s[6,3]=70 # Border neighbor can suppress FAST.
 elif kind=='plateau':c[:]=1;s[:]=99
 else:
  c=rng.integers(0,2,(h,w)).astype(bool);s=np.where(c,rng.integers(0,255,(h,w)),0).astype(np.uint8)
 keys=nms(c,s);idx=len(maps)
 hexfile(f'n{idx}_map.hex',((c.astype(np.uint16)<<8)|s).ravel(),3)
 hexfile(f'n{idx}_keys.hex',[pack_key(*k) for k in keys],7)
 maps.append(dict(width=w,height=h,kind=kind,keys=len(keys)))
params=[len(frames),len(windows),len(maps)]
for f in frames: params += [f[k] for k in ('width','height','level','threshold','mode','keys','v1keys')]
for m in maps:params += [m[k] for k in ('width','height','keys')]
hexfile('parameters.hex',params,8)
(out/'manifest.json').write_text(json.dumps(dict(frames=frames,border_cases=len(windows),nms_maps=maps),indent=2))
print('Reference vectors:',len(frames),'frames,',len(windows),'border cases,',len(maps),'NMS maps.')
