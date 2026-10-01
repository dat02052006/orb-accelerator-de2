"""Independent behavioral reference. Loops here are software/test code, not RTL.
Score oracle uses binary search over the strict FAST predicate, not RTL min trees.
Run from this directory: python make_vectors.py
"""
from pathlib import Path
import random
import json
import numpy as np

OUT = Path(__file__).resolve().parent / 'vectors'
OUT.mkdir(exist_ok=True)
RING = [(0,-3),(1,-3),(2,-2),(3,-1),(3,0),(3,1),(2,2),(1,3),
        (0,3),(-1,3),(-2,2),(-3,1),(-3,0),(-3,-1),(-2,-2),(-1,-3)]

def fast(ring, center, threshold):
    bright = [p > center + threshold for p in ring]
    dark = [p < center - threshold for p in ring]
    return any(all(bright[(s+k)%16] for k in range(9)) or
               all(dark[(s+k)%16] for k in range(9)) for s in range(16))

def score(ring, center, threshold):
    if not fast(ring, center, threshold):
        return 0, 0
    lo, hi = threshold, 255
    while lo < hi:
        mid = (lo+hi+1)//2
        if fast(ring, center, mid): lo = mid
        else: hi = mid-1
    return 1, lo

def hexfile(name, values, digits):
    (OUT/name).write_text(''.join(f'{int(v):0{digits}x}\n' for v in values) or '0\n')

def keypoints(scores, corners):
    h,w = scores.shape
    result=[]
    for y in range(1,h-1):
        for x in range(1,w-1):
            v=int(scores[y,x])
            if corners[y,x] and all(v > int(scores[y+dy,x+dx])
                for dy in (-1,0,1) for dx in (-1,0,1) if dx or dy):
                result.append((x+3,y+3,v))
    return result

def pack_keypoint(x,y,s): return (x<<17)|(y<<8)|s

rng=random.Random(0xF9A57)
cases=[]
# Every circular start, both polarities, strict equality and exactly 8/9 pixels.
for start in range(16):
    for polarity in (-1,1):
        for length in (8,9):
            for delta in (19,20,21,80):
                r=[128]*16
                for k in range(length): r[(start+k)%16]=128+polarity*delta
                cases.append((128,20,r))
for center in (0,1,127,128,254,255):
    for threshold in (0,1,20,254,255):
        cases += [(center,threshold,[v]*16) for v in (0,center,255)]
# Score zero but corner=true at threshold=0; extreme scores 254; mixed signs.
cases += [(100,0,[101]*16),(100,0,[99]*16),(0,254,[255]*16),
          (255,254,[0]*16),(128,0,[127,129]*8)]
for _ in range(1200):
    center=rng.randrange(256); threshold=rng.choice((0,1,20,40,100,254,255))
    r=[rng.randrange(256) for _ in range(16)]
    if rng.randrange(2):
        start=rng.randrange(16); polarity=rng.choice((-1,1)); value=rng.randrange(256)
        for k in range(rng.randrange(8,17)): r[(start+k)%16]=value
    cases.append((center,threshold,r))
windows=[]; expected=[]; thresholds=[]
for center,threshold,r in cases:
    pixels=[center]*49
    for p,(dx,dy) in zip(r,RING): pixels[(dy+3)*7+(dx+3)]=p
    windows.append(sum(p<<(8*i) for i,p in enumerate(pixels)))
    corner,value=score(r,center,threshold)
    expected.append((corner<<8)|value); thresholds.append(threshold)
hexfile('unit_windows.hex',windows,98)
hexfile('unit_thresholds.hex',thresholds,2)
hexfile('unit_expected.hex',expected,3)

frames=[]
def save_frame(w,h,level,threshold,kind,mode):
    idx=len(frames)
    if kind=='random':
        image=np.array([rng.randrange(256) for _ in range(w*h)],dtype=np.uint8).reshape(h,w)
    else:
        image=np.full((h,w),100,dtype=np.uint8)
        for y in range(4,h-4,24):
            for x in range(4,w-4,24): image[y,x]=255 if (x+y+level)%2 else 0
    dense=np.zeros((h-6,w-6),dtype=np.uint16)
    # Vectorized predicate plus threshold binary search: still a behavioral oracle.
    center=image[3:h-3,3:w-3].astype(np.int16)
    circle=np.stack([image[3+dy:h-3+dy,3+dx:w-3+dx].astype(np.int16) for dx,dy in RING])
    def predicate(t):
        bright=circle > center+t; dark=circle < center-t
        answer=np.zeros_like(center,dtype=bool)
        for start in range(16):
            ids=[(start+k)%16 for k in range(9)]
            answer |= np.all(bright[ids],axis=0) | np.all(dark[ids],axis=0)
        return answer
    corners=predicate(np.full_like(center,threshold))
    lo=np.full_like(center,threshold); hi=np.full_like(center,255)
    for _ in range(8):
        mid=(lo+hi+1)//2; yes=predicate(mid)
        lo=np.where(yes,mid,lo); hi=np.where(yes,hi,mid-1)
    scores=np.where(corners,lo,0).astype(np.uint8)
    dense=(corners.astype(np.uint16)<<8)|scores
    result=keypoints(scores,corners)
    hexfile(f'frame_{idx}_pixels.hex',image.ravel(),2)
    hexfile(f'frame_{idx}_scores.hex',dense.ravel(),3)
    hexfile(f'frame_{idx}_keypoints.hex',[pack_keypoint(*p) for p in result],7)
    frames.append(dict(width=w,height=h,level=level,threshold=threshold,mode=mode,
                       scores=(w-6)*(h-6),keypoints=len(result),kind=kind))

save_frame(7,7,0,20,'sparse',0)
save_frame(9,9,1,20,'sparse',2)
save_frame(31,27,2,20,'random',1)
save_frame(33,29,0,0,'random',2)
save_frame(17,15,1,255,'random',1)
save_frame(640,480,0,20,'sparse',0)
save_frame(320,240,1,20,'sparse',1)
save_frame(160,120,2,20,'sparse',2)

# Independent dense score maps exercise plateaus/ties without relying on image FAST.
maps=[]
for kind in ('ties','zero','random'):
    w,h=19,15
    scores=np.zeros((h-6,w-6),dtype=np.uint8)
    corners=np.zeros_like(scores,dtype=np.uint8)
    if kind=='ties':
        scores[2,2]=80; scores[2,3]=80; corners[2,2:4]=1
        scores[5,7]=100; corners[5,7]=1
        scores[0,10]=200; corners[0,10]=1 # Discard NMS border.
    elif kind=='zero': corners[:]=1
    else:
        scores[:]=np.array([rng.randrange(255) for _ in range(scores.size)]).reshape(scores.shape)
        corners[:]=np.array([rng.randrange(2) for _ in range(scores.size)]).reshape(scores.shape)
        scores[corners==0]=0
    result=keypoints(scores,corners); i=len(maps)
    hexfile(f'nms_{i}_scores.hex',((corners.astype(np.uint16)<<8)|scores).ravel(),3)
    hexfile(f'nms_{i}_keys.hex',[pack_keypoint(*p) for p in result],7)
    maps.append(dict(width=w,height=h,scores=scores.size,keypoints=len(result),kind=kind))

params=[len(cases),len(frames),len(maps)]
for f in frames:
    params += [f[k] for k in ('width','height','level','threshold','mode','scores','keypoints')]
for m in maps: params += [m[k] for k in ('width','height','scores','keypoints')]
hexfile('parameters.hex',params,8)
(OUT/'manifest.json').write_text(json.dumps(dict(unit_cases=len(cases),frames=frames,nms_maps=maps),indent=2))
print(f'Generated {len(cases)} FAST/score cases, {len(frames)} image frames, {len(maps)} NMS maps.')
