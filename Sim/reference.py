"""Behavioral image reference; no modeling of RTL pipeline/comparator trees."""
import numpy as np

RING=[(0,-3),(1,-3),(2,-2),(3,-1),(3,0),(3,1),(2,2),(1,3),
      (0,3),(-1,3),(-2,2),(-3,1),(-3,0),(-3,-1),(-2,-2),(-1,-3)]

def fast_point(image,x,y,threshold=0):
    c=int(image[y,x]); ring=[int(image[y+dy,x+dx]) for dx,dy in RING]
    def test(t):
        return any(all(ring[(s+k)%16]>c+t for k in range(9)) or
                   all(ring[(s+k)%16]<c-t for k in range(9)) for s in range(16))
    if not test(threshold): return False,0
    lo,hi=threshold,255
    while lo<hi:
        mid=(lo+hi+1)//2
        if test(mid): lo=mid
        else: hi=mid-1
    return True,lo

def neighborhood(image,x,y):
    h,w=image.shape
    xs=[x,x+1,x+2] if x<3 else [x,x-1,x-2] if x>w-4 else [x-1,x,x+1]
    ys=[y,y+1,y+2] if y<3 else [y,y-1,y-2] if y>h-4 else [y-1,y,y+1]
    assert min(xs)>=0 and max(xs)<w and min(ys)>=0 and max(ys)<h
    return image[np.ix_(ys,xs)].astype(np.int32)

def border_margin(q):
    # Integrated finite differences on each 2x2 cell, then orientation diversity.
    dx=np.diff(q,axis=1); dy=np.diff(q,axis=0)
    gx=dx[:-1,:]+dx[1:,:]; gy=dy[:,:-1]+dy[:,1:]
    dominance=np.abs(gx)-np.abs(gy)
    h=max(0,int(np.max(dominance))); v=max(0,int(np.max(-dominance)))
    return min(h,v)//2

def border_point(image,x,y,threshold):
    margin=border_margin(neighborhood(image,x,y))
    return margin>threshold, margin-1 if margin>threshold else 0

def dense(image,threshold):
    h,w=image.shape
    corners=np.zeros((h,w),dtype=bool); scores=np.zeros((h,w),dtype=np.uint8)
    source=np.ones((h,w),dtype=np.uint8)
    center=image[3:h-3,3:w-3].astype(np.int16)
    circle=np.stack([image[3+dy:h-3+dy,3+dx:w-3+dx].astype(np.int16) for dx,dy in RING])
    def predicate(t):
        b=circle>center+t; d=circle<center-t; result=np.zeros_like(center,dtype=bool)
        for s in range(16):
            ids=[(s+k)%16 for k in range(9)]
            result |= np.all(b[ids],axis=0)|np.all(d[ids],axis=0)
        return result
    mask=predicate(np.full_like(center,threshold)); lo=np.full_like(center,threshold); hi=np.full_like(center,255)
    for _ in range(8):
        mid=(lo+hi+1)//2; yes=predicate(mid)
        lo=np.where(yes,mid,lo); hi=np.where(yes,hi,mid-1)
    corners[3:h-3,3:w-3]=mask; scores[3:h-3,3:w-3]=np.where(mask,lo,0)
    source[3:h-3,3:w-3]=0
    for y in range(h):
        for x in range(w):
            if source[y,x]: corners[y,x],scores[y,x]=border_point(image,x,y,threshold)
    return corners,scores,source

def nms(corners,scores):
    h,w=scores.shape; keys=[]
    for y in range(h):
        for x in range(w):
            if not corners[y,x]: continue
            neighbors=[int(scores[yy,xx]) for yy in range(max(0,y-1),min(h,y+2))
                       for xx in range(max(0,x-1),min(w,x+2)) if (xx,yy)!=(x,y)]
            if all(int(scores[y,x])>s for s in neighbors): keys.append((x,y,int(scores[y,x])))
    return keys

def pack_key(x,y,score): return (x<<17)|(y<<8)|score

def v1_keys(corners,scores):
    h,w=scores.shape
    # Same FAST field, V1's missing border field is absent, not manufactured.
    return [(x+3,y+3,s) for x,y,s in nms(corners[3:h-3,3:w-3],scores[3:h-3,3:w-3])
            if 1<=x<w-7 and 1<=y<h-7]
