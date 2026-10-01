from pathlib import Path
import json
import numpy as np
from reference import fast_point,border_point,border_margin,neighborhood

out=Path(__file__).resolve().parent/'calibration'; out.mkdir(exist_ok=True)
rng=np.random.default_rng(9127)
patterns=['flat','horizontal','vertical','diagonal_plus','diagonal_minus',
          'L_corner','bright_corner','dark_corner','checker','isolated_bright','isolated_dark']
contrasts=[1,5,10,20,21,40,80,120,200,255]
rows=[]
def pattern(name,xx,yy):
    if name=='flat': return np.zeros_like(xx,dtype=bool)
    if name=='horizontal': return yy>1
    if name=='vertical': return xx>1
    if name=='diagonal_plus': return xx+yy>1
    if name=='diagonal_minus': return xx-yy>1
    if name in ('L_corner','bright_corner','dark_corner'): return (xx>1)|(yy>1)
    if name=='checker': return (xx>=1)^(yy>=1)
    if name=='isolated_bright': return (xx==0)&(yy==0)
    if name=='isolated_dark': return ~((xx==0)&(yy==0))
for name in patterns:
    for contrast in contrasts:
        for noise in (0,2):
            a=[]; b=[]
            for sample in range(16 if noise else 1):
                yy,xx=np.indices((13,13))
                interior=pattern(name,xx-6,yy-6).astype(int)*contrast
                border=pattern(name,xx,yy).astype(int)*contrast
                if name=='bright_corner': interior=contrast-interior; border=contrast-border
                if noise:
                    interior=np.clip(interior+rng.integers(-noise,noise+1,interior.shape),0,255)
                    border=np.clip(border+rng.integers(-noise,noise+1,border.shape),0,255)
                _,fs=fast_point(interior,6,6,0)
                _,bs=border_point(border,0,0,0)
                raw=border_margin(neighborhood(border,0,0))
                a.append(fs); b.append(bs)
                if not noise and name in ('L_corner','bright_corner','dark_corner'):
                    assert raw==contrast and fs==bs==contrast-1
                if not noise and name in ('flat','horizontal','vertical','diagonal_plus','diagonal_minus'):
                    assert raw==0
            rows.append(dict(pattern=name,contrast=contrast,noise=noise,
                fast_score_mean=float(np.mean(a)),border_score_mean=float(np.mean(b)),
                fast_score_min=min(a),fast_score_max=max(a),border_score_min=min(b),border_score_max=max(b)))
(out/'results.json').write_text(json.dumps(rows,indent=2))
text='''# Calibration trước RTL\n\nCoefficient bổ sung: **1**. Chia 2 trong border margin xuất phát từ hai hiệu intensity được cộng trong gx/gy, không phải hệ số fit tùy ý.\n\nL/bright/dark corner có turn tại offset (1,1): ở toàn bộ contrast 1..255 đã thử, border margin=C và cả hai score=C-1 khi không noise. Flat, horizontal, vertical và diagonal ±45° đều border margin=0. Checker và isolated point có đáp ứng khác FAST: không thể ép phân bố của mọi loại hình học trùng nhau bằng một coefficient. Noise ±2 được ghi lại theo range/mean trong JSON.\n\n| Pattern | Contrast | Noise | Mean FAST score | Mean Border score |\n|---|---:|---:|---:|---:|\n'''
for r in rows:
    if r['contrast'] in (20,80,200):
        text+=f"| {r['pattern']} | {r['contrast']} | {r['noise']} | {r['fast_score_mean']:.2f} | {r['border_score_mean']:.2f} |\n"
text+='\nĐây là paired calibration trên các patch chuyển từ interior đến góc ảnh. Neighborhood một phía có thể lệch vị trí tới hai pixel và bỏ isolated point; không tuyên bố tương đương FAST hay false-positive bằng 0 ở mọi góc/ảnh tự nhiên.\n'
(out/'REPORT.md').write_text(text)
print('ALL SCORE NORMALIZATION TESTS PASSED: paired clean/noisy patterns; identity intensity scale.')
# Paired calibration at four edges and all four image corners, mirrored inward.
placements=[('top',6,0),('bottom',6,12),('left',0,6),('right',12,6),
            ('top_left',0,0),('top_right',12,0),('bottom_left',0,12),('bottom_right',12,12)]
placed=[]
for pose,x0,y0 in placements:
    sx=-1 if x0==12 else 1; sy=-1 if y0==12 else 1
    tx=1 if x0 in (0,12) else 0; ty=1 if y0 in (0,12) else 0
    yy,xx=np.indices((13,13))
    for name in patterns:
        for contrast in contrasts:
            for noise in (0,2):
                aa=[];bb=[]
                for sample in range(8 if noise else 1):
                    ix=sx*(xx-6);iy=sy*(yy-6);bx=sx*(xx-x0);by=sy*(yy-y0)
                    if name in ('L_corner','bright_corner','dark_corner'):
                        fi=(ix>tx)|(iy>ty); fb=(bx>tx)|(by>ty)
                    else: fi=pattern(name,ix,iy);fb=pattern(name,bx,by)
                    interior=fi.astype(int)*contrast;border=fb.astype(int)*contrast
                    if name=='bright_corner':interior=contrast-interior;border=contrast-border
                    if noise:
                        interior=np.clip(interior+rng.integers(-noise,noise+1,interior.shape),0,255)
                        border=np.clip(border+rng.integers(-noise,noise+1,border.shape),0,255)
                    _,fs=fast_point(interior,6,6,0);_,bs=border_point(border,x0,y0,0)
                    aa.append(fs);bb.append(bs)
                    if not noise and name in ('L_corner','bright_corner','dark_corner'):assert fs==bs==contrast-1
                    if not noise and name in ('flat','horizontal','vertical','diagonal_plus','diagonal_minus'):assert border_margin(neighborhood(border,x0,y0))==0
                placed.append(dict(placement=pose,pattern=name,contrast=contrast,noise=noise,
                    fast_mean=float(np.mean(aa)),border_mean=float(np.mean(bb)),fast_range=[min(aa),max(aa)],border_range=[min(bb),max(bb)]))
(out/'placement_results.json').write_text(json.dumps(placed,indent=2))
with (out/'REPORT.md').open('a') as f:
    f.write('\n## Kiểm tra thêm bốn edge và bốn corner\n\nĐã chạy toàn bộ 11 pattern, 10 contrast level và hai mức noise trên tám vị trí. Các L-corner được đặt turn ở pixel nhìn thấy trong neighborhood thật; clean L/bright/dark score=C-1 tại mọi vị trí. Kết quả từng distribution nằm trong placement_results.json. Không fit coefficient sau khi quan sát một case đơn lẻ.\n')
print('ALL EIGHT-PLACEMENT CALIBRATION TESTS PASSED')

angle_rows=[]
yy,xx=np.indices((23,23))
for angle in range(0,180,5):
 a=np.deg2rad(angle);image=np.where((xx-11)*np.cos(a)+(yy-11)*np.sin(a)>0,220,40).astype(np.uint8)
 count=sum(border_point(image,x,y,20)[0] for y in range(23) for x in range(23) if x<3 or x>19 or y<3 or y>19)
 assert count==0
 angle_rows.append(dict(angle_degrees=angle,border_candidates=count))
(out/'straight_angle_results.json').write_text(json.dumps(angle_rows,indent=2))
with (out/'REPORT.md').open('a') as f:f.write('\n36 ideal single-edge orientations (0..175 degrees, step 5), contrast 180, threshold 20: zero border candidates in this synthetic experiment. Not a guarantee on textured/noisy real images.\n')
print('ALL IDEAL EDGE ANGLE TESTS PASSED: 36 orientations')
