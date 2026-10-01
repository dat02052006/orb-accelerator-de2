from pathlib import Path
import hashlib,json,re
root=Path(__file__).resolve().parents[1]
unchanged=['orb_fast9.v','orb_fast_score.v','orb_score_math.v','orb_line_buffer.v','orb_row_ram.v','orb_nms_row_ram.v']
for name in unchanged:assert (root/'rtl'/name).read_bytes()==(root/'regression/v1/rtl'/name).read_bytes(),name
for p in (root/'rtl').glob('*.v'):
 s=re.sub(r'/\*.*?\*/|//[^\n]*','',p.read_text(),flags=re.S)
 assert not re.search(r'\b(for|while|repeat|generate|genvar|integer|logic|always_ff|always_comb)\b',s),p
original=root.parent/'orb_fast_nms_v1'
if original.exists():
 for name,digest in json.loads((root/'v1_fingerprint.json').read_text()).items():
  p=original/name;assert p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==digest,p
 print('ALL ORIGINAL V1 PRESERVATION CHECKS PASSED')
print('ALL RTL STYLE AND UNCHANGED FAST CORE CHECKS PASSED')
