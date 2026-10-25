from pathlib import Path
import json,hashlib
root=Path(__file__).resolve().parents[1]
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
reports=[]
for folder,manifest in [(root.parent,'preserved_previous_delivery_sha256.json'),(root.parent/'FORWARD_BOW_LANDING','preserved_second_delivery_sha256.json')]:
 expected=json.loads((root/manifest).read_text(encoding='utf-8-sig'))['files']
 checks={name:sha(folder/name)==value for name,value in expected.items()}
 assert all(checks.values()),[name for name,value in checks.items() if not value]
 reports.append({'directory':str(folder),'files_checked':len(checks),'all_preserved':True,'checks':checks})
(root/'previous_delivery_preservation.json').write_text(json.dumps({'all_preserved':True,'deliveries':reports,'parent_runtime_receipt_restored':True},indent=2),encoding='utf-8')
print('Both previous water-lab deliveries preserved:',sum(r['files_checked'] for r in reports),'files')
