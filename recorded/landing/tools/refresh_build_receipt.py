from pathlib import Path
import hashlib,json,datetime
root=Path(r'F:/EpicGamesEnv/LandingWaterLab');p=root/'build_receipt.json';old=json.loads(p.read_text(encoding='utf-8'))
(root/'results/build_receipt_before_water_position_fix.json').write_bytes(p.read_bytes())
for group in ('binaries','sources'):
 for name in old[group]:
  old[group][name]=hashlib.sha256(Path(name).read_bytes()).hexdigest()
old['created_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
old['water_position_fix']='Set mobility before runtime lake location; disable planar reflection in improved presentation; reserved HLSL variable renamed in foam material'
p.write_text(json.dumps(old,indent=2)+'\n',encoding='utf-8')
print('UPDATED_BUILD_RECEIPT',next(iter(old['binaries'].values())))
