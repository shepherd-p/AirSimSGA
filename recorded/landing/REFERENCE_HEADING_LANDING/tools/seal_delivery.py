"""Hash delivery files and link the already sealed experiment data manifest."""
import hashlib,json,datetime
from pathlib import Path
root=Path(__file__).resolve().parents[1];take=root/'results/landing_take_01'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for c in iter(lambda:f.read(1024*1024),b''):h.update(c)
 return h.hexdigest()
paths=[p for p in root.iterdir() if p.is_file() and p.suffix in {'.json','.html','.txt','.md','.jpg','.mp4'} and p.name!='delivery_sha256.json']
paths+=list((root/'tools').glob('*.py'))+list((root/'tools').glob('*.ps1'))
paths+=[p for p in (root/'CityPark2/Content/SGAWaterLab').glob('*.uasset') if p.name!='lake_grid_yup.uasset']
paths+=[take/p for p in ['experiment_result.json','presentation_manifest.json','presentation.mp4','experiment/evidence_sha256.json','experiment/receipt.json','camera_originals/recording_manifest.json']]
paths+=list((take/'camera_originals').glob('*/capture_metadata.jsonl.gz'))+list((take/'camera_originals').glob('*/video_frame_index.csv'))
report={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'algorithm':'SHA256','files':{p.relative_to(root).as_posix():sha(p) for p in paths},'sealed_experiment_manifest':'results/landing_take_01/experiment/evidence_sha256.json','preservation_report':'preservation_report.json','result_card':'Two-second explicit static summary; actual durations are in quality_report.json'}
(root/'delivery_sha256.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Delivery hashes saved:',len(paths))
