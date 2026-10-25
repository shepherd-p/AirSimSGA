"""Check protected originals and sealed evidence without modifying run evidence."""
import json,hashlib,sys,datetime
from pathlib import Path
root=Path(__file__).resolve().parents[1];take=root/sys.argv[1]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
protected=json.loads((root/'protected_inputs.json').read_text(encoding='utf-8-sig'))
checks={name:sha(name)==expected for name,expected in protected.items()}
original=Path(r'F:/EpicGamesEnv/ProjectAirSim_AG_Experiments/LANDING_VIDEO_RERUN_20261004')
old=json.loads((original/'quality_report.json').read_text(encoding='utf-8-sig'))
for name,key in [('moving_usv_landing_30fps.mp4','presentation'),('fixed_camera_overview_30fps.mp4','clean_fixed_camera_overview')]:
 if key in old:checks[str(original/name)]=sha(original/name)==old[key]['sha256']
assert all(checks.values()),checks
sealed=take/'experiment';hashes=json.loads((sealed/'evidence_sha256.json').read_text())
invalid=[name for name,digest in hashes.items() if sha(sealed/name)!=digest]
assert not invalid,invalid
sources=json.loads((sealed/'source_sha256.json').read_text());invalid=[name for name,digest in sources.items() if sha(sealed/'source_snapshot'/name)!=digest]
assert not invalid,invalid
report={'verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'protected_input_checks':checks,'sealed_evidence_files_verified':len(hashes),'source_snapshot_files_verified':len(sources),'sealed_run_evidence_not_modified':True}
(root/'preservation_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
cfg=json.loads((sealed/'config.json').read_text())
if cfg['provenance'].get('physics_source')!=str(root/'source'):
 clarification={'scope':'Additive metadata clarification; original run config and sealed evidence remain unchanged','historical_descriptive_field':cfg['provenance'].get('physics_source'),'actual_build_source':str(root/'source'),'authoritative_build_receipt':str(sealed/'source_snapshot/physics_build/build_receipt.json'),'actual_loaded_plugin_receipt':str(sealed/'source_snapshot/physics_build/runtime_receipt.json'),'explanation':'The trial inherited a descriptive baseline path in provenance.physics_source. The active model and snapshots were selected and SHA256-verified through physics_build_receipt, which points to the independent LandingWaterLab build.'}
 (root/'provenance_clarification.json').write_text(json.dumps(clarification,indent=2),encoding='utf-8')
 print('Saved additive provenance field clarification; sealed evidence unchanged')
print(json.dumps(report,indent=2))
