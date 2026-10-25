"""Copy a complete diagnostic suite and verify every copied byte by SHA256."""
import argparse
import json
import shutil
from datetime import datetime,timezone
from pathlib import Path
from ag.landing_evidence import digest
from ag.storage import ROOT,write_json


def archive(source,destination):
    source=source.resolve();destination=destination.resolve()
    if not source.is_relative_to((ROOT/'outputs/landing').resolve()):
        raise ValueError('Source must be an existing landing suite inside this kit')
    if not destination.is_relative_to((ROOT/'archives').resolve()) or destination.exists():
        raise ValueError('Use a new directory under this kit archives; existing archives are never overwritten')
    if not (source/'summary.json').is_file() or not (source/'audit_report.json').is_file():
        raise ValueError('Finish trials and audit before archiving')
    summary=json.loads((source/'summary.json').read_text(encoding='utf-8'))
    audit=json.loads((source/'audit_report.json').read_text(encoding='utf-8'))
    if not audit.get('audit_passed'):
        raise ValueError('Evidence audit must pass before a verified archive is created')
    shutil.copytree(source,destination/'suite')
    docs=destination/'documentation';docs.mkdir(parents=True)
    for name in ['MOVING_DECK_EXPERIMENT.md','MOVING_DECK_PHYSICS_FINDINGS.md','TASK_REGION_LAYOUT.md']:
        shutil.copy2(ROOT/name,docs/name)
    notice=ROOT/'verification/LANDING_V1_VALIDITY_NOTICE.md'
    if notice.exists():shutil.copy2(notice,docs/notice.name)
    verified=0
    for original in source.rglob('*'):
        if original.is_file():
            target=destination/'suite'/original.relative_to(source)
            if digest(original)!=digest(target):raise RuntimeError('Archive byte mismatch: '+str(target))
            verified+=1
    inventory={p.relative_to(destination).as_posix():{'sha256':digest(p),'bytes':p.stat().st_size}
               for p in sorted(destination.rglob('*')) if p.is_file()}
    write_json(destination/'archive_manifest.json',inventory)
    receipt={'created_utc':datetime.now(timezone.utc).isoformat(),'source':str(source),
       'archive':str(destination),'verified_source_files':verified,'files':len(inventory),
       'bytes':sum(v['bytes'] for v in inventory.values()),
       'manifest_sha256':digest(destination/'archive_manifest.json'),
       'experimental_status':{'trials':len(summary['trials']),
           'successes':sum(bool(t.get('success')) for t in summary['trials']),
           'failures':sum(not bool(t.get('success')) for t in summary['trials']),
           'audit_passed':True,
           'verified_motor_off_landings':sum(bool(t.get('verified_motor_off_landing')) for t in audit['trials']),
           'note':'Statuses come from recorded trial manifests; audit pass does not imply landing success'}}
    write_json(destination/'archive_receipt.json',receipt)
    print(json.dumps(receipt,indent=2));return receipt


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input-dir',type=Path,required=True);p.add_argument('--destination',type=Path,required=True)
    a=p.parse_args();archive(a.input_dir,a.destination)
