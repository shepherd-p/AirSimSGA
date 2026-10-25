"""Seal independent copies of selected successful studies and retained failed trials.

Uses SHA-256 for later integrity verification, not third-party attestation.
Existing destinations are never overwritten. Verification is read-only.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
from archive_plaza_evidence import checked_member,digest,verify

ROOT=Path(__file__).resolve().parent


def create(selection_file,destination):
    selection=json.loads(selection_file.read_text(encoding='utf-8'));destination=destination.resolve()
    if destination.exists():raise FileExistsError('Refusing to overwrite an existing archive: '+str(destination))
    sources={};runs={}
    def add(source,target):
        if not source.is_file():raise FileNotFoundError(source)
        name=Path(target).as_posix()
        if name in sources:raise ValueError('Duplicate archive member: '+name)
        sources[name]=source
    def tree(folder,target):
        if destination.is_relative_to(folder.resolve()):raise ValueError('Archive cannot be inside a source run')
        for source in sorted(folder.rglob('*')):
            if source.is_file() and '__pycache__' not in source.parts:add(source,target/source.relative_to(folder))
    for task,relative in selection['successful_runs'].items():
        folder=(ROOT/relative).resolve()
        if not folder.is_relative_to((ROOT/'outputs').resolve()):raise ValueError('Selected run lies outside outputs')
        manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
        audit=json.loads((folder/'study_results/audit.json').read_text(encoding='utf-8'))
        if manifest['status']!='completed' or audit['status']!='passed' or audit['run']!=folder.name or audit['task']!=task:
            raise ValueError('Successful-run selection has not passed its audit: '+relative)
        tree(folder,Path('data')/task/folder.name);runs[task]=str(Path('data')/task/folder.name)
    for trial in selection['failed_trials']:
        folder=(ROOT/trial['path']).resolve()
        if not folder.is_relative_to((ROOT/'outputs').resolve()):raise ValueError('Failed trial lies outside outputs')
        manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
        if manifest['status']=='completed':raise ValueError('Completed run incorrectly labeled failed')
        tree(folder,Path('failed_trials')/manifest['task']/folder.name)
    for source in sorted(ROOT.iterdir()):
        if source.is_file() and source.suffix in {'.py','.ps1','.md'}:add(source,Path('code/current')/source.name)
    for folder in ('ag','configs','tests'):
        for source in sorted((ROOT/folder).rglob('*')):
            if source.is_file() and source.suffix in {'.py','.json','.jsonc','.html','.md'}:
                add(source,Path('code/current')/source.relative_to(ROOT))
    for source in sorted((ROOT/'verification').iterdir()):
        if not source.is_file():continue
        if source.name in {'cooperative_archive_create.log','cooperative_archive_receipt.json'}:continue
        if source.name.startswith(('cooperative_','COOPERATIVE_','navigation_study_','tracking_study_','formation_study_')) or source.name in {
            'navigation_figures.log','tracking_figures.log','formation_figures.log','protected_files.json'}:
            add(source,Path('verification')/source.name)
    tree(ROOT/'delivery/cooperative_studies',Path('gallery'))
    add(ROOT/'verification/COOPERATIVE_REVIEWER_GUIDE.md',Path('README.md'))
    add(ROOT/'archive_plaza_evidence.py',Path('verify_archive.py'))
    for name,label in [
        ('446eeec6-9800-48a2-96ba-3b1d964f1f98','navigation_reference'),
        ('90f8c588-b08d-4a39-9abc-fe9dcbbdd70c','tracking_reference'),
        ('0371666c-47ad-4c2b-b713-3cfd0e96f89e','occlusion_reference'),
        ('686241f2-38a0-4474-bb72-570f6b23f73f','seven_platform_reference')]:
        source=Path('C:/Users/Stephen/AppData/Local/Temp')/f'codex-clipboard-{name}.png'
        if source.is_file():add(source,Path('references')/(label+'.png'))
    total=sum(source.stat().st_size for source in sources.values())
    destination.parent.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(destination.parent).free<total+512*1024*1024:raise OSError('Insufficient free space for independent copy')
    destination.mkdir();records=[]
    for i,(name,source) in enumerate(sorted(sources.items())):
        before=source.stat();target=checked_member(destination,name);target.parent.mkdir(parents=True,exist_ok=True)
        sha=hashlib.sha256();size=0
        with source.open('rb') as src,target.open('xb') as dst:
            for block in iter(lambda:src.read(4*1024*1024),b''):
                dst.write(block);sha.update(block);size+=len(block)
        after=source.stat()
        if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns) or size!=before.st_size:
            raise RuntimeError('Source changed while copying; archive remains unsealed: '+str(source))
        shutil.copystat(source,target);records.append({'path':name,'bytes':size,'sha256':sha.hexdigest()})
        if (i+1)%1000==0:print(f'Copied {i+1}/{len(sources)} files',flush=True)
    manifest={'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),
        'archive_scope':'Three measured cooperative studies, all original selected-run files, three failed debugging trials, figures and source snapshots',
        'successful_runs':runs,'failed_trial_policy':'Failed runs retain their original failed status and are not used for published result metrics.',
        'payload_file_count':len(records),'payload_bytes':sum(r['bytes'] for r in records),
        'snapshot_roles':{'data/*/*/source_snapshot':'Code at acquisition start',
                          'data/*/*/study_results/source_snapshot':'Code used for offline figures',
                          'code/current':'Code and configurations at archival time; not retroactively an acquisition snapshot',
                          'references':'User-provided comparison figures, not experimental measurements'},
        'external_prerequisites':'Existing UE/CityPark/ProjectAirSim binaries and Python environments are not bundled.',
        'integrity_limit':'Unsigned locally generated hashes detect later changes; they do not establish independent authenticity.',
        'files':records}
    path=destination/'MANIFEST.json';path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
    (destination/'MANIFEST.sha256').write_text(digest(path)+'  MANIFEST.json\n',encoding='ascii')
    return verify(destination,progress=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--selection',type=Path,default=ROOT/'verification/cooperative_selected_runs.json')
    p.add_argument('--output-dir',type=Path);p.add_argument('--verify',type=Path);p.add_argument('--receipt',type=Path)
    a=p.parse_args()
    if a.verify:
        if a.output_dir:p.error('--verify and --output-dir cannot be combined')
        result=verify(a.verify,progress=True)
    else:
        if not a.output_dir:p.error('--output-dir is required when creating an archive')
        result=create(a.selection,a.output_dir)
    if a.receipt:
        destination=a.verify or a.output_dir
        if a.receipt.resolve().is_relative_to(destination.resolve()):raise ValueError('Write the receipt outside the sealed archive')
        a.receipt.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,indent=2,ensure_ascii=False),flush=True)
