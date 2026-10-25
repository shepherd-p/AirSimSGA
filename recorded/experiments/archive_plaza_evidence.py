"""Create an independent evidence copy and verify every payload file with SHA-256.

Manifests detect later changes; they are not signed authenticity attestations.
Creation refuses to overwrite an existing archive. Verification is read-only.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parent
META={'MANIFEST.json','MANIFEST.sha256'}


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def checked_member(root,name):
    p=(root/name).resolve()
    if not p.is_relative_to(root.resolve()):raise ValueError('Manifest path escapes archive: '+name)
    return p


def verify(root,progress=False):
    root=root.resolve();manifest_path=root/'MANIFEST.json'
    expected=(root/'MANIFEST.sha256').read_text(encoding='ascii').split()[0]
    if digest(manifest_path)!=expected:raise ValueError('MANIFEST.json checksum mismatch')
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    listed=set();total=0
    for i,record in enumerate(manifest['files']):
        name=record['path']
        if name in listed or name in META:raise ValueError('Duplicate or reserved manifest member: '+name)
        listed.add(name);p=checked_member(root,name)
        if not p.is_file() or p.stat().st_size!=record['bytes'] or digest(p)!=record['sha256']:
            raise ValueError('Missing or changed archive member: '+name)
        total+=record['bytes']
        if progress and i%1000==999:print(f'Verified {i+1}/{len(manifest["files"])} files',flush=True)
    actual={p.relative_to(root).as_posix() for p in root.rglob('*') if p.is_file()}-META
    if actual!=listed:raise ValueError('Unlisted archive files: '+repr(sorted(actual-listed)[:10]))
    if total!=manifest['payload_bytes'] or len(listed)!=manifest['payload_file_count']:
        raise ValueError('Manifest totals disagree')
    return {'status':'passed','archive':str(root),'verified_payload_files':len(listed),
            'payload_bytes':total,'manifest_sha256':expected,
            'meaning':'Content integrity at verification time; not third-party authenticity attestation'}


def create(run,photo,destination):
    run=run.resolve();photo=photo.resolve();destination=destination.resolve()
    if destination.exists():raise FileExistsError('Archive already exists; choose a new destination')
    if destination.is_relative_to(run) or destination.is_relative_to(photo):
        raise ValueError('Archive must be separate from original data')
    sources={}
    def add(source,target):
        if not source.is_file():raise FileNotFoundError(source)
        name=Path(target).as_posix()
        if name in sources:raise ValueError('Duplicate archive destination: '+name)
        sources[name]=source
    for folder,target in [(run,Path('data/mapping')/run.name),(photo,Path('data/scene_overview')/photo.name)]:
        for source in sorted(folder.rglob('*')):
            if source.is_file():add(source,target/source.relative_to(folder))
    for source in sorted(ROOT.iterdir()):
        if source.is_file() and source.suffix in {'.py','.ps1','.md'}:add(source,Path('code/current')/source.name)
    for sub in ('ag','configs','tests'):
        for source in sorted((ROOT/sub).rglob('*')):
            if source.is_file() and source.suffix in {'.py','.html','.json','.jsonc','.md'}:
                add(source,Path('code/current')/source.relative_to(ROOT))
    for source in sorted((ROOT/'verification').iterdir()):
        if source.is_file() and source.name not in {'client.lock','ue_session.log'} and source.suffix in {'.py','.json','.txt','.md','.log','.png','.jpg'}:
            add(source,Path('verification')/source.name)
    add(ROOT/'verification/REVIEWER_EVIDENCE_GUIDE.md',Path('README.md'))
    for name,label in [
        ('codex-clipboard-c93da89c-d090-4cac-a033-d3a0f003ea19.png','user_previous_pointcloud.png'),
        ('codex-clipboard-b702cd2d-0ad6-4e60-a50b-51c6a3969770.png','user_reference_pointcloud_crop.png'),
        ('codex-clipboard-b7bd5647-db15-49cb-9b83-21872db327dd.png','user_reference_scene_crop.png')]:
        source=Path('C:/Users/Stephen/AppData/Local/Temp')/name
        if source.is_file():add(source,Path('references')/label)
    required=['plaza_records_audit.json','plaza_lidar_replay_audit.json']
    for filename in required:
        record=json.loads((ROOT/'verification'/filename).read_text(encoding='utf-8'))
        if record['status']!='passed':raise ValueError('Audit did not pass: '+filename)
        if filename=='plaza_records_audit.json' and (record['mapping_run']!=run.name or record['scene_session']!=photo.name):
            raise ValueError('Audit belongs to a different mapping or scene session')
    payload_bytes=sum(source.stat().st_size for source in sources.values())
    destination.parent.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(destination.parent).free<payload_bytes+256*1024*1024:
        raise OSError('Insufficient free space for independent evidence copy')
    destination.mkdir()
    records=[]
    for i,(name,source) in enumerate(sorted(sources.items())):
        before=source.stat();target=checked_member(destination,name);target.parent.mkdir(parents=True,exist_ok=True)
        h=hashlib.sha256();size=0
        with source.open('rb') as src,target.open('xb') as dst:
            for block in iter(lambda:src.read(4*1024*1024),b''):
                dst.write(block);h.update(block);size+=len(block)
        after=source.stat()
        if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns) or size!=before.st_size:
            raise RuntimeError('Source changed during copy; archive remains unsealed: '+str(source))
        shutil.copystat(source,target)
        records.append({'path':name,'bytes':size,'sha256':h.hexdigest()})
        if i%1000==999:print(f'Copied {i+1}/{len(sources)} files',flush=True)
    manifest={
        'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),
        'archive_scope':'Current plaza mapping run, accepted figures, and separate illustrative scene capture',
        'original_mapping_directory':str(run),'original_scene_directory':str(photo),
        'payload_file_count':len(records),'payload_bytes':sum(r['bytes'] for r in records),
        'snapshot_roles':{
            'data/mapping/'+run.name+'/source_snapshot':'Existing acquisition-start source snapshot',
            'data/mapping/'+run.name+'/plaza_results/source_snapshot':'Existing reconstruction-stage source snapshot',
            'code/current':'Source and configuration snapshot at archival time; not retroactively an acquisition snapshot',
            'references':'User-provided comparison screenshots; not experiment measurements'},
        'external_prerequisites':'UE, CityPark assets, ProjectAirSim SDK/binaries, and installed Python environments are not bundled',
        'excluded_live_files':['verification/client.lock','verification/ue_session.log'],
        'integrity_limit':'Unsigned locally generated hashes detect later changes; no independent attestation of prior history',
        'files':records,
    }
    path=destination/'MANIFEST.json';path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
    (destination/'MANIFEST.sha256').write_text(digest(path)+'  MANIFEST.json\n',encoding='ascii')
    return verify(destination,progress=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--verify',type=Path);p.add_argument('--mapping-dir',type=Path)
    p.add_argument('--scene-dir',type=Path);p.add_argument('--output-dir',type=Path)
    a=p.parse_args()
    if a.verify:
        if any((a.mapping_dir,a.scene_dir,a.output_dir)):p.error('--verify cannot be combined with creation arguments')
        result=verify(a.verify,progress=True)
    else:
        if not all((a.mapping_dir,a.scene_dir,a.output_dir)):p.error('Creation requires --mapping-dir, --scene-dir, and --output-dir')
        result=create(a.mapping_dir,a.scene_dir,a.output_dir)
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()
