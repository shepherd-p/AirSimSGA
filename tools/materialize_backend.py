"""Assemble a pinned upstream commit and one extension profile in a NEW directory.
Reads the upstream Git object database; never copies working-tree changes or build caches.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def command(repo, *args):
    return subprocess.check_output(['git','-c',f'safe.directory={repo.as_posix()}','-C',str(repo),*args])


def verify_profile(upstream, profile):
    metadata = json.loads((ROOT/f'native/profiles/{profile}/profile.json').read_text(encoding='utf-8'))
    commit = metadata['base_commit']
    actual = command(upstream,'rev-parse',commit+'^{commit}').decode().strip()
    if actual != commit:
        raise ValueError('Pinned upstream commit is unavailable')
    for record in metadata['files']:
        relative = PurePosixPath(record['path'])
        if relative.is_absolute() or '..' in relative.parts or '\\' in record['path']:
            raise ValueError('Unsafe profile member')
        data = (ROOT/f'native/profiles/{profile}/overlay'/relative).read_bytes()
        if hashlib.sha256(data).hexdigest()!=record['extension_sha256']:
            raise ValueError('Overlay changed: '+record['path'])
        if record['upstream_sha256'] is not None:
            old=command(upstream,'show',commit+':'+record['path']).replace(b'\r\n',b'\n')
            if hashlib.sha256(old).hexdigest()!=record['upstream_sha256']:
                raise ValueError('Baseline mismatch: '+record['path'])
    return metadata


def materialize(upstream, destination, profile):
    upstream=Path(upstream).resolve();destination=Path(destination).resolve()
    if destination.exists():
        raise FileExistsError('Destination must not exist; existing builds are never overwritten')
    if destination.is_relative_to(upstream) or destination.is_relative_to(ROOT):
        raise ValueError('Choose a NEW sibling directory outside upstream and this release')
    metadata=verify_profile(upstream,profile)
    data=command(upstream,'archive','--format=tar',metadata['base_commit'])
    # Validate every member before creating destination. Git archives contain no build caches.
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        members=archive.getmembers()
        for member in members:
            name=PurePosixPath(member.name)
            if name.is_absolute() or '..' in name.parts or '\\' in member.name or not (member.isfile() or member.isdir()):
                raise ValueError('Unsupported or unsafe Git archive member: '+member.name)
        # Upstream has docs images whose names differ only by case. Windows cannot
        # represent both; omit duplicates only for non-runtime documentation assets.
        skipped=[]; seen={}; selected=[]
        for member in members:
            key=member.name.casefold() if os.name=='nt' else member.name
            if key in seen:
                previous=seen[key]
                if member.isdir() and previous.isdir():continue
                if not member.name.startswith('docs/') or Path(member.name).suffix.lower() not in ['.png','.jpg','.jpeg','.svg','.gif']:
                    raise ValueError('Case-colliding runtime/source member: '+member.name)
                skipped.append({'omitted':member.name,'retained':previous.name,'reason':'Windows filename case collision in documentation images'})
                continue
            seen[key]=member;selected.append(member)
        destination.mkdir(parents=True)
        for member in selected:
            target=destination/member.name
            if member.isdir():target.mkdir(parents=True,exist_ok=True)
            else:
                target.parent.mkdir(parents=True,exist_ok=True)
                with archive.extractfile(member) as source,target.open('xb') as output:
                    while True:
                        block=source.read(1024*1024)
                        if not block:break
                        output.write(block)
                if member.mode & 0o111:target.chmod(target.stat().st_mode | 0o111)
    for record in metadata['files']:
        data=(ROOT/f'native/profiles/{profile}/overlay'/record['path']).read_bytes()
        target=destination/record['path'];target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    (destination/'AIRSIMSGA_BUILD_PROFILE.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    (destination/'AIRSIMSGA_SOURCE_ASSEMBLY.json').write_text(json.dumps({'documentation_case_collisions':skipped,'base_commit':metadata['base_commit']},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'destination':str(destination),'profile':profile,'base_commit':metadata['base_commit'],
                      'overlay_files':len(metadata['files']),'documentation_case_collisions':skipped,'scope':'source assembly only; not compiled or runtime-verified'},indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--upstream',type=Path,required=True)
    p.add_argument('--profile',choices=['main','moving-deck','recorded-landing'],required=True)
    p.add_argument('--destination',type=Path)
    p.add_argument('--check-only',action='store_true')
    a=p.parse_args()
    if a.check_only:
        d=verify_profile(a.upstream.resolve(),a.profile)
        print(json.dumps({'status':'passed','profile':a.profile,'overlay_files':len(d['files']),'base_commit':d['base_commit']},indent=2))
    else:
        if a.destination is None:p.error('--destination is required for assembly')
        materialize(a.upstream,a.destination,a.profile)

if __name__=='__main__':main()
