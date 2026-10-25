"""Seal a source release and create a zip; exclude runtime output and local settings."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[1]

def included(path):
    rel=path.relative_to(ROOT)
    if not path.is_file() or '__pycache__' in rel.parts or '.git' in rel.parts:return False
    if path.suffix in ['.pyc','.pyo']:return False
    if any(part.startswith('.venv') or part.endswith('.egg-info') for part in rel.parts):return False
    if rel.as_posix() in ['settings/local.json','RELEASE_MANIFEST.json','experiments/verification/client.lock']:return False
    if rel.parts[:2]==('experiments','outputs'):return False
    return True

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--zip',type=Path,required=True,help='NEW zip path outside source directory')
    a=p.parse_args();destination=a.zip.resolve()
    if destination.exists():raise FileExistsError('Zip exists; choose a new filename')
    if destination.is_relative_to(ROOT):raise ValueError('Zip must be outside the source release')
    files=sorted((path for path in ROOT.rglob('*') if included(path)),key=lambda x:x.relative_to(ROOT).as_posix())
    records=[]
    for path in files:
        content=path.read_bytes()
        records.append({'path':path.relative_to(ROOT).as_posix(),'bytes':len(content),'sha256':hashlib.sha256(content).hexdigest()})
    manifest={'schema':1,'project':'AirSimSGA','version':'0.1.0','created_utc':datetime.now(timezone.utc).isoformat(),
              'files':records,'payload_bytes':sum(record['bytes'] for record in records),
              'scope':'Source, lightweight historical evidence, notices and docs; external simulator/assets/raw supplement required',
              'excluded':['.git','Python caches','virtual environments','local settings','runtime outputs','manifest itself']}
    (ROOT/'RELEASE_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    files.append(ROOT/'RELEASE_MANIFEST.json');destination.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(destination,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as archive:
        for path in files:archive.write(path,ROOT.name+'/'+path.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(destination) as archive:
        if archive.testzip() is not None:raise ValueError('Zip CRC check failed')
        for record in records:
            if hashlib.sha256(archive.read(ROOT.name+'/'+record['path'])).hexdigest()!=record['sha256']:
                raise ValueError('Zip payload hash mismatch: '+record['path'])
    digest=hashlib.sha256(destination.read_bytes()).hexdigest()
    destination.with_suffix(destination.suffix+'.sha256').write_text(digest+'  '+destination.name+'\n',encoding='ascii')
    print(json.dumps({'zip':str(destination),'zip_bytes':destination.stat().st_size,'sha256':digest,'source_files':len(records),
                      'source_bytes':manifest['payload_bytes'],'zip_crc_and_payload_hashes_verified':True},indent=2))
if __name__=='__main__':main()
