"""Record a LOCAL compiled landing plugin and running simulator, without launching it.
Hashes establish file identity, not correctness. Required C++ test reports are checked.
Run after independent build, plugin installation, and simulator startup.
"""
import argparse
import hashlib
import json
from pathlib import Path
from datetime import datetime, timezone
import psutil
ROOT=Path(__file__).resolve().parents[1]

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(2**20),b''):h.update(chunk)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path,required=True)
    p.add_argument('--plugin',type=Path,required=True,help='Installed ProjectAirSim plugin DLL')
    p.add_argument('--build-log',type=Path,required=True)
    p.add_argument('--support-tests',type=Path,required=True,help='gtest JSON from support_contact_tests')
    p.add_argument('--firmware-tests',type=Path,required=True,help='gtest JSON from firmware_disarm_tests')
    p.add_argument('--pid',type=int,required=True)
    p.add_argument('--output',type=Path,required=True,help='NEW directory for local build/runtime receipts')
    a=p.parse_args()
    backend=a.backend.resolve();plugin=a.plugin.resolve()
    if a.output.exists():raise FileExistsError('Receipt directory already exists')
    metadata=json.loads((ROOT/'native/profiles/moving-deck/profile.json').read_text(encoding='utf-8'))
    for record in metadata['files']:
        if sha(backend/record['path'])!=record['extension_sha256']:
            raise ValueError('Extended source differs from release profile: '+record['path'])
    reports={}
    for name,path in [('support',a.support_tests),('firmware',a.firmware_tests)]:
        report=json.loads(path.read_text(encoding='utf-8'))
        if report.get('tests',0)<1 or report.get('failures',1) or report.get('errors',1) or report.get('disabled',1):
            raise ValueError('C++ tests failed or are incomplete: '+name)
        reports[name]={'file':str(path.resolve()),'sha256':sha(path),'tests':report['tests']}
    process=psutil.Process(a.pid)
    loaded=[Path(mapping.path).resolve() for mapping in process.memory_maps(grouped=True)]
    if plugin not in loaded:raise ValueError('Selected simulator PID has not loaded the given plugin DLL')
    sources={str((backend/record['path']).resolve()):sha(backend/record['path']) for record in metadata['files']}
    sources[str(a.build_log.resolve())]=sha(a.build_log)
    for info in reports.values():sources[info['file']]=info['sha256']
    receipt={'created_utc':datetime.now(timezone.utc).isoformat(),'project':'AirSimSGA local moving-deck build',
             'binaries':{str(plugin):sha(plugin)},'sources':sources,'physics_tests':reports['support'],
             'firmware_tests':reports['firmware'],'base_commit':metadata['base_commit'],
             'scope':'User-supplied build log/test reports + source and loaded-binary identity; no automated proof that binary derives from these sources'}
    runtime={'pid':a.pid,'create_time':process.create_time(),'loaded_plugin':str(plugin),'plugin_sha256':sha(plugin)}
    a.output.mkdir(parents=True)
    (a.output/'build_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    (a.output/'runtime_receipt.json').write_text(json.dumps(runtime,indent=2)+'\n')
    print(a.output.resolve())

if __name__=='__main__':main()
