"""AirSimSGA source-release entry point. Doctor has no simulator side effects."""
import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TASKS = {
    'mapping': ('run_mapping_plaza.py', 'CityPark plaza, UAV+UGV, measured LiDAR/RGB-D reconstruction'),
    'navigation': ('citysample_scene/run_aerial_route_navigation.py', 'City Sample, three-intersection aerial route assistance'),
    'containment': ('citysample_scene/run_pursuit.py', 'City Sample, two UAVs + two UGVs + moving Target'),
    'landing': ('run_landing_study.py', 'Landing build, AprilTag-guided landing on translating USV'),
}
SETTINGS = {
    'sdk_path':'AIRSIMSGA_SDK_PATH', 'citysample_project':'AIRSIMSGA_CITYSAMPLE_PROJECT',
    'zonegraph':'AIRSIMSGA_ZONEGRAPH', 'reconstruction_python':'AIRSIMSGA_RECONSTRUCTION_PYTHON',
    'landing_build_receipt':'AIRSIMSGA_LANDING_BUILD_RECEIPT', 'landing_backend':'AIRSIMSGA_LANDING_BACKEND',
    'output_root':'AIRSIMSGA_OUTPUT_ROOT', 'host':'AIRSIMSGA_HOST',
}


def load_settings(filename):
    if filename is None:
        filename = ROOT/'settings/local.json'
        if not filename.exists():
            return
    p = Path(filename).resolve()
    settings = json.loads(p.read_text(encoding='utf-8-sig'))
    unknown = set(settings) - set(SETTINGS)
    if unknown:
        raise ValueError(f'Unknown settings: {sorted(unknown)}')
    for key,value in settings.items():
        if value is None or value == '':
            continue
        if key != 'host':
            path = Path(value).expanduser()
            value = str(path.resolve() if path.is_absolute() else (p.parent/path).resolve())
        # Explicit settings file overrides environment for this process only.
        os.environ[SETTINGS[key]] = str(value)


def release_check():
    manifest = json.loads((ROOT/'RELEASE_MANIFEST.json').read_text(encoding='utf-8'))
    changed = []
    for record in manifest['files']:
        p = ROOT/record['path']
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != record['sha256']:
            changed.append(record['path'])
    return {'checked_files':len(manifest['files']), 'changed_files':changed, 'unchanged':not changed}


def doctor(task, offline):
    report = {'release':release_check(), 'no_simulator_connection':True, 'task':task, 'checks':[]}
    def check(name,ok,detail):
        report['checks'].append({'name':name,'ok':bool(ok),'detail':str(detail)})
    for p in (ROOT/'experiments').rglob('*.py'):
        ast.parse(p.read_text(encoding='utf-8-sig'), filename=str(p))
    check('python_sources', True, 'Syntax parsed without execution')
    modules = ['numpy','cv2','matplotlib','PIL','psutil','networkx']
    if task == 'landing': modules += ['pupil_apriltags']
    for module in modules:
        check(module, importlib.util.find_spec(module) is not None, 'Install editable source with selected optional dependencies')
    if not offline:
        sdk = Path(os.environ.get('AIRSIMSGA_SDK_PATH','__unconfigured__'))
        check('sdk', (sdk/'projectairsim').is_dir(), sdk)
        if task in ('navigation','containment'):
            sys.path.insert(0,str(ROOT/'experiments'))
            from ag.portable import zonegraph_path
            try:
                path = zonegraph_path()
                scenario_name = 'aerial_route_scenario.json' if task == 'navigation' else 'pursuit_scenario.json'
                scenario=json.loads((ROOT/'experiments/citysample_scene/generated'/scenario_name).read_text())
                match=path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==scenario['zonegraph_sha256']
                check('zonegraph', match, f'{path}; prepare scenario again if export hash differs')
            except Exception as exc:
                check('zonegraph',False,exc)
        if task == 'landing':
            value=os.environ.get('AIRSIMSGA_LANDING_BUILD_RECEIPT')
            receipt=Path(value) if value else None
            check('build_receipt',receipt is not None and receipt.is_file(),receipt or 'Set landing_build_receipt')
            if receipt and receipt.is_file():
                data=json.loads(receipt.read_text(encoding='utf-8'))
                for kind in ['binaries','sources']:
                    records=data.get(kind,{})
                    ok=bool(records) and all(Path(p).is_file() and hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in records.items())
                    check(kind+'_hashes',ok,f'{len(records)} recorded files')
                check('runtime_receipt',(receipt.parent/'runtime_receipt.json').is_file(),'Created after starting dedicated landing simulator; run time checks are repeated by experiment')
    report['status']='passed' if report['release']['unchanged'] and all(item['ok'] for item in report['checks']) else 'not_ready'
    report['scope']='offline package checks only' if offline else 'configuration and local files only; runtime/scene/assets not tested'
    print(json.dumps(report,indent=2))
    return 0 if report['status']=='passed' else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings',type=Path, help='Local JSON settings; values are relative to this file')
    subs=parser.add_subparsers(dest='command',required=True)
    subs.add_parser('list',help='List the four released demonstrations')
    d=subs.add_parser('doctor',help='Read-only source/environment checks')
    d.add_argument('--task',choices=list(TASKS),default='mapping')
    d.add_argument('--offline',action='store_true')
    subs.add_parser('check-release',help='Check packaged file hashes')
    r=subs.add_parser('run',help='Loads selected scene and controls robots in a running simulator')
    r.add_argument('task',choices=list(TASKS));r.add_argument('arguments',nargs=argparse.REMAINDER)
    p=subs.add_parser('prepare',help='Regenerate City Sample scenario files from exported ZoneGraph')
    p.add_argument('task',choices=['navigation','containment'])
    a=parser.parse_args(argv)
    try:
        load_settings(a.settings)
        if a.command=='list':
            for name,(_,description) in TASKS.items():print(f'{name}: {description}')
            return 0
        if a.command=='check-release':
            report=release_check();print(json.dumps(report,indent=2));return 0 if report['unchanged'] else 2
        if a.command=='doctor':return doctor(a.task,a.offline)
        if a.command=='prepare':
            script='prepare_aerial_route_navigation.py' if a.task=='navigation' else 'prepare_pursuit.py'
            args=[sys.executable,str(ROOT/'experiments/citysample_scene'/script)]
        else:
            forwarded=a.arguments[1:] if a.arguments[:1]==['--'] else a.arguments
            args=[sys.executable,str(ROOT/'experiments'/TASKS[a.task][0]),*forwarded]
        return subprocess.call(args,cwd=ROOT/'experiments')
    except (ValueError,FileNotFoundError,KeyError) as exc:
        print(f'AirSimSGA: {exc}',file=sys.stderr);return 2
