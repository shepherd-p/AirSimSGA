import argparse
import asyncio
import json
import os
import sys
from pathlib import Path
from .storage import ROOT, write_json, verify_originals


def configuration(task, args):
    path = Path(args.config) if args.config else ROOT/f'configs/{task}.json'
    from .portable import resolve_config
    config = resolve_config(json.loads(path.read_text(encoding='utf-8-sig')))
    if config['task'] != task:
        raise ValueError(f'Config task {config["task"]} does not match {task}')
    if getattr(args,'duration',None) is not None:
        config['duration_sec'] = args.duration
    if args.ip:
        config['ip'] = args.ip
    if getattr(args,'land',False):
        config['cleanup'] = 'land'
    if config['duration_sec'] <= 0 or config['control_hz'] <= 0:
        raise ValueError('Duration and control frequency must be positive')
    return config


def parser(task):
    p = argparse.ArgumentParser(description=f'ProjectAirSim air-ground {task}; loads only independent configs')
    p.add_argument('--config')
    p.add_argument('--ip')
    p.add_argument('--duration',type=float)
    p.add_argument('--output-dir')
    p.add_argument('--land',action='store_true',help='Land drones at their final location; default bounded hover')
    p.add_argument('--validate-only',action='store_true',help='Validate config/SDK schema without connecting')
    if task == 'mapping':
        p.add_argument('--no-view',action='store_true',help='Generate 3D HTML but do not open a browser')
    return p


def validate(config):
    sys.path.insert(0, config['sdk_path'])
    from projectairsim.utils import load_scene_config_as_dict
    scene,_ = load_scene_config_as_dict(config['scene'],str(ROOT/'configs'))
    names = [a['name'] for a in scene['actors']]
    if len(set(names)) != len(names):
        raise ValueError('Duplicate actor names')
    expected = 7 if config['task'] == 'formation' else 3 if config['task'] == 'tracking' else 2
    if len(names) != expected:
        raise ValueError(f'Expected {expected} actors')
    for actor in scene['actors']:
        sensors = actor['robot-config']['sensors']
        if not any(s['type']=='lidar' and s['enabled'] for s in sensors):
            raise ValueError('Missing lidar')
    return {'status':'valid','task':config['task'],'actors':names,'scene_id':scene['id']}


def main(task):
    args = parser(task).parse_args()
    os.chdir(ROOT)  # SDK's own logger writes relative to cwd; keep it outside existing projects.
    config = configuration(task,args)
    if args.validate_only:
        print(json.dumps(validate(config),ensure_ascii=False,indent=2))
        return
    from .experiments import Experiment
    experiment = Experiment(config,args.output_dir)
    try:
        path,status = asyncio.run(experiment.execute())
    except KeyboardInterrupt:
        print(f'Interrupted; partial data: {experiment.run.path}')
        raise SystemExit(130)
    print(json.dumps({'output':str(path),'status':status},ensure_ascii=False))
    if task == 'mapping' and list(path.glob('map_*.pcd')):
        try:
            from .map3d import build_viewer
            viewer = build_viewer(path)
            print(f'3D map: {viewer}',flush=True)
            if not args.no_view:
                import webbrowser
                webbrowser.open(viewer.as_uri())
        except Exception as exc:
            print(f'3D viewer export failed (PCD files retained): {exc}',flush=True)
    if status != 'completed':
        raise SystemExit(2)


def preflight():
    p = argparse.ArgumentParser(description='Read-only topic check; never loads a scene or moves a robot')
    p.add_argument('--config',default=str(ROOT/'configs/mapping.json'))
    p.add_argument('--seconds',type=float,default=5)
    p.add_argument('--offline',action='store_true')
    args = p.parse_args()
    os.chdir(ROOT)
    report = {'protection':verify_originals(),'configs':[]}
    try:
        for task in ('mapping','navigation','tracking','formation'):
            report['configs'].append(validate(json.loads((ROOT/f'configs/{task}.json').read_text(encoding='utf-8'))))
        if not args.offline:
            from .backend import Backend
            config = json.loads(Path(args.config).read_text(encoding='utf-8'))
            backend = Backend(config)
            async def inspect():
                try:
                    backend.connect()
                    return await backend.inspect(max(.1,args.seconds))
                finally:
                    backend.close()
            report['live'] = asyncio.run(inspect())
        report['status'] = 'passed' if report['protection']['unchanged'] else 'original_files_changed'
        if report['protection']['unchanged'] and 'live' in report and not report['live']['all_selected_streams_active']:
            report['status'] = 'inactive_sensor_streams' if report['live']['streams'] else 'ready_for_explicit_scene_load'
    except Exception as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
    write_json(ROOT/'verification/preflight_latest.json',report)
    print(json.dumps(report,indent=2,ensure_ascii=False))
    if report['status'] not in ('passed','ready_for_explicit_scene_load'):
        raise SystemExit(2)
