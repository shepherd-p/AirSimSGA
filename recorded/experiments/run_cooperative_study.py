"""Run one of the remaining three experiments with synchronized evidence capture."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
from ag.storage import ROOT
from ag.cooperative_study import CooperativeStudy


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('task',choices=['navigation','tracking','formation']);p.add_argument('--config',type=Path)
    p.add_argument('--duration',type=float);p.add_argument('--capture-only',action='store_true')
    p.add_argument('--validate-only',action='store_true')
    a=p.parse_args();os.chdir(ROOT)
    config=json.loads((a.config or ROOT/f'configs/{a.task}_study.json').read_text(encoding='utf-8'))
    if config['task']!=a.task:p.error('Task/config mismatch')
    if a.duration is not None:config['duration_sec']=a.duration
    sys.path.insert(0,config['sdk_path'])
    from projectairsim.utils import load_scene_config_as_dict
    scene,_=load_scene_config_as_dict(config['scene'],config.get('sim_config_path',str(ROOT/'configs')))
    actors=[x['name'] for x in scene['actors'] if x['name']!='Observer']
    expected=config.get('controlled_actor_count',{'navigation':2,'tracking':3,'formation':7}[a.task])
    if len(actors)!=expected:p.error('Incorrect controlled actor count')
    if len(set(actors))!=len(actors):p.error('Duplicate controlled actor names')
    if a.validate_only:
        print(json.dumps({'status':'valid','task':a.task,'controlled_actors':actors,'observer_is_camera_only':True}));return
    experiment=CooperativeStudy(config)
    try:folder,status=asyncio.run(experiment.execute())
    except KeyboardInterrupt:print(f'Interrupted: {experiment.run.path}');raise SystemExit(130)
    print(json.dumps({'output':str(folder),'status':status}),flush=True)
    if status!='completed':raise SystemExit(2)
    exporter=ROOT/'export_cooperative_figures.py'
    if not a.capture_only:
        subprocess.run([sys.executable,str(exporter),'--input-dir',str(folder)],check=True)
        subprocess.run([sys.executable,str(ROOT/'audit_cooperative_study.py'),'--input-dir',str(folder)],check=True)


if __name__=='__main__':main()
