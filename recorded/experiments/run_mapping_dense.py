"""Collect dense RGB-D with existing SDK, then reconstruct in the isolated Open3D env."""
import asyncio
import json
import os
import subprocess
import sys
from ag.cli import parser, configuration, validate
from ag.storage import ROOT


def main():
    p = parser('mapping')
    p.set_defaults(config=str(ROOT/'configs/mapping_dense.json'))
    args = p.parse_args()
    os.chdir(ROOT)
    config = configuration('mapping',args)
    if args.validate_only:
        print(json.dumps(validate(config),indent=2))
        return
    python = ROOT/'.venv_reconstruction/Scripts/python.exe'
    if not python.exists():
        p.error('Run setup_reconstruction.ps1 once before dense mapping')
    from ag.experiments import Experiment
    experiment = Experiment(config,args.output_dir)
    try:
        folder,status = asyncio.run(experiment.execute())
    except KeyboardInterrupt:
        print(f'Interrupted; partial capture: {experiment.run.path}')
        raise SystemExit(130)
    print(json.dumps({'capture_status':status,'output':str(folder)}),flush=True)
    if status!='completed':
        raise SystemExit(2)
    cmd = [str(python),str(ROOT/'reconstruct_map.py'),'--input-dir',str(folder),'--method','tsdf']
    if args.no_view:
        cmd.append('--no-open')
    raise SystemExit(subprocess.call(cmd,cwd=ROOT))


if __name__=='__main__':
    main()
