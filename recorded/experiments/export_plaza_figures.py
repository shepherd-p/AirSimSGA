"""Regenerate sharp figures from a completed run. Does not connect to simulation."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from ag.storage import ROOT


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--input-dir',type=Path)
    p.add_argument('--refresh-viewer',action='store_true',help='Also rebuild the high-detail interactive viewer')
    a=p.parse_args()
    candidates=sorted((ROOT/'outputs/mapping').glob('*/plaza_results/surface/quality.json'),reverse=True)
    candidates=[f for f in candidates if json.loads(f.read_text(encoding='utf-8'))['quality_status']=='passed'
                and json.loads((f.parents[2]/'manifest.json').read_text(encoding='utf-8'))['status']=='completed']
    folder=a.input_dir.resolve() if a.input_dir else (candidates[0].parents[2] if candidates else None)
    if folder is None:p.error('No completed plaza reconstruction found')
    py=ROOT/'.venv_reconstruction/Scripts/python.exe'
    env=dict(os.environ,OMP_NUM_THREADS='4')
    if a.refresh_viewer:
        subprocess.run([str(py),str(ROOT/'reconstruct_plaza.py'),str(folder),'--view-only'],cwd=ROOT,env=env,check=True)
    subprocess.run([str(py),str(ROOT/'render_plaza.py'),str(folder)],cwd=ROOT,env=env,check=True)
    subprocess.run([str(py),str(ROOT/'render_structural_cloud.py'),str(folder)],cwd=ROOT,env=env,check=True)
    subprocess.run([sys.executable,str(ROOT/'plot_plaza.py'),str(folder)],cwd=ROOT,env=env,check=True)
    print(f'Figures updated: {folder / "plaza_results"}')
