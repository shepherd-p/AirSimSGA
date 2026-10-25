"""Export structure-focused point clouds only; no simulator or other figure edits."""
import argparse
import json
import os
from pathlib import Path
import subprocess
from ag.storage import ROOT
from ag.figure_plot import frame_structural_figures


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir',type=Path)
    args=parser.parse_args()
    folder=args.input_dir.resolve() if args.input_dir else None
    if folder is None:
        for path in sorted((ROOT/'outputs/mapping').glob('*/manifest.json'),reverse=True):
            if json.loads(path.read_text(encoding='utf-8')).get('status')=='completed' and all(
                (path.parent/'plaza_results'/f'lidar_{name}_timed.npz').exists() for name in ('Drone1','Car1')):
                folder=path.parent;break
    if folder is None:parser.error('No completed plaza capture found; specify --input-dir')
    py=ROOT/'.venv_reconstruction/Scripts/python.exe'
    if not py.exists():parser.error('Run setup_reconstruction.ps1 first')
    subprocess.run([str(py),str(ROOT/'render_structural_cloud.py'),str(folder)],
                   cwd=ROOT,env=dict(os.environ,OMP_NUM_THREADS='4'),check=True)
    frame_structural_figures(folder)
    print(f'Structure-focused point clouds: {folder / "plaza_results"}')


if __name__=='__main__':main()
