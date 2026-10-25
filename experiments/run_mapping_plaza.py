"""One run -> measured LiDAR, colored surface, and measured trajectories."""
import asyncio
import json
import os
import subprocess
import sys
from pathlib import Path
from ag.cli import parser,configuration,validate
from ag.storage import ROOT


def main():
    p=parser('mapping')
    p.set_defaults(config=str(ROOT/'configs/mapping_plaza.json'))
    p.add_argument('--capture-only',action='store_true')
    p.add_argument('--reconstruct',type=Path,help='Reprocess an existing plaza capture without moving vehicles')
    a=p.parse_args()
    os.chdir(ROOT)
    c=configuration('mapping',a)
    if a.validate_only:
        print(json.dumps(validate(c),indent=2));return
    py=Path(os.environ.get('AIRSIMSGA_RECONSTRUCTION_PYTHON',sys.executable))
    if not a.capture_only and not py.is_file(): p.error('Set AIRSIMSGA_RECONSTRUCTION_PYTHON to a Python with the reconstruction dependencies')
    folder=a.reconstruct
    if folder is None:
        from ag.plaza import PlazaExperiment
        e=PlazaExperiment(c,a.output_dir)
        try: folder,status=asyncio.run(e.execute())
        except KeyboardInterrupt:
            print(f'Interrupted: {e.run.path}');raise SystemExit(130)
        print(f'Capture {status}: {folder}',flush=True)
        if status!='completed': raise SystemExit(2)
        if a.capture_only:return
    env=dict(os.environ,OMP_NUM_THREADS='4')
    result=subprocess.call([str(py),str(ROOT/'reconstruct_plaza.py'),str(folder)],env=env,cwd=ROOT)
    if (folder/'plaza_results/plot_points.npz').exists():
        if (folder/'plaza_results/surface/surface_tsdf_neu.ply').exists():
            subprocess.run([str(py),str(ROOT/'render_plaza.py'),str(folder)],env=env,check=True)
        subprocess.run([str(py),str(ROOT/'render_structural_cloud.py'),str(folder)],env=env,check=True)
        subprocess.run([sys.executable,str(ROOT/'plot_plaza.py'),str(folder)],check=True)
    if result:raise SystemExit(result)
    html=folder/'plaza_results/map_plaza_3d.html'
    print(f'Results: {html}',flush=True)
    if not a.no_view:
        import webbrowser
        webbrowser.open(html.resolve().as_uri())


if __name__=='__main__':main()
