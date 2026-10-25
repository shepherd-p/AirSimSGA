"""Offline reconstruction; execute with .venv_reconstruction/Scripts/python.exe."""
import argparse
import json
import os
import webbrowser
from datetime import datetime
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',required=True,type=Path)
    p.add_argument('--method',choices=['lidar','tsdf'],default='tsdf')
    p.add_argument('--voxel',type=float,default=.08)
    p.add_argument('--no-open',action='store_true')
    args=p.parse_args()
    if not .03<=args.voxel<=.5:
        p.error('--voxel must be between .03 and .5 metres')
    # Bound CPU thread count; large laptop thread pools severely slow small RGB-D integrations.
    os.environ.setdefault('OMP_NUM_THREADS','4')
    from ag.reconstruction import lidar_reconstruct,tsdf_reconstruct
    from ag.storage import write_json
    folder=args.input_dir.resolve()
    output=folder/('reconstruction_'+args.method)/datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True,exist_ok=False)
    import hashlib
    sources={}
    for source in [Path(__file__),Path(__file__).parent/'ag/reconstruction.py',Path(__file__).parent/'ag/map3d_template.html']:
        content=source.read_bytes()
        (output/source.name).write_bytes(content)
        sources[source.name]=hashlib.sha256(content).hexdigest()
    write_json(output/'reconstruction_source.json',{'sha256':sources,'voxel_m':args.voxel,'method':args.method})
    write_json(output/'status.json',{'status':'running','method':args.method,'input_dir':str(folder)})
    try:
        html,report=(lidar_reconstruct if args.method=='lidar' else tsdf_reconstruct)(folder,output,args.voxel)
        write_json(output/'status.json',{'status':'completed' if report.get('quality_status','passed')=='passed' else 'quality_check_failed',
                                         'viewer':str(html),'method':args.method})
    except Exception as exc:
        write_json(output/'status.json',{'status':'failed','error':str(exc),'method':args.method})
        raise
    print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)
    print(f'3D reconstruction: {html}',flush=True)
    if not args.no_open:
        webbrowser.open(html.as_uri())
    if report.get('quality_status','passed')!='passed':
        print('Quality check failed; output is diagnostic only.',flush=True)
        raise SystemExit(2)


if __name__=='__main__':
    main()
