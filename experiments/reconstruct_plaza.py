import os
os.environ.setdefault('OMP_NUM_THREADS','4')
import argparse
import json
from pathlib import Path
from ag.plaza_reconstruction import lidar_products,combined_viewer,existing_lidar_layers
from ag.reconstruction import tsdf_reconstruct


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path)
    p.add_argument('--surface-only',action='store_true',help='Reuse completed LiDAR products')
    p.add_argument('--view-only',action='store_true',help='Refresh views of completed products without reconstructing')
    a=p.parse_args()
    out=a.folder/'plaza_results';out.mkdir(exist_ok=True)
    print('Reusing saved LiDAR products...' if a.surface_only or a.view_only else 'Aggregating every saved LiDAR packet...',flush=True)
    layers=existing_lidar_layers(out) if a.surface_only or a.view_only else lidar_products(a.folder,out)
    surface=out/'surface';surface.mkdir(exist_ok=True)
    if a.view_only:
        html=surface/'map_dense_3d.html'
        report=json.loads((surface/'quality.json').read_text(encoding='utf-8'))
    else:
        html,report=tsdf_reconstruct(a.folder,surface,voxel=.1,support_distance=.35)
    print(combined_viewer(a.folder,out,layers,html),flush=True)
    if report['quality_status']!='passed':raise SystemExit(2)
