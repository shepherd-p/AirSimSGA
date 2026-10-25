"""Add LiDAR surface support to an existing TSDF without rerunning capture/integration."""
import os
os.environ.setdefault('OMP_NUM_THREADS','4')
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from ag.reconstruction import constrain_surface,cloud,mesh_layer,point_layer,viewer
from ag.storage import write_json


def refine(folder):
    out=folder/'plaza_results/surface'
    report=json.loads((out/'quality.json').read_text(encoding='utf-8'))
    if report.get('lidar_surface_support'):
        raise ValueError('Already constrained; use reconstruct_plaza.py --view-only to update views')
    poses=json.loads((out/'pose_refinement.json').read_text(encoding='utf-8'))
    mesh=o3d.io.read_triangle_mesh(str(out/'surface_tsdf_ned.ply'))
    reference=o3d.io.read_point_cloud(str(folder/'lidar_reference_ned.ply'))
    support=constrain_surface(mesh,reference)
    scene=o3d.t.geometry.RaycastingScene();scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    rows=list(csv.DictReader((folder/'rgbd_frames.csv').open(encoding='utf-8')))
    errors=[];rays_total=0;hits_total=0;used=0
    for i,row in enumerate(rows):
        if i%10!=5:continue
        with np.load(folder/row['file']) as f:
            position=f['camera_position_ned'];rotation=f['camera_rotation_ned']
            rec=poses.get(str(int(f['time_ns']))+':'+str(position.tolist()))
            if not rec or not rec['integrable']:continue
            t=np.array(rec['transform']);position=t[:3,:3]@position+t[:3,3];rotation=t[:3,:3]@rotation
            d=f['range_m'][::8,::8];h,w=f['range_m'].shape
            v,u=np.mgrid[0:h:8,0:w:8];focal=w/(2*np.tan(np.deg2rad(float(f['fov_deg']))/2))
            valid=(d>.5)&(d<45.)
            directions=np.stack([np.ones_like(u),(u-w/2)/focal,(v-h/2)/focal],axis=-1)[valid]
            directions/=np.linalg.norm(directions,axis=1,keepdims=True);directions=directions@rotation.T
            rays=np.c_[np.broadcast_to(position,directions.shape),directions].astype('f4')
            if not len(rays):continue
            used+=1;prediction=scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy();good=np.isfinite(prediction)
            rays_total+=len(rays);hits_total+=int(good.sum());errors.extend(abs(prediction[good]-d[valid][good]))
    e=np.array(errors);evaluation=dict(heldout_frames=sum(i%10==5 for i in range(len(rows))),heldout_used_frames=used,
        heldout_rejected_frames=sum(i%10==5 for i in range(len(rows)))-used,rays=rays_total,
        surface_hit_fraction=hits_total/max(rays_total,1),absolute_range_error_median_m=float(np.median(e)),
        absolute_range_error_p90_m=float(np.percentile(e,90)),range_rmse_m=float(np.sqrt(np.mean(e**2))))
    passed=len(e)>100 and np.median(e)<.2 and np.percentile(e,90)<.8 and hits_total/max(rays_total,1)>.75
    report.update(lidar_surface_support=support,evaluation=evaluation,mesh_vertices=len(mesh.vertices),
                  mesh_triangles=len(mesh.triangles),quality_status='passed' if passed else 'quality_check_failed')
    backup=out/'before_lidar_support';backup.mkdir(exist_ok=False)
    for filename in ['surface_tsdf_ned.ply','surface_tsdf_neu.ply','dense_colored_ned.ply','quality.json','map_dense_3d.html']:
        (out/filename).rename(backup/filename)
    o3d.io.write_triangle_mesh(str(out/'surface_tsdf_ned.ply'),mesh)
    up=o3d.geometry.TriangleMesh(mesh);up.vertices=o3d.utility.Vector3dVector(np.asarray(up.vertices)*[1,1,-1])
    up.triangles=o3d.utility.Vector3iVector(np.asarray(up.triangles)[:,[0,2,1]]);up.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(out/'surface_tsdf_neu.ply'),up)
    p=cloud(np.asarray(mesh.vertices));p.colors=mesh.vertex_colors
    o3d.io.write_point_cloud(str(out/'dense_colored_ned.ply'),p)
    write_json(out/'quality.json',report)
    print(json.dumps(report,indent=2),flush=True)
    viewer(out/'map_dense_3d.html',[mesh_layer('TSDF 彩色表面',mesh),point_layer('稠密彩色点云',p)],report['quality_status'],
           'RGB-D / TSDF 彩色表面 + 独立激光观测支持；移除遮挡边缘悬空面，保留真实未观测空洞。')
    if not passed:raise SystemExit(2)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);refine(p.parse_args().folder)
