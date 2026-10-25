"""Structure-focused display of measured LiDAR; original cloud files unchanged."""
import os
os.environ.setdefault('OMP_NUM_THREADS','4')
import argparse
import json
from pathlib import Path
import numpy as np
import open3d as o3d
import ag.figure_render as renderer


def structural_display(points,source):
    if len(points)==0:raise ValueError('No LiDAR points in the selected plaza region')
    pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    input_count=len(points)
    _,valid=pc.remove_radius_outlier(nb_points=8,radius=.24)
    original_indices=np.asarray(valid,dtype=np.int64)
    if not len(original_indices):raise ValueError('No supported LiDAR points; retain the full-cloud view for this capture')
    points=points[original_indices];source=source[original_indices]
    pc=pc.select_by_index(valid)
    # Estimate geometry on an evenly spaced support cloud so overlapping
    # platform returns do not dominate the nearest-neighbour covariance.
    pc=pc.voxel_down_sample(.20)
    search=o3d.geometry.KDTreeSearchParamHybrid(radius=.8,max_nn=80)
    pc.estimate_normals(search);pc.estimate_covariances(search)
    eigen=np.linalg.eigvalsh(np.asarray(pc.covariances))
    roughness=eigen[:,0]/np.maximum(eigen.sum(axis=1),1e-12)
    lookup=o3d.core.nns.NearestNeighborSearch(o3d.core.Tensor(np.asarray(pc.points).astype('f4')))
    lookup.knn_index()
    nearest=[]
    for start in range(0,len(points),250000):
        ids,_=lookup.knn_search(o3d.core.Tensor(points[start:start+250000].astype('f4')),1)
        nearest.append(ids.numpy().ravel())
    nearest=np.concatenate(nearest)
    normals=np.asarray(pc.normals)[nearest]
    flat=(abs(normals[:,2])>.94)&(roughness[nearest]<.08)
    # Select existing fused points, separately for each platform. The support
    # centroids guide classification only; they never replace exported points.
    ids=np.flatnonzero(flat)
    keys=np.c_[source[ids],np.floor(points[ids]/.28).astype('i4')]
    _,sample=np.unique(keys,axis=0,return_index=True)
    kept=np.r_[np.flatnonzero(~flat),ids[sample]]
    return points[kept],source[kept],normals[kept],original_indices[kept],{'input_points':input_count,
        'radius_supported_points':len(points),'radius_removed_points':input_count-len(points),
        'support_radius_m':.24,'minimum_neighbors':8,
        'flat_points':int(flat.sum()),'flat_points_shown':len(sample),'display_points':len(kept),
        'flat_spacing_m':.28,'normal_support_spacing_m':.20,'normal_radius_m':.8,
        'selection':'Existing fused points selected by index; all supported non-flat points retained'}


def render(folder):
    out=folder/'plaza_results';parts=[];sources=[]
    for i,n in enumerate(['Drone1','Car1']):
        with np.load(out/f'lidar_{n}_timed.npz') as d:p=d['points_ned']*[1,1,-1]
        p=p[renderer.crop(p)];parts.append(p);sources.append(np.full(len(p),i,dtype='u1'))
    print('Estimating measured local surface geometry...',flush=True)
    points,source,normals,indices,report=structural_display(np.vstack(parts),np.concatenate(sources))
    np.savez_compressed(out/'structural_display.npz',points_neu=points,source=source,normals=normals,
                        input_point_indices=indices)
    light=np.array([-.4,-.2,1.]);light/=np.linalg.norm(light)
    intensity=.4+.6*abs(normals@light)
    # Saturated blue/green follows the two-platform convention in the reference.
    base=np.array([[.05,.12,.95],[.05,.73,.16]])[source]
    color=base*intensity[:,None]
    rgb=renderer.point_image(points,color,splat=False,width=3200,height=2600)
    o3d.io.write_image(str(out/'pointcloud_structure_render.png'),o3d.geometry.Image(rgb))
    # The central sculpture and terrace stairs can also be examined without
    # distant vegetation obscuring them. This is explicitly a cropped view.
    mask=np.all((points>=[10,34,-15])&(points<=[56,80,22]),axis=1)
    previous_low,previous_high=renderer.LOW.copy(),renderer.HIGH.copy()
    try:
        renderer.LOW[:]=[10,34,-15];renderer.HIGH[:]=[56,80,22]
        rgb=renderer.point_image(points[mask],color[mask],splat=False,width=3000,height=2400,
                                  world_width=66,elevation=35,azimuth=-115)
    finally:
        renderer.LOW[:]=previous_low;renderer.HIGH[:]=previous_high
    o3d.io.write_image(str(out/'pointcloud_structure_detail_render.png'),o3d.geometry.Image(rgb))
    report.update(note='Display-only radius support filtering, flat-surface thinning and normal shading; no mesh, invented outlines, or changes to archived LiDAR.',
                  source_colors={'Drone1':'blue','Car1':'green'},focused_points=int(mask.sum()),
                  input_index_reference='Concatenate cropped Drone1 then Car1 lidar_*_timed.npz, in original row order',
                  whole_roi_neu=[[-5,25,-18],[65,88,30]],detail_roi_neu=[[10,34,-15],[56,80,22]],
                  elevation_degrees={'whole':32,'detail':35},azimuth_degrees=-115,
                  limitations='Display processing only; not a new scan or a demonstrated improvement in geometric accuracy. Radius filtering can remove sparsely observed thin structures; the full-cloud view remains available.')
    (out/'structural_display.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);render(p.parse_args().folder)
