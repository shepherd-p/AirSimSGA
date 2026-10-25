"""Measured-data reconstruction: multiscale robust ICP and projective RGB-D TSDF."""
import base64
import csv
import json
import math
from pathlib import Path
import numpy as np
import open3d as o3d
from .storage import ROOT, write_json


def cloud(points):
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points,dtype=float)))


def rgbd_geometry(range_m, fov_deg, position, rotation, max_depth):
    """Convert perspective RANGE + FRD camera pose to Open3D axial Z/OpenCV camera."""
    h,w = range_m.shape
    f = w/(2*math.tan(math.radians(float(fov_deg))/2))
    v,u = np.mgrid[:h,:w]
    factor = np.sqrt(1+((u-w/2)/f)**2+((v-h/2)/f)**2)
    axial = np.where((range_m>.1)&(range_m<max_depth),range_m/factor,0).astype(np.float32)
    # OpenCV right/down/forward -> ProjectAirSim forward/right/down.
    cv_to_frd = np.array([[0,0,1],[1,0,0],[0,1,0]],dtype=float)
    camera_to_world = np.eye(4)
    camera_to_world[:3,:3] = rotation@cv_to_frd
    camera_to_world[:3,3] = position
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w,h,f,f,w/2,h/2)
    return axial,intrinsic,np.linalg.inv(camera_to_world)


def align_multiscale(source,target):
    reg = o3d.pipelines.registration
    transform=np.eye(4)
    stages=[]
    for size,gate in [(.4,1.),(.2,.5),(.08,.2)]:
        a,b=source.voxel_down_sample(size),target.voxel_down_sample(size)
        b.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=size*4,max_nn=40))
        method=reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=gate/2))
        result=reg.registration_icp(a,b,gate,transform,method,reg.ICPConvergenceCriteria(max_iteration=35))
        transform=result.transformation
        stages.append({'voxel_m':size,'gate_m':gate,'fitness':result.fitness,'rmse_m':result.inlier_rmse})
    # Compare on SAME full-resolution sampling and correspondence gate, not old ICP's 1800 points.
    a,b=source.voxel_down_sample(.08),target.voxel_down_sample(.08)
    before=reg.evaluate_registration(a,b,.25,np.eye(4))
    after=reg.evaluate_registration(a,b,.25,transform)
    angle=math.degrees(math.acos(np.clip((np.trace(transform[:3,:3])-1)/2,-1,1)))
    displacement=float(np.linalg.norm(transform[:3,3]))
    accepted=bool(after.fitness>=before.fitness*.98 and after.inlier_rmse<before.inlier_rmse
                  and displacement<.25 and angle<.5 and after.fitness>.1)
    return transform if accepted else np.eye(4),{
        'method':'multiscale point-to-plane ICP with Tukey robust loss',
        'stages':stages,'evaluation_gate_m':.25,
        'before':{'fitness':before.fitness,'rmse_m':before.inlier_rmse},
        'candidate_after':{'fitness':after.fitness,'rmse_m':after.inlier_rmse},
        'candidate_translation_m':displacement,'candidate_rotation_deg':angle,
        'accepted':accepted,'candidate_transform':transform.tolist(),
        'note':'Shared simulator poses already align clouds. Reject corrections without improved local fit and small drift.'}


def binary(values,dtype='<f4'):
    return base64.b64encode(np.asarray(values,dtype=dtype).tobytes()).decode('ascii')


def point_layer(name,p):
    points=np.asarray(p.points)*[1,1,-1]
    item={'name':name,'count':len(points),'positions_b64':binary(points)}
    if p.has_colors():
        item['colors_b64']=binary(np.asarray(p.colors))
    return item


def mesh_layer(name,mesh,max_triangles=180000):
    preview=mesh.simplify_quadric_decimation(max_triangles) if len(mesh.triangles)>max_triangles else mesh
    preview.compute_vertex_normals()
    item={'name':name,'count':len(preview.vertices),'positions_b64':binary(np.asarray(preview.vertices)*[1,1,-1]),
          'triangles_b64':binary(np.asarray(preview.triangles)[:,[0,2,1]],'<u4'),
          'normals_b64':binary(np.asarray(preview.vertex_normals)*[1,1,-1]),
          'faces':len(preview.triangles),'full_faces':len(mesh.triangles),'full_vertices':len(mesh.vertices)}
    if preview.has_vertex_colors():
        item['colors_b64']=binary(np.asarray(preview.vertex_colors))
    return item


def viewer(path,layers,status,description):
    data={'run':path.parents[2].name,'status':status,'layers':layers,
          'description':description,'defaultColor':'rgb'}
    template=(ROOT/'ag/map3d_template.html').read_text(encoding='utf-8')
    payload=json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('<','\\u003c')
    path.write_text(template.replace('/*__MAP_DATA__*/',payload),encoding='utf-8')
    return path


def lidar_reconstruct(folder,output,voxel=.08):
    combined={}; stats={}
    for name in ('Drone1','Car1'):
        scans=sorted((folder/'raw'/name).glob('*.npz'))
        if not scans:
            raise ValueError(f'No raw LiDAR for {name}')
        raw=np.vstack([np.load(f)['world_points'] for f in scans])
        raw=raw[np.isfinite(raw).all(axis=1)]
        p=cloud(raw).voxel_down_sample(voxel)  # centroid, not first return per voxel
        filtered,indices=p.remove_statistical_outlier(nb_neighbors=20,std_ratio=2.5)
        stats[name]={'scans':len(scans),'raw_returns':len(raw),'centroids':len(p.points),
                     'retained_points':len(filtered.points),'rejected_outliers':len(p.points)-len(filtered.points)}
        combined[name]=filtered
    transform,registration=align_multiscale(combined['Drone1'],combined['Car1'])
    combined['Drone1'].transform(transform)
    fused=(combined['Drone1']+combined['Car1']).voxel_down_sample(voxel)
    o3d.io.write_point_cloud(str(output/'lidar_refined_ned.ply'),fused)
    up=cloud(np.asarray(fused.points)*[1,1,-1])
    o3d.io.write_point_cloud(str(output/'lidar_refined_neu.ply'),up)
    layers=[point_layer(f'精细激光融合 · {voxel*100:g} cm',fused)]
    old=folder/'map_fused.pcd'
    if old.exists():
        layers.append(point_layer('原始基线 · 30 cm',o3d.io.read_point_cloud(str(old))))
    report={'method':'raw scans -> centroid voxel -> statistical outlier filter -> guarded robust multiscale ICP',
            'voxel_m':voxel,'sensors':stats,'registration':registration,'fused_points':len(fused.points),
            'limitation':'No new observations; holes and sparse scan rings cannot be reconstructed from absent returns.'}
    write_json(output/'quality.json',report)
    html=viewer(output/'map_refined_3d.html',layers,'completed',
                '同一批原始激光数据 · 8 cm 体素质心 + 离群点过滤 + 鲁棒多尺度配准；未填补未观测区域。')
    return html,report


def constrain_surface(mesh,reference,distance=.35):
    measured=np.asarray(cloud(np.asarray(mesh.vertices)).compute_point_cloud_distance(reference))
    unsupported=np.any(measured[np.asarray(mesh.triangles)]>distance,axis=1)
    report={'maximum_vertex_distance_m':distance,'vertices_before':len(mesh.vertices),
            'triangles_before':len(mesh.triangles),'rejected_triangles':int(unsupported.sum()),
            'unsupported_vertex_fraction':float(np.mean(measured>distance)),
            'reason':'Reject RGB-D frustum/occlusion surfaces without independent LiDAR support; leave gaps, never fill them'}
    mesh.remove_triangles_by_mask(unsupported);mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return report


def tsdf_reconstruct(folder,output,voxel=.08,max_depth=45.,support_distance=None):
    rows=list(csv.DictReader((folder/'rgbd_frames.csv').open(encoding='utf-8')))
    if len(rows)<4:
        raise ValueError('Insufficient RGB-D frames; run run_mapping_dense.py first')
    # The rendered depth and reported camera pose can disagree during motion.
    # Anchor EACH RGB-D camera to independent LiDAR, retaining the simulator pose as prior.
    scans=[f for name in ('Drone1','Car1') for f in (folder/'raw'/name).glob('*.npz')]
    cached_reference=folder/'lidar_reference_ned.ply'
    reference=(o3d.io.read_point_cloud(str(cached_reference)) if cached_reference.exists()
               else cloud(np.vstack([np.load(f)['world_points'] for f in scans])).voxel_down_sample(.15))
    reference.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.7,max_nn=40))
    reference_points=np.asarray(reference.points)
    reference_normals=np.asarray(reference.normals)
    reg=o3d.pipelines.registration
    pose_records={}
    def corrected_pose(frame):
        stamp=str(int(frame['time_ns']))
        position=frame['camera_position_ned'];rotation=frame['camera_rotation_ned']
        d=frame['range_m'];h,w=d.shape;v,u=np.mgrid[0:h:5,0:w:5];depth=d[::5,::5]
        valid=(depth>.5)&(depth<max_depth)
        f=w/(2*np.tan(np.deg2rad(float(frame['fov_deg']))/2))
        rays=np.stack([np.ones_like(u),(u-w/2)/f,(v-h/2)/f],axis=-1)[valid]
        rays/=np.linalg.norm(rays,axis=1,keepdims=True)
        p=cloud((rays*depth[valid,None])@rotation.T+position).voxel_down_sample(.15)
        if len(p.points)<50:
            return position,rotation,False
        # Registration is local to this camera's observed volume. Avoid rebuilding
        # nearest-neighbor indices for distant parts of a multi-million-point map.
        observed=np.asarray(p.points)
        lo=observed.min(axis=0)-2.;hi=observed.max(axis=0)+2.
        keep=np.all((reference_points>=lo)&(reference_points<=hi),axis=1)
        local_reference=cloud(reference_points[keep])
        local_reference.normals=o3d.utility.Vector3dVector(reference_normals[keep])
        if len(local_reference.points)<50:
            return position,rotation,False
        before=np.asarray(p.compute_point_cloud_distance(local_reference))
        # Reserve every fifth point to accept/reject registration independently of fit points.
        fit=p.select_by_index(list(range(0,len(p.points),5)),invert=True)
        hold=p.select_by_index(list(range(0,len(p.points),5)))
        result=reg.registration_icp(fit,local_reference,1.5,np.eye(4),
                   reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=.7)),
                   reg.ICPConvergenceCriteria(max_iteration=40))
        t=result.transformation
        before_hold=np.asarray(hold.compute_point_cloud_distance(local_reference))
        hold.transform(t)
        after=np.asarray(hold.compute_point_cloud_distance(local_reference))
        shift=float(np.linalg.norm(t[:3,:3]@position+t[:3,3]-position))
        angle=math.degrees(math.acos(np.clip((np.trace(t[:3,:3])-1)/2,-1,1)))
        accepted=bool(np.median(after)<np.median(before_hold)*.97 and shift<1.0 and angle<6 and result.fitness>.5)
        use=bool((np.median(after) if accepted else np.median(before))<.25)
        pose_records[stamp+':'+str(position.tolist())]={'translation_m':shift,'rotation_deg':angle,
                       'before_holdout_median_m':float(np.median(before_hold)),
                       'after_holdout_median_m':float(np.median(after)),
                       'accepted':accepted,'integrable':use,'transform':t.tolist() if accepted else np.eye(4).tolist()}
        return (t[:3,:3]@position+t[:3,3],t[:3,:3]@rotation,use) if accepted else (position,rotation,use)

    volume=o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel,sdf_trunc=voxel*4,
                    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,depth_sampling_stride=4)
    heldout=[];train=[]
    for index,row in enumerate(rows):
        # Hold out every 10th frame for independent depth-surface validation.
        (heldout if index%10==5 else train).append(row)
    platforms={};valid_pixels=0;rejected_frames=0
    for index,row in enumerate(train):
        with np.load(folder/row['file']) as frame:
            position,rotation,valid_pose=corrected_pose(frame)
            if not valid_pose:
                rejected_frames+=1
                continue
            axial,intrinsic,extrinsic=rgbd_geometry(frame['range_m'],frame['fov_deg'],
                         position,rotation,max_depth)
            rgbd=o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.ascontiguousarray(frame['rgb'])),o3d.geometry.Image(axial),
                depth_scale=1.,depth_trunc=max_depth,convert_rgb_to_intensity=False)
            volume.integrate(rgbd,intrinsic,extrinsic)
            valid_pixels+=int(np.count_nonzero(axial))
            platforms[row['name']]=platforms.get(row['name'],0)+1
        if index%20==0:
            print(f'TSDF: {index+1}/{len(train)} synchronized frames integrated',flush=True)
    mesh=volume.extract_triangle_mesh()
    mesh.remove_degenerate_triangles();mesh.remove_duplicated_triangles();mesh.remove_unreferenced_vertices()
    if not len(mesh.triangles):
        raise ValueError('TSDF produced no observed surfaces')
    # Discard tiny disconnected specks, never bridge holes with Poisson extrapolation.
    labels,counts,areas=mesh.cluster_connected_triangles()
    remove=np.asarray(counts)[np.asarray(labels)]<30
    removed=int(remove.sum())
    mesh.remove_triangles_by_mask(remove);mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals()
    support=constrain_surface(mesh,reference,support_distance) if support_distance is not None else None
    o3d.io.write_triangle_mesh(str(output/'surface_tsdf_ned.ply'),mesh)
    up=o3d.geometry.TriangleMesh(mesh)
    up.vertices=o3d.utility.Vector3dVector(np.asarray(mesh.vertices)*[1,1,-1])
    up.triangles=o3d.utility.Vector3iVector(np.asarray(mesh.triangles)[:,[0,2,1]])
    up.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(output/'surface_tsdf_neu.ply'),up)
    surface=cloud(np.asarray(mesh.vertices));surface.colors=mesh.vertex_colors
    o3d.io.write_point_cloud(str(output/'dense_colored_ned.ply'),surface)
    # Geometric quality against held-out sensor rays (not merely a larger point count).
    scene=o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    errors=[];rays_total=0;hits_total=0;heldout_used=0
    for row in heldout:
        with np.load(folder/row['file']) as frame:
            position,rotation,valid_pose=corrected_pose(frame)
            if not valid_pose:
                continue
            d=frame['range_m'][::8,::8];h,w=frame['range_m'].shape
            v,u=np.mgrid[0:h:8,0:w:8];f=w/(2*np.tan(np.deg2rad(float(frame['fov_deg']))/2))
            valid=(d>.5)&(d<max_depth)
            directions=np.stack([np.ones_like(u),(u-w/2)/f,(v-h/2)/f],axis=-1)[valid]
            directions=directions/np.linalg.norm(directions,axis=1,keepdims=True)
            directions=directions@rotation.T
            origins=np.broadcast_to(position,directions.shape)
            rays=np.concatenate([origins,directions],axis=1).astype(np.float32)
            if not len(rays): continue
            heldout_used+=1
            predicted=scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
            good=np.isfinite(predicted)
            rays_total+=len(rays);hits_total+=int(good.sum())
            errors.extend(np.abs(predicted[good]-d[valid][good]).tolist())
    e=np.array(errors)
    evaluation={'heldout_frames':len(heldout),'heldout_used_frames':heldout_used,
                'heldout_rejected_frames':len(heldout)-heldout_used,
                'rays':rays_total,'surface_hit_fraction':hits_total/max(rays_total,1),
                'absolute_range_error_median_m':float(np.median(e)) if len(e) else None,
                'absolute_range_error_p90_m':float(np.percentile(e,90)) if len(e) else None,
                'range_rmse_m':float(np.sqrt(np.mean(e**2))) if len(e) else None}
    quality_pass=bool(len(e)>100 and np.median(e)<.20 and np.percentile(e,90)<.8 and hits_total/max(rays_total,1)>.75)
    report={'method':'LiDAR-anchored robust RGB-D pose refinement + scalable TSDF + observed zero-crossing extraction',
            'pose_source':'simulator pose prior refined per-frame against independent LiDAR; not autonomous SLAM',
            'quality_status':'passed' if quality_pass else 'quality_check_failed','voxel_m':voxel,'sdf_trunc_m':voxel*4,
            'integration_frames':sum(platforms.values()),'rejected_pose_frames':rejected_frames,
            'refined_poses':sum(r['accepted'] for r in pose_records.values()),
            'platform_frames':platforms,'integrated_valid_pixels':valid_pixels,
            'mesh_vertices':len(mesh.vertices),'mesh_triangles':len(mesh.triangles),
            'removed_tiny_triangles':removed,'evaluation':evaluation,'capture_run':folder.name,
            'lidar_surface_support':support,
            'limitation':'Only observed surfaces; underside/occluded regions remain incomplete. No texture atlas or pose SLAM.'}
    write_json(output/'quality.json',report)
    write_json(output/'pose_refinement.json',pose_records)
    layers=[mesh_layer('TSDF 彩色表面',mesh),point_layer('稠密彩色点云',surface)]
    old=folder/'map_fused.pcd'
    if old.exists():
        layers.append(point_layer('本次激光基线',o3d.io.read_point_cloud(str(old))))
    html=viewer(output/'map_dense_3d.html',layers,report['quality_status'],
                '同步 RGB-D · TSDF 表面融合 · 顶点颜色 · 预览网格简化至最多 18 万面；完整网格见 PLY。')
    return html,report
