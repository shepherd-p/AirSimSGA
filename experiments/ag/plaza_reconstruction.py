"""Memory-bounded measured-return aggregation, retaining source and first observation."""
import csv
import base64
import json
import numpy as np
import open3d as o3d
from .reconstruction import cloud,point_layer,binary,mesh_layer
from .storage import ROOT,write_json


def plaza_preview(points):
    """Presentation ROI only. Full PLY and timed NPZ retain the entire scan."""
    keep=np.all((points>=[-5,25,-30])&(points<=[65,88,18]),axis=1)
    p=points[keep]
    if not len(p):raise ValueError('No measurements inside plaza preview ROI')
    return p[np.linspace(0,len(p)-1,min(len(p),800000),dtype=int)]


def reduce_voxels(points, weights, times, size):
    keys=np.floor(points/size).astype(np.int32)
    # Scalar integer IDs avoid expensive structured-array sorting of millions
    # of XYZ tuples. The packing is lossless within this batch's bounds.
    lower=keys.min(axis=0).astype(np.int64)
    spans=keys.max(axis=0).astype(np.int64)-lower+1
    if int(spans[0])*int(spans[1])*int(spans[2]) < 2**63:
        shifted=keys.astype(np.int64)-lower
        packed=(shifted[:,0]*spans[1]+shifted[:,1])*spans[2]+shifted[:,2]
        _,inv=np.unique(packed,return_inverse=True)
    else:
        _,inv=np.unique(keys,axis=0,return_inverse=True)
    counts=np.bincount(inv,weights=weights)
    sums=np.column_stack([np.bincount(inv,weights=points[:,i]*weights) for i in range(3)])
    first=np.full(len(counts),np.inf)
    np.minimum.at(first,inv,times)
    return sums/counts[:,None],counts,first


def aggregate(folder, name, size=.1):
    points=np.empty((0,3));weights=np.empty(0);times=np.empty(0)
    pending=[];pending_times=[];raw=0
    files=sorted((folder/'raw'/name).glob('*.npz'))
    for i,f in enumerate(files):
        with np.load(f) as d:
            p=d['world_points'].astype(float)
        p=p[np.isfinite(p).all(axis=1)]
        raw+=len(p);pending.append(p);pending_times.append(np.full(len(p),int(f.stem)/1e9))
        if len(pending)>=100 or i==len(files)-1:
            batch=np.vstack(pending)
            points,weights,times=reduce_voxels(np.vstack([points,batch]),
                np.r_[weights,np.ones(len(batch))],np.r_[times,np.concatenate(pending_times)],size)
            pending=[];pending_times=[]
            if i%500==499 or i==len(files)-1:
                print(f'{name}: {i+1}/{len(files)} packets, {len(points):,} occupied voxels',flush=True)
    if not len(points):raise ValueError('No valid scans for '+name)
    return points,times,{'scans':len(files),'raw_returns':raw,'voxel_centroids':len(points)}


def lidar_products(folder,out):
    layers=[];stats={};all_points=[];preview={}
    for name,color in [('Drone1',[.15,.55,1.]),('Car1',[.18,.85,.4])]:
        p,t,s=aggregate(folder,name)
        pc=cloud(p)
        pc,indices=pc.remove_statistical_outlier(nb_neighbors=20,std_ratio=3.)
        p=np.asarray(pc.points);t=t[indices]
        pc.paint_uniform_color(color)
        s['retained_points']=len(p);stats[name]=s
        o3d.io.write_point_cloud(str(out/f'lidar_{name}_ned.ply'),pc)
        np.savez_compressed(out/f'lidar_{name}_timed.npz',points_ned=p,time_sec=t)
        all_points.append(p)
        sample=plaza_preview(p)
        small=cloud(sample);small.paint_uniform_color(color)
        item=point_layer('无人机实测激光' if name=='Drone1' else '无人车实测激光',small)
        item['full_count']=len(p)
        layers.append(item)
        preview[name]=sample
    fused=cloud(np.vstack(all_points))
    fused.colors=o3d.utility.Vector3dVector(np.vstack([np.tile(c,(len(p),1)) for p,c in zip(all_points,[[.15,.55,1.],[.18,.85,.4]])]))
    o3d.io.write_point_cloud(str(out/'lidar_fused_ned.ply'),fused)
    up=o3d.geometry.PointCloud(fused);up.points=o3d.utility.Vector3dVector(np.asarray(up.points)*[1,1,-1])
    o3d.io.write_point_cloud(str(out/'lidar_fused_neu.ply'),up)
    o3d.io.write_point_cloud(str(folder/'lidar_reference_ned.ply'),fused.voxel_down_sample(.15))
    pp=cloud(np.vstack(list(preview.values())))
    pp.colors=o3d.utility.Vector3dVector(np.vstack([np.tile(c,(len(p),1)) for p,c in zip(preview.values(),[[.15,.55,1.],[.18,.85,.4]])]))
    layers.insert(0,point_layer('空地融合实测激光 · 蓝 UAV / 绿 UGV',pp))
    np.savez_compressed(out/'plot_points.npz',**preview)
    write_json(out/'lidar_quality.json',{'voxel_m':.1,'sensors':stats,
      'pose_source':'Per-packet simulator sensor pose; shared NED frame, no fictitious point timestamps or trajectory correction',
      'filter':'10 cm weighted centroid, invalid/self/moving robot rejection, statistical outliers; no synthetic fill'})
    return layers


def trajectory_data(folder):
    rows=list(csv.DictReader((folder/'trajectories.csv').open(encoding='utf-8')))
    tracks=[]
    for n,color in [('Drone1',[1.,.32,.2]),('Car1',[1.,.83,.15])]:
        selected=[r for r in rows if r['name']==n]
        tracks.append({'name':n,'color':color,'positions':[[float(r['x']),float(r['y']),-float(r['z'])] for r in selected],
                       'times':[float(r['time']) for r in selected]})
    return tracks


def existing_lidar_layers(out):
    layers=[];clouds=[];preview={}
    for n,c in [('Drone1',[.15,.55,1.]),('Car1',[.18,.85,.4])]:
        with np.load(out/f'lidar_{n}_timed.npz') as archive:
            p=plaza_preview(archive['points_ned'])
        preview[n]=p
        pc=cloud(p);pc.paint_uniform_color(c);clouds.append(pc)
        layers.append(point_layer('无人机实测激光' if n=='Drone1' else '无人车实测激光',pc))
    np.savez_compressed(out/'plot_points.npz',**preview)
    return [point_layer('空地融合实测激光 · 蓝 UAV / 绿 UGV',clouds[0]+clouds[1])]+layers


def combined_viewer(folder,out,layers,surface_html):
    # Crop the FULL mesh before simplifying. Reusing the old 180k-face browser
    # preview permanently discarded local detail and blurred vertex colors.
    surface=json.loads((surface_html.parent/'quality.json').read_text(encoding='utf-8'))
    full=o3d.io.read_triangle_mesh(str(surface_html.parent/'surface_tsdf_ned.ply'))
    v=np.asarray(full.vertices);f=np.asarray(full.triangles)
    inside=np.all((v>=[-5,25,-30])&(v<=[65,88,18]),axis=1)
    full.remove_triangles_by_mask(~np.all(inside[f],axis=1));full.remove_unreferenced_vertices()
    mesh=mesh_layer('TSDF 彩色表面 · 高清预览',full,max_triangles=1000000)
    mesh['full_faces']=surface['mesh_triangles'];mesh['full_vertices']=surface['mesh_vertices']
    layers.append(mesh)
    np.savez_compressed(out/'plot_surface.npz',
        vertices=np.frombuffer(base64.b64decode(mesh['positions_b64']),dtype='<f4').reshape(-1,3),
        faces=np.frombuffer(base64.b64decode(mesh['triangles_b64']),dtype='<u4').reshape(-1,3),
        colors=np.frombuffer(base64.b64decode(mesh['colors_b64']),dtype='<f4').reshape(-1,3))
    data=dict(run=folder.name,status=surface['quality_status'],layers=layers,defaultColor='rgb',
              trajectories=trajectory_data(folder),
              description='广场区域预览 · 完整扫描见 PLY。蓝/绿：UAV/UGV 激光；红/黄：实际轨迹。滑块回放轨迹，地图为离线融合结果。')
    template=(ROOT/'ag/map3d_template.html').read_text(encoding='utf-8')
    target=out/'map_plaza_3d.html'
    target.write_text(template.replace('/*__MAP_DATA__*/',json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('<','\\u003c')),encoding='utf-8')
    return target
