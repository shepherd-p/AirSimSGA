"""Read-only audit of archived sensor records and their derived figure inputs.

No simulator connection, scene loading, figure regeneration, or input writes.
The optional LiDAR replay runs in the isolated Open3D environment.
"""
import os
os.environ.setdefault('OMP_NUM_THREADS','4')
import argparse
from collections import Counter
import csv
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import numpy as np
from ag.math3d import pose_parts


def require(condition,message):
    if not condition:raise ValueError(message)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def rows(path):
    with path.open(encoding='utf-8',newline='') as stream:return list(csv.DictReader(stream))


def member(root,relative):
    path=(root/relative.replace('\\','/')).resolve()
    require(path.is_relative_to(root.resolve()),f'Path outside run: {relative}')
    return path


def snapshots(run):
    results=[]
    for base in [run,run/'plaza_results']:
        expected=read_json(base/'source_sha256.json')
        for name,digest in expected.items():
            path=member(base/'source_snapshot',name)
            require(hashlib.sha256(path.read_bytes()).hexdigest()==digest,f'Historical source mismatch: {path}')
        results.append({'path':str(base.relative_to(run)),'verified_source_files':len(expected)})
    return results


def audit(run,photo):
    from PIL import Image
    manifest=read_json(run/'manifest.json');scan_report=read_json(run/'lidar_capture.json')
    require(manifest['status']=='completed','Run did not complete')
    scans=rows(run/'scans.csv');counts=Counter();returns=Counter();seen=set();files=set()
    transform_max=0.;time_bounds={};previous={}
    for index,row in enumerate(scans):
        name=row['name'];stamp=int(row['time_ns']);key=(name,stamp)
        require(key not in seen,f'Duplicate scan: {key}');seen.add(key)
        require(stamp>previous.get(name,-1),f'Non-increasing scan timestamp: {key}');previous[name]=stamp
        path=member(run,row['file']);files.add(path)
        with np.load(path,allow_pickle=False) as scan:
            local=scan['local_points'];world=scan['world_points']
            pos=scan['sensor_position_ned'];rot=scan['sensor_rotation_ned']
            require(local.shape==world.shape and world.ndim==2 and world.shape[1]==3,f'Scan shape: {path}')
            require(len(world)==int(row['points']) and int(scan['time_ns'])==stamp,f'Scan metadata: {path}')
            require(np.isfinite(local).all() and np.isfinite(world).all(),f'Nonfinite scan: {path}')
            p,r=pose_parts(json.loads(row['pose']))
            require(np.allclose(pos,p,atol=1e-10,rtol=0) and np.allclose(rot,r,atol=1e-10,rtol=0),f'Scan pose: {path}')
            difference=float(np.max(np.abs(world-(local@rot.T+pos).astype('f4')))) if len(world) else 0.
            transform_max=max(transform_max,difference)
            require(difference<=1e-5,f'Sensor/world coordinate mismatch: {path}')
            returns[name]+=len(world)
        counts[name]+=1
        time_bounds.setdefault(name,[stamp,stamp])[1]=stamp
        if index%1000==999:print(f'Audited {index+1}/{len(scans)} LiDAR packets',flush=True)
    require(files==set(path.resolve() for path in (run/'raw').rglob('*.npz')),'Unindexed or missing LiDAR archive files')
    require(len(scans)==scan_report['saved']==scan_report['enqueued']==scan_report['received'],'Scan count mismatch')
    for name in counts:
        require(counts[name]==scan_report[name+'_saved'] and returns[name]==scan_report[name+'_points'],'Per-platform scan totals mismatch')
    require(scan_report['queue_drops']==0 and not scan_report['errors'],'Capture reported archive errors')

    rgbd=rows(run/'rgbd_frames.csv');rgb_counts=Counter();rgbfiles=set();usage=[];pair_ms=0.
    pose_records=read_json(run/'plaza_results/surface/pose_refinement.json')
    for index,row in enumerate(rgbd):
        path=member(run,row['file']);rgbfiles.add(path)
        with np.load(path,allow_pickle=False) as frame:
            d=frame['range_m'];color=frame['rgb'];position=frame['camera_position_ned'];rot=frame['camera_rotation_ned']
            require(color.shape==(*d.shape,3) and color.dtype==np.uint8,f'RGB-D dimensions/type: {path}')
            require(np.isfinite(d).all() and (d>=0).all(),f'Invalid saved range: {path}')
            require(int(frame['time_ns'])==int(row['time_ns']) and np.count_nonzero(d)==int(row['valid_pixels']),f'RGB-D metadata: {path}')
            require(np.allclose(position,json.loads(row['position_ned']),atol=1e-9,rtol=0),'RGB-D position metadata mismatch')
            require(np.allclose(rot@rot.T,np.eye(3),atol=1e-6) and np.linalg.det(rot)>.999,'RGB-D rotation invalid')
            key=str(int(frame['time_ns']))+':'+str(position.tolist())
            rec=pose_records.get(key)
            role='heldout' if index%10==5 else 'integration_candidate'
            usable=bool(rec and rec['integrable'])
            usage.append({'csv_row_0based':index,'name':row['name'],'time_ns':row['time_ns'],
                          'file':row['file'],'role':role,'used':usable,
                          'pose_refinement_record_present':rec is not None,
                          'pose_refined':bool(rec and rec['accepted'])})
        pair_ms=max(pair_ms,abs(int(row['time_ns'])-int(row['color_time_ns']))/1e6)
        rgb_counts[row['name']]+=1
    require(rgbfiles==set(path.resolve() for path in (run/'rgbd').rglob('*.npz')),'Unindexed RGB-D files')
    require(dict(rgb_counts)==manifest['rgbd_frame_counts'] and pair_ms<=15.0001,'RGB-D count/synchronization mismatch')
    quality=read_json(run/'plaza_results/surface/quality.json')
    integrated=sum(u['role']=='integration_candidate' and u['used'] for u in usage)
    heldout=sum(u['role']=='heldout' for u in usage)
    rejected=sum(u['role']=='integration_candidate' and not u['used'] for u in usage)
    require(integrated==quality['integration_frames'] and rejected==quality['rejected_pose_frames'] and heldout==quality['evaluation']['heldout_frames'],'Recorded TSDF frame roles disagree')

    tracks=rows(run/'trajectories.csv');trajectory={}
    for name in ('Drone1','Car1'):
        chosen=[r for r in tracks if r['name']==name]
        pts=np.array([[float(r[k]) for k in ('x','y','z')] for r in chosen])
        times=np.array([float(r['time']) for r in chosen])
        require(np.isfinite(pts).all() and (np.diff(times)>=0).all(),'Invalid trajectory')
        distance=float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum())
        horizontal=float(np.linalg.norm(np.diff(pts[:,:2],axis=0),axis=1).sum())
        require(abs(distance-manifest['path_lengths_m'][name])<1e-6 and abs(horizontal-manifest['horizontal_path_lengths_m'][name])<1e-6,'Trajectory length mismatch')
        trajectory[name]={'rows':len(chosen),'sim_time_span_seconds':[float(times[0]),float(times[-1])],
                          'distance_3d_m':distance,'distance_horizontal_m':horizontal}

    images=rows(run/'images.csv')
    for row in images:
        with Image.open(member(run,row['file'])) as im:im.verify()
    csv_counts={p.stem:len(rows(p)) for p in run.glob('*.csv')}
    point_parts=[];source_parts=[]
    for index,name in enumerate(('Drone1','Car1')):
        with np.load(run/f'plaza_results/lidar_{name}_timed.npz') as data:p=data['points_ned']*[1,1,-1]
        p=p[np.all((p>=[-5,25,-18])&(p<=[65,88,30]),axis=1)]
        point_parts.append(p);source_parts.append(np.full(len(p),index,dtype='u1'))
    with np.load(run/'plaza_results/structural_display.npz') as data:
        indices=data['input_point_indices']
        require(len(np.unique(indices))==len(indices),'Repeated display indices')
        require(np.array_equal(data['points_neu'],np.vstack(point_parts)[indices]),'Structural view changed coordinates')
        require(np.array_equal(data['source'],np.concatenate(source_parts)[indices]),'Structural view changed platform labels')
        displayed=len(indices)
    scene=read_json(photo/'scene_figure.json')
    require((photo/'scene_overview.png').read_bytes()==(photo/scene['original_image']).read_bytes(),'Original scene picture altered')
    require(hashlib.sha256((photo/'scene_overview.png').read_bytes()).hexdigest()==scene['original_sha256'],'Scene source checksum mismatch')
    for path in photo.glob('*.png'):
        with Image.open(path) as im:im.verify()
    report={'audit_time_utc':datetime.now(timezone.utc).isoformat(),'status':'passed',
            'mapping_run':run.name,'scene_session':photo.name,'historical_source_snapshots':snapshots(run),
            'lidar':{'verified_packets':len(scans),'platform_packets':dict(counts),
                     'verified_valid_returns':sum(returns.values()),'platform_returns':dict(returns),
                     'per_packet_transform_max_abs_difference_m':transform_max,'sim_time_bounds_ns':time_bounds,
                     'local_queue_drops':scan_report['queue_drops'],'transport_loss':scan_report['transport_loss']},
            'rgbd':{'verified_frame_groups':len(rgbd),'platform_frame_groups':dict(rgb_counts),
                    'maximum_rgb_range_time_difference_ms':pair_ms,'integration_frames':integrated,
                    'rejected_integration_frames':rejected,'heldout_frames':heldout,
                    'note':'Saved range pixels already masked; RGB-D is selected keyframes, not all camera messages'},
            'trajectory':trajectory,'csv_rows':csv_counts,'verified_jpeg_snapshots':len(images),
            'verified_structure_display_points':displayed,'original_scene_frame_byte_identical':True,
            'not_performed':['Re-running flight','Reintegrating the TSDF mesh','Independent third-party capture attestation'],
            'limitations':['Simulated sensors and simulator pose prior; not physical sensing or autonomous SLAM.',
                           'LiDAR files omit invalid/no-return and robot returns. Discarded returns cannot be recovered.',
                           'RGB-D depth is range-filtered, robot-masked, and discontinuity-masked before saving.',
                           'Trajectory is logged at the experiment loop, not every simulator pose callback.',
                           'AVI files are 1 fps sampled camera summaries, not continuous original capture.',
                           'Holdout depths come from this simulation run; camera pose correction still uses LiDAR from that run.',
                           'Scene overview was photographed in a separate illustrative session.']}
    return report,usage


def replay_lidar(run):
    import open3d as o3d
    from ag.plaza_reconstruction import aggregate
    result={}
    for name in ('Drone1','Car1'):
        points,times,stats=aggregate(run,name,size=.1)
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        pc,indices=pc.remove_statistical_outlier(nb_neighbors=20,std_ratio=3.)
        replayed=np.asarray(pc.points);replayed_times=times[indices]
        with np.load(run/f'plaza_results/lidar_{name}_timed.npz') as saved:
            require(replayed.shape==saved['points_ned'].shape,f'Replayed point count mismatch: {name}')
            error=float(np.max(np.abs(replayed-saved['points_ned'])))
            time_error=float(np.max(np.abs(replayed_times-saved['time_sec'])))
        require(error<1e-7 and time_error<1e-7,f'Replayed fused cloud mismatch: {name}, {error} m')
        result[name]={**stats,'replayed_retained_points':len(replayed),'maximum_coordinate_difference_m':error,
                      'maximum_first_observation_time_difference_sec':time_error}
    return {'audit_time_utc':datetime.now(timezone.utc).isoformat(),'status':'passed','platforms':result,
            'method':'All archived scans -> 10 cm weighted centroids -> statistical outlier filter -> compare every saved fused point and timestamp',
            'input_files_modified':False}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mapping-dir',type=Path,required=True);p.add_argument('--scene-dir',type=Path)
    p.add_argument('--report',type=Path,required=True);p.add_argument('--replay-lidar',action='store_true')
    a=p.parse_args();run=a.mapping_dir.resolve();destination=a.report.resolve()
    require(not destination.is_relative_to(run),'Write audit reports outside the preserved mapping run')
    destination.parent.mkdir(parents=True,exist_ok=True)
    if a.replay_lidar:report=replay_lidar(run)
    else:
        if a.scene_dir is None:p.error('--scene-dir is required for the record audit')
        require(not destination.is_relative_to(a.scene_dir.resolve()),'Write reports outside the preserved scene session')
        report,usage=audit(run,a.scene_dir.resolve())
        with destination.with_name(destination.stem+'_frame_usage.csv').open('w',encoding='utf-8',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(usage[0]));writer.writeheader();writer.writerows(usage)
    destination.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)
