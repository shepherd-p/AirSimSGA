"""Offline consistency audit of cooperative-study evidence; never connects to UE.

Checks consistency and reproducibility, not independent authenticity. LiDAR NPZ
files contain filtered returns, and camera bundles are selected exposures.
"""
from collections import Counter
from datetime import datetime,timezone
import csv
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import cv2
import numpy as np
from .math3d import depth_world,pose_parts
from .storage import write_json
from .contact import is_ground_contact


def rows(path):
    with path.open(encoding='utf-8',newline='') as f:return list(csv.DictReader(f))


def json_file(path):return json.loads(path.read_text(encoding='utf-8'))


def require(condition,message):
    if not condition:raise ValueError(message)


def member(root,name):
    path=(root/name).resolve()
    require(path.is_relative_to(root.resolve()),'Indexed path escapes run: '+name)
    require(path.is_file(),'Missing indexed file: '+name)
    return path


def same(actual,expected,label,tolerance=1e-7):
    require(np.allclose(actual,expected,atol=tolerance,rtol=0,equal_nan=True),'Mismatch: '+label)


def snapshot_module(run,name):
    spec=importlib.util.spec_from_file_location('_audit_'+name,run/'source_snapshot'/'ag'/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def decode(payload,metadata):
    h,w=int(metadata['height']),int(metadata['width']);encoding=metadata['encoding']
    if encoding=='16UC1':return payload.view('>u2' if metadata.get('big_endian') else '<u2').reshape(h,w)
    if encoding=='PNG':
        result=cv2.imdecode(payload,cv2.IMREAD_COLOR)
        require(result is not None,'PNG camera payload does not decode');return result
    if encoding=='AVX':return cv2.cvtColor(payload.reshape(h,w,4),cv2.COLOR_RGB2BGR)
    if encoding=='GRAY8':return payload.reshape(h,w,1)
    require(encoding=='BGR','Unsupported camera encoding: '+encoding)
    return payload.reshape(h,w,3)


def segmentation_boxes(seg,colors):
    result={}
    for name,color in colors.items():
        mask=cv2.inRange(seg[:,:,:3],np.array(color,dtype='u1'),np.array(color,dtype='u1'))
        count=int(cv2.countNonZero(mask))
        if count:
            x,y,w,h=cv2.boundingRect(mask);result[name]={'pixels':count,'bbox':[x,y,x+w-1,y+h-1]}
    return result


class StudyAudit:
    def __init__(self,run):
        self.run=Path(run).resolve();self.manifest=json_file(self.run/'manifest.json')
        self.config=json_file(self.run/'config.json');self.task=self.manifest['task']
        self.metrics=json_file(self.run/'study_results/metrics.json')
        self.trajectory_rows=rows(self.run/'trajectories.csv');self.tracks={};self.times={}
        for name in sorted({r['name'] for r in self.trajectory_rows}):
            selected=[r for r in self.trajectory_rows if r['name']==name]
            self.tracks[name]=np.array([[float(r[k]) for k in 'xyz'] for r in selected])
            self.times[name]=np.array([float(r['time']) for r in selected])
        self.tracking_rows=rows(self.run/'tracking.csv') if self.task=='tracking' else []
        self.publication_rows=rows(self.run/'study_results/trajectories_topic.csv')
        self.publication_tracks={n:np.array([[float(r[a]) for a in 'xyz'] for r in self.publication_rows if r['name']==n]) for n in self.tracks}
        self.report={'schema_version':1,'run':self.run.name,'task':self.task,'status':'running',
                     'checked_utc':datetime.now(timezone.utc).isoformat(),'checks':{},
                     'limits':['Local consistency and replay do not prove independent authenticity.',
                         'LiDAR archives retain finite nonzero returns after removal of segmentation IDs 240 and 241; original excluded returns are not retained.',
                         'Camera archive contains selected synchronized exposures, not every incoming camera message.',
                         'Pose/collision archive covers the capture window, not startup or cleanup.',
                         'Zero application queue drops does not prove zero network loss; messages lack sequence IDs.',
                         'Single successful run per task; no claim of statistical reliability or superiority to the paper.']}

    def provenance(self):
        counts={}
        for base in (self.run,self.run/'study_results'):
            hashes=json_file(base/'source_sha256.json')
            for name,expected in hashes.items():
                actual=hashlib.sha256(member(base/'source_snapshot',name).read_bytes()).hexdigest()
                require(actual==expected,'Source snapshot hash differs: '+name)
            counts[base.name]=len(hashes)
        processing=json_file(self.run/'study_results/processing.json')
        same_hash=hashlib.sha256((self.run/'manifest.json').read_bytes()).hexdigest()
        require(processing['input_manifest_sha256']==same_hash,'Figure input manifest changed')
        scene=json_file(self.run/'loaded_scene.json')
        controlled=[a['name'] for a in scene['actors'] if a['name']!='Observer']
        require(set(controlled)==set(self.tracks),'Loaded scene/trajectory actor mismatch')
        require(len(controlled)==self.config.get('controlled_actor_count',{'navigation':2,'tracking':3,'formation':7}[self.task]),'Incorrect actor count')
        self.report['checks']['provenance']={'verified_snapshot_files':counts,'controlled_actors':controlled}

    def lidar(self):
        index=rows(self.run/'scans.csv');receipt=json_file(self.run/'lidar_capture.json')
        require(not receipt['errors'] and receipt.get('queue_drops',0)==0,'LiDAR capture reported errors/drops')
        require(len(index)==receipt['saved'],'LiDAR saved count/index mismatch')
        require(receipt['received']==receipt['saved']+receipt.get('duplicates',0),'LiDAR receipt accounting mismatch')
        seen=set();points=Counter();counts=Counter();maximum=0.
        for r in index:
            key=(r['name'],r['time_ns']);require(key not in seen,'Duplicate scan index');seen.add(key)
            path=member(self.run,r['file'])
            with np.load(path,allow_pickle=False) as d:
                local=d['local_points'];world=d['world_points'];p=d['sensor_position_ned'];rotation=d['sensor_rotation_ned']
                require(local.shape==world.shape==(int(r['points']),3),'LiDAR array shape mismatch')
                require(np.isfinite(local).all() and np.isfinite(world).all(),'Invalid archived LiDAR coordinates')
                require(int(d['time_ns'])==int(r['time_ns']),'LiDAR timestamp mismatch')
                origin,rot=pose_parts(json.loads(r['pose']));same(p,origin,'LiDAR origin');same(rotation,rot,'LiDAR rotation')
                reproduced=(local@rot.T+origin).astype('f4');same(reproduced,world,'LiDAR capture-pose transform',2e-5)
                if len(world):maximum=max(maximum,float(np.max(abs(reproduced-world))))
                points[r['name']]+=len(world);counts[r['name']]+=1
        require(len(list((self.run/'raw').rglob('*.npz')))==len(index),'Unindexed LiDAR files')
        for name,count in counts.items():
            require(count==receipt[name+'_saved'] and points[name]==receipt[name+'_points'],'Per-platform LiDAR totals differ')
        self.report['checks']['lidar']={'unique_packets':len(index),'filtered_points':dict(points),
            'packets_by_platform':dict(counts),'duplicate_receipts':receipt.get('duplicates',0),
            'maximum_transform_difference_m':maximum,'application_queue_drops':0}

    def cameras(self):
        index=rows(self.run/'camera_frames.csv');receipt=json_file(self.run/'camera_archive.json')
        require(not receipt['errors'] and receipt['counts'].get('queue_drops',0)==0,'Camera archive errors/drops')
        counts=Counter();seen=set();matched=Counter();max_delta=0.;replay_error=0.
        observed={}
        for r in self.tracking_rows:
            for camera,prefix in [('Drone1','uav'),('Car1','ugv')]:
                if r[prefix+'_observation_time']:
                    key=(camera,round(float(r[prefix+'_observation_time'])*1e9))
                    observed[key]=np.array([float(r[prefix+'_estimate_'+a]) for a in 'xyz'])
        for i,row in enumerate(index):
            camera=row['camera'];stamp=int(row['time_ns']);key=(camera,stamp)
            require(key not in seen,'Duplicate camera index');seen.add(key);counts[camera]+=1
            with np.load(member(self.run,row['file']),allow_pickle=False) as d:
                metadata=json.loads(str(d['metadata_json']));decoded={}
                for name,m in metadata.items():decoded[name]=decode(d[name],m)
                rgb=decoded['scene_camera'];png=cv2.imread(str(member(self.run,row['image'])),cv2.IMREAD_COLOR)
                require(np.array_equal(rgb,png),'PNG differs from original scene camera payload')
                require(rgb.shape[:2]==(int(row['height']),int(row['width'])),'Camera dimensions mismatch')
                require(int(metadata['scene_camera']['time_stamp'])==stamp,'Scene camera timestamp mismatch')
                delta=max(abs(int(m['time_stamp'])-stamp) for m in metadata.values())/1e6
                same(delta,float(row['pair_delta_ms']),'Camera pair time');require(delta<=30,'Camera pair exceeds tolerance')
                max_delta=max(max_delta,delta)
                boxes=segmentation_boxes(decoded['segmentation_camera'],receipt['segmentation_bgr'])
                require(boxes==json.loads(row['visible_robots']),'Segmentation does not reproduce annotated bounding boxes')
                if self.task=='tracking' and camera in ('Drone1','Car1'):
                    depth=decoded['depth_camera'].astype(float)/1000
                    color=np.array(receipt['segmentation_bgr']['Target'],dtype='u1')
                    mask=np.all(decoded['segmentation_camera'][:,:,:3]==color,axis=2)&(depth>.1)&(depth<65.)
                    key=(camera,int(metadata['depth_camera']['time_stamp']))
                    if key in observed and int(mask.sum())>=self.config['tracking']['minimum_pixels']:
                        cloud=depth_world(depth,metadata['depth_camera'],stride=1,mask=mask)
                        estimate=np.median(cloud,axis=0);error=float(np.max(abs(estimate-observed[key])))
                        same(estimate,observed[key],'Tracking observation replay from depth and segmentation',1e-7)
                        replay_error=max(replay_error,error);matched[camera]+=1
            if (i+1)%200==0:print(f'{self.task}: audited {i+1}/{len(index)} camera bundles',flush=True)
        for name,count in counts.items():require(count==receipt['counts'][name],'Camera indexed count mismatch')
        require(len(list((self.run/'sensor_frames').rglob('*.npz')))==len(index),'Unindexed camera bundles')
        require(len(list((self.run/'sensor_frames').rglob('*.png')))==len(index),'Unindexed camera PNGs')
        if self.task=='tracking':require(all(matched[n]>0 for n in ('Drone1','Car1')),'No raw frames overlap tracking observations')
        self.frames={(r['camera'],r['time_ns']):r for r in index}
        self.report['checks']['cameras']={'original_payload_bundles':len(index),'lossless_pngs_equal_to_payload':len(index),
            'frames_by_camera':dict(counts),'maximum_pair_delta_ms':max_delta,'application_queue_drops':0,
            'unsynchronized_selection_attempts':receipt['counts'].get('unsynchronized_attempts',0),
            'tracking_observations_replayed_from_selected_frames':dict(matched),'maximum_observation_replay_difference_m':replay_error}

    def poses(self):
        receipt=json_file(self.run/'topic_archive.json');counts=Counter();found=set();topics={};impacts=0
        require(not receipt['errors'] and receipt['counts'].get('queue_drops',0)==0,'Pose/collision capture errors')
        wanted={(r['name']+'/pose',int(r['time_ns'])):np.array([float(r[k]) for k in 'xyz']) for r in self.publication_rows}
        with gzip.open(self.run/'pose_collision_topics.jsonl.gz','rt',encoding='utf-8') as stream:
            for line in stream:
                record=json.loads(line);key=record['stream'];m=record['message'];counts[key]+=1;stamp=int(m['time_stamp'])
                if key.endswith('/pose'):
                    p=[m['position'][a] for a in 'xyz'];topics.setdefault(key,[]).append([stamp/1e9]+p)
                    if (key,stamp) in wanted:
                        same(p,wanted[(key,stamp)],'Publication trajectory/original capture-stamped pose')
                        found.add((key,stamp))
                elif key.endswith('/collision') and not is_ground_contact(m):impacts+=1
        require(dict(counts)=={k:v for k,v in receipt['counts'].items() if k!='queue_drops'},'Pose/collision receipt totals differ')
        require(len(found)==len(wanted),'Published trajectory contains a pose absent from original topic archive')
        require(impacts==0,'Raw collision topics contain non-ground contact')
        topics={k:np.array(v) for k,v in topics.items()};errors=[];skews=[];startup_mismatches=[]
        for row in self.trajectory_rows:
            a=topics[row['name']+'/pose'];t=float(row['time']);p=np.array([float(row[k]) for k in 'xyz'])
            candidates=a[abs(a[:,0]-t)<=.6]
            require(len(candidates)>0,'Controller pose lies outside archived capture window')
            distance=np.linalg.norm(candidates[:,1:]-p,axis=1);error=float(distance.min())
            if error>=.01:
                if self.task=='navigation' and row['name']=='Drone1' and row['phase'] in ('initialization','aerial_scan'):
                    startup_mismatches.append((t,error))
                    continue
                raise ValueError('Controller object coordinate is inconsistent with nearby physics poses')
            close=distance<=error+1e-5
            skew=float(np.min(abs(candidates[close,0]-t)));errors.append(error);skews.append(skew)
        if startup_mismatches:
            cutoff=max(t for t,_ in startup_mismatches)
            selected=rows(self.run/'navigation_selected_scans.csv')
            require(all(int(r['points'])==0 for r in selected if int(r['time_ns'])/1e9<=cutoff),
                    'UAV pose mismatch overlaps nonempty selected navigation LiDAR')
            self.report['limits'].append('During UAV takeoff, GetObjectPose lagged the physics pose topic through '+
                f'{cutoff:.3f} simulation seconds; all navigation-selected LiDAR packets in that interval had zero usable returns.')
        self.report['checks']['pose_collision_topics']={'records':sum(counts.values()),'streams':dict(counts),
            'publication_poses_reproduced_at_exact_capture_timestamp':len(found),'raw_non_ground_collisions':impacts,
            'uav_zero_return_takeoff_pose_mismatches':{'samples':len(startup_mismatches),
                'last_sim_time_sec':max((t for t,_ in startup_mismatches),default=None),
                'maximum_residual_m':max((e for _,e in startup_mismatches),default=0.)},
            'controller_service_pose_neighborhood_check':{'window_sec':.6,'maximum_position_residual_m':max(errors),
                'maximum_matching_time_offset_sec':max(skews),'p95_matching_time_offset_sec':float(np.percentile(skews,95)),
                'note':'GetObjectPose returns a rendered-actor position; its separate clock query is not the physics sample time. Publication uses exact-timestamp pose topics.'}}

    def trajectories(self):
        result={}
        for name,points in self.tracks.items():
            require(np.all(np.diff(self.times[name])>=0),'Nonmonotonic trajectory time')
            horizontal=float(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1).sum())
            full=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
            same(horizontal,self.manifest['horizontal_path_lengths_m'][name],name+' XY path length')
            same(full,self.manifest['path_lengths_m'][name],name+' 3D path length')
            result[name]={'samples':len(points),'horizontal_length_m':horizontal,'length_3d_m':full}
        collision_rows=rows(self.run/'collisions.csv') if (self.run/'collisions.csv').exists() else []
        impacts=sum(r['ground_contact']!='True' for r in collision_rows)
        require(impacts==self.manifest['collision_count']==0,'Obstacle collision was recorded')
        self.report['checks']['trajectories']={'platforms':result,'obstacle_collisions':impacts,
            'ground_contact_events_in_control_log':len(collision_rows)-impacts}

    def navigation(self):
        geometry=snapshot_module(self.run,'math3d');planning=snapshot_module(self.run,'planning')
        planner_class=snapshot_module(self.run,'study_planning').ObservedGroundPlanner
        batches=[]
        for row in rows(self.run/'navigation_selected_scans.csv'):
            # A zero-return selection contributes no points. During takeoff a
            # repeated report timestamp can also have a nonempty archive packet.
            if int(row['points'])==0:continue
            with np.load(member(self.run,row['file']),allow_pickle=False) as d:
                pts=d['local_points'].astype(float)@d['sensor_rotation_ned'].T+d['sensor_position_ned']
                require(len(pts)==int(row['points']),'Navigation selected-scan count mismatch');batches.append(pts)
        reproduced=geometry.voxel(np.vstack(batches),.12)
        with np.load(self.run/'navigation_observations.npz',allow_pickle=False) as d:
            same(reproduced,d['points_ned'],'Navigation raw-scan/voxel replay')
            start=json_file(self.run/'initial_poses.json')['Car1']
            yaw=next(float(r['yaw_rad']) for r in self.trajectory_rows if r['name']=='Car1')
            points,prior=planning.clear_ego_footprint(d['points_ned'],start,yaw,self.config['ground_z_ned'],self.config['navigation']['resolution_m'])
            same(prior,d['ego_footprint_prior_ned'],'Declared ego-footprint prior')
        grid=planner_class(points,start,self.config['navigation'],self.config['ground_z_ned'])
        with np.load(self.run/'navigation_grid.npz',allow_pickle=False) as d:
            for field,key in [('ground','ground_counts'),('obstacle','obstacle_counts'),('observed','observed'),('inflated','inflated_obstacles'),('free','free')]:
                require(np.array_equal(getattr(grid,field),d[key]),'Navigation grid replay mismatch: '+key)
        path=grid.route(start,self.config['navigation']['goal_ned']);same(path,json_file(self.run/'planned_path.json'),'A* route replay')
        error=float(np.linalg.norm(self.publication_tracks['Car1'][-1,:2]-np.array(self.config['navigation']['goal_ned'][:2])))
        require(error<1,'Navigation did not reach goal');same(error,self.metrics['goal_error_m'],'Published goal error')
        self.report['checks']['task']={'selected_scans_replayed':len(batches),'observed_voxel_points':len(reproduced),
            'grid_and_route_reproduced':True,'waypoints':len(path),'goal_error_m':error}

    def tracking(self):
        tracker_class=snapshot_module(self.run,'planning').Tracker
        trailing=snapshot_module(self.run,'study_planning').trailing_point
        tracker=tracker_class(max_age=self.config['sensor_max_age_sec'],prediction_horizon=self.config['tracking']['prediction_horizon_sec'])
        last={};trail=[];complementary=0;maximum=0.;controls=rows(self.run/'controls.csv')
        goals={name:[r for r in controls if r['name']==name] for name in ('Drone1','Car1')}
        for i,row in enumerate(self.tracking_rows):
            fresh=[]
            for name,prefix in [('Drone1','uav'),('Car1','ugv')]:
                stamp=float(row[prefix+'_observation_time']) if row[prefix+'_observation_time'] else None
                if stamp is not None and stamp>last.get(name,-1):
                    fresh.append({'time':stamp,'position':np.array([float(row[prefix+'_estimate_'+a]) for a in 'xyz']),
                                  'confidence':min(1.,int(row[prefix+'_pixels'])/500)})
                    last[name]=stamp
            estimate,measured=tracker.update(float(row['time']),fresh)
            require(measured==(row['measurement_update']=='True'),'Fusion measurement/prediction flag mismatch')
            if estimate is not None:
                expected=[float(row['fused_'+a]) for a in 'xyz'];same(estimate,expected,'Complete fusion replay')
                maximum=max(maximum,float(np.max(abs(estimate-expected))))
                if not trail or np.linalg.norm(estimate[:2]-trail[-1][:2])>.25:trail.append(estimate.copy())
                air_goal=estimate.copy();air_goal[2]=self.config['ground_z_ned']-self.config['uav_altitude_agl_m']
                same(air_goal,[float(goals['Drone1'][i]['goal_'+a]) for a in 'xyz'],'UAV goal from sensor fusion')
                car_goal=goals['Car1'][i]
                if car_goal['goal_x']:
                    same(trailing(trail,self.config['tracking']['follow_distance_m']),[float(car_goal['goal_'+a]) for a in 'xyz'],'UGV goal from estimated trail')
            complementary+=row.get('target_in_occlusion_region',row.get('target_origin_under_canopy'))=='True' and row['uav_visible']=='False' and row['ugv_visible']=='True' and row['uav_reason']=='target_not_in_image'
        same(trail,json_file(self.run/'estimated_target_trail.json'),'Estimated trail replay')
        require(complementary==self.manifest['ground_only_tracking_frames']==self.metrics['ground_only_occlusion_samples'],'Occlusion control-sample mismatch')
        require(complementary>=3 and self.manifest['air_reacquired'],'No verified occlusion/reacquisition')
        self.report['checks']['task']={'fusion_steps_replayed':len(self.tracking_rows),'sensor_derived_follower_goals_reproduced':True,
            'maximum_fusion_difference_m':maximum,'ground_only_occlusion_control_samples':complementary,
            'reacquired':True,'note':'Raw-camera replay covers the selected exposures; complete fusion replay uses all recorded observation vectors.'}

    def formation(self):
        routes=json_file(self.run/'reference_routes.json');end_errors={}
        for name,points in self.tracks.items():
            dimensions=3 if name.startswith('Drone') else 2
            error=float(np.linalg.norm(points[-1,:dimensions]-np.array(routes[name][-1][:dimensions])))
            require(error<self.config['waypoint_tolerance_m'],'Final route waypoint not reached: '+name)
            require(self.manifest['route_indices'][name]==len(routes[name]),'Route incomplete: '+name)
            require(self.manifest['laps'][name]>=1,'No complete lap: '+name);end_errors[name]=error
        # Initial pre-takeoff logging precedes the controller safety loop.
        positions=np.stack(list(self.tracks.values()),axis=1)[1:]
        actor_count=positions.shape[1]
        require(actor_count>=2,'Formation requires at least two controlled platforms')
        minimum=min(float(np.linalg.norm(positions[:,i]-positions[:,j],axis=1).min())
                    for i in range(actor_count) for j in range(i))
        same(minimum,self.manifest['minimum_separation_m'],'Formation control-step separation')
        require(minimum>=self.config['minimum_separation_m'],'Formation minimum separation violated')
        self.report['checks']['task']={'completed_laps':self.manifest['laps'],'final_waypoint_errors_m':end_errors,
            'minimum_same_control_step_separation_m':minimum}

    def figures_and_performance(self):
        sources=json_file(self.run/'study_results/figure_sources.json');references=[]
        def walk(value):
            if isinstance(value,dict):
                if 'camera' in value and 'time_ns' in value:
                    key=(value['camera'],str(value['time_ns']));require(key in self.frames,'Figure frame not indexed')
                    row=self.frames[key]
                    for field in ('file','image'):require(row[field]==value[field],'Figure source path mismatch')
                    references.append(value)
                for v in value.values():walk(v)
            elif isinstance(value,list):
                for v in value:walk(v)
        walk(sources)
        scene=sources['scene_overview'];original=member(self.run,scene['image'])
        require(original.read_bytes()==(self.run/'study_results/scene_overview.png').read_bytes(),'Raw overview was altered')
        boxes=json.loads(self.frames[(scene['camera'],str(scene['time_ns']))]['visible_robots'])
        require(all(boxes.get(name,{}).get('pixels',0)>=10 for name in self.tracks),'Scene does not show all platforms')
        if self.task=='tracking':
            frames=sources['occlusion_sequence']['panels'];require(len(frames)==6,'Occlusion panel missing')
            for i in range(3):
                air=self.frames[(frames[i]['camera'],str(frames[i]['time_ns']))]
                car=self.frames[(frames[i+3]['camera'],str(frames[i+3]['time_ns']))]
                require(abs(int(air['time_ns'])-int(car['time_ns']))<=150_000_000,'Occlusion panels not synchronized')
                air_visible=json.loads(air['visible_robots']).get('Target',{}).get('pixels',0)>=15
                car_visible=json.loads(car['visible_robots']).get('Target',{}).get('pixels',0)>=15
                require(air_visible==(i!=1) and car_visible,'Occlusion panel visibility does not match labels')
        performance=rows(self.run/'performance.csv');rates=rows(self.run/'study_results/sensor_rates.csv');counts=rows(self.run/'sensor_counts.csv')
        for field in ('fps','memory_mb','cpu_total_percent','real_time_factor'):
            values=[float(r[field]) for r in performance if r.get(field)]
            if values:same(np.mean(values),self.metrics['performance'][field]['mean'],'Published performance mean '+field)
        for r in rates:
            stream=[s for s in counts if s['stream']==r['stream']];delta=int(stream[-1]['messages_total'])-int(stream[0]['messages_total'])
            require(delta==int(r['messages_delta']),'Topic-rate count mismatch')
            for field,timefield in [('wall_hz','wall_elapsed_sec'),('sim_hz','time')]:
                dt=float(stream[-1][timefield])-float(stream[0][timefield])
                if dt>0:same(delta/dt,float(r[field]),'Topic receipt rate '+field)
        self.report['checks']['figures_and_performance']={'camera_source_references':len(references),
            'raw_scene_pixel_file_unchanged':True,'all_controlled_platforms_visible':True,
            'performance_samples':len(performance),'topic_stream_rates_reproduced':len(rates)}

    def execute(self):
        require(self.manifest['status']=='completed','Run is not completed')
        require(not self.manifest['archive_errors'] and not self.manifest['cleanup_errors'],'Run reports archive/cleanup errors')
        for stage in (self.provenance,self.lidar,self.cameras,self.poses,self.trajectories,getattr(self,self.task),self.figures_and_performance):
            print(f'{self.task}: checking {stage.__name__}',flush=True);stage()
        self.report['status']='passed';return self.report


def audit(run,output=None):
    checker=StudyAudit(run);destination=Path(output) if output else checker.run/'study_results/audit.json'
    try:report=checker.execute()
    except Exception as exc:
        checker.report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        write_json(destination,checker.report);raise
    write_json(destination,report);return report
