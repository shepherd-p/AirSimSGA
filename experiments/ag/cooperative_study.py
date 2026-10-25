"""Measured navigation, occlusion tracking, and configurable fleet experiments.

Existing mapping code/configs are preserved. Observer poses affect cameras only.
"""
import asyncio
import math
import time
import numpy as np
from .backend import Backend
from .experiments import Experiment
from .math3d import pose_parts,voxel
from .planning import clear_ego_footprint,rover_command
from .scan_capture import ScanRecorder
from .study_recording import TopicRecorder,CameraRecorder
from .study_planning import ObservedGroundPlanner,trailing_point,path_lookahead
from .storage import ROOT,write_json


class StudyBackend(Backend):
    def load(self):
        from projectairsim import World,Drone,Rover
        self.world=World(self.client,self.config['scene'],sim_config_path=self.config.get('sim_config_path',str(ROOT/'configs')),delay_after_load_sec=1)
        scene=self.world.get_configuration();self.cameras={}
        leaves={0:'scene_camera',2:'depth_camera',3:'segmentation_camera'}
        for actor in scene['actors']:
            name=actor['name'];root=f'{self.world.parent_topic}/robots/{name}'
            if name!='Observer':
                kind='uav' if name.startswith('Drone') else 'ugv';self.kinds[name]=kind
                self.robots[name]=(Drone if kind=='uav' else Rover)(self.client,self.world,name)
                for leaf,key in [('actual_pose','pose'),('collision_info','collision')]:self.subscribe(name+'/'+key,root+'/'+leaf)
            for sensor in actor['robot-config']['sensors']:
                if not sensor['enabled']:continue
                if sensor['type']=='lidar':self.subscribe(name+'/lidar',root+'/sensors/'+sensor['id']+'/lidar')
                elif sensor['type']=='camera':
                    key=name+'/'+sensor['id'] if name=='Observer' else name
                    self.cameras[key]=[]
                    for setting in sensor['capture-settings']:
                        leaf=leaves[setting['image-type']]
                        self.subscribe(key+'/'+leaf,root+'/sensors/'+sensor['id']+'/'+leaf);self.cameras[key].append(leaf)
        if set(self.world.list_actors())!={a['name'] for a in scene['actors']}:raise RuntimeError('Loaded actor inventory mismatch')
        self.world.resume()


class CooperativeStudy(Experiment):
    def __init__(self,config,output=None):
        super().__init__(config,output)
        self.b=StudyBackend(config);self.phase='initialization';self.media=None
        self.navigation_batches=[];self.navigation_stamp=-1;self.navigation_path=None
        self.scan_done_since=None;self.target_done_since=None;self.estimated_trail=[]
        self.laps={};self.route_completion_marked=set();self.ground_only_frames=0;self.reacquired=False;self.had_air_occlusion=False
        self.last_performance=-1e9;self.last_progress=-1e9;self.recorders=[]

    async def execute(self):
        status='failed';error=None;archive_errors=[]
        self.run.json('study_protocol.json',{'task':self.c['task'],'config':self.c['study'],
            'success':'Navigation: observed-map plan and actual goal reached; tracking: target route completed, measured occlusion and reacquisition; formation: all configured paths completed',
            'limits':'Existing planar rover physics; simulator ego poses; segmentation/depth tracking baseline; no imported scene mesh for planning'})
        try:
            self.b.connect();self.run.json('previous_scene_ids.json',self.b.previous_scene_ids)
            self.b.load();self.run.json('loaded_scene.json',self.b.world.get_configuration())
            await self.b.ready(35);await self.b.settle()
            self.run.json('startup_pose_sync.json', await self.b.wait_for_pose_sync())
            self.initial={n:self.b.pose(n)['position'] for n in self.b.kinds}
            self.run.json('initial_poses.json',self.initial)
            for name,sid in self.c['study']['segmentation_ids'].items():
                if not self.b.world.set_segmentation_id_by_name(name,sid,False,True):raise RuntimeError('Segmentation assignment failed: '+name)
            if self.c['task']=='tracking' and self.c['tracking'].get('canopy',{}).get('enabled',False):
                canopy=self.c['tracking']['canopy'];name=self.b.create_canopy(canopy)
                self.run.json('runtime_canopy.json',{'name':name,**canopy})
            scans=ScanRecorder(self.run.path);topics=TopicRecorder(self.run.path)
            self.media=CameraRecorder(self.b,self.run.path);self.recorders=[scans,topics,self.media]
            def receive(key,message):scans.receive(key,message);topics.receive(key,message)
            self.b.buffer.sink=receive
            self.configure_routes();self.start_sim=self.b.now();self.start_wall=time.monotonic();self._find_process()
            self.record({n:self.b.pose(n) for n in self.b.kinds});await self.b.start()
            self.run.json('post_takeoff_pose_sync.json', await self.b.wait_for_pose_sync())
            print(f'Running {self.c["task"]} study: {self.run.path}',flush=True)
            status=await self.study_loop()
        except asyncio.CancelledError:
            status,error='interrupted','Cancelled by user';raise
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}';print(error,flush=True)
        finally:
            self.b.buffer.sink=None
            cleanup=await self.b.cleanup()
            for recorder in self.recorders:
                try:recorder.close()
                except Exception as exc:archive_errors.append(str(exc))
            if archive_errors and status=='completed':status='incomplete_archive'
            self.run.finish(status,error=error,cleanup_errors=cleanup,archive_errors=archive_errors,
                sensor_message_counts=dict(self.b.buffer.counts),collision_count=self.collision_count,
                minimum_separation_m=self.min_separation,path_lengths_m=self.path_lengths,
                horizontal_path_lengths_m=self.horizontal_lengths,route_indices=self.indices,laps=self.laps,
                ground_only_tracking_frames=self.ground_only_frames,air_reacquired=self.reacquired,
                phase=self.phase,wall_duration_sec=time.monotonic()-getattr(self,'start_wall',time.monotonic()))
        return self.run.path,status

    def record(self,poses):
        for name,p in poses.items():
            pos=p['position'];previous=self.previous_positions.get(name)
            self.path_lengths[name]=self.path_lengths.get(name,0.)+(float(np.linalg.norm(pos-previous)) if previous is not None else 0.)
            self.horizontal_lengths[name]=self.horizontal_lengths.get(name,0.)+(float(np.linalg.norm(pos[:2]-previous[:2])) if previous is not None else 0.)
            self.previous_positions[name]=pos.copy()
            self.run.row('trajectories',{'time':p['time'],'name':name,'x':pos[0],'y':pos[1],'z':pos[2],
                'yaw_rad':p['yaw'],'speed_mps':np.linalg.norm(p['velocity']),'latency_ms':p['latency_ms'],'phase':self.phase})
            if self.b.buffer.latest(name+'/lidar',self.c['sensor_max_age_sec']) is not None:self.last_sensor_time[name]=time.monotonic()
        if self.media:self.media.collect();self.media.check()

    def phase_change(self,phase,now):
        if phase!=self.phase:
            self.phase=phase;self.run.row('phases',{'time':now,'phase':phase});print(f'{phase} at {now-self.start_sim:.1f} s',flush=True)

    def navigation_goals(self,now,poses):
        self.phase_change('aerial_scan' if self.navigation_path is None else 'ground_navigation',now)
        message=self.b.buffer.latest('Drone1/lidar',1.5)
        if self.navigation_path is None and message is not None and int(message['time_stamp'])!=self.navigation_stamp:
            self.navigation_stamp=int(message['time_stamp'])
            local=np.asarray(message['point_cloud'],dtype=float).reshape(-1,3)
            keep=np.isfinite(local).all(axis=1)&(np.linalg.norm(local,axis=1)>.05)
            seg=np.asarray(message.get('segmentation_cloud',[]))
            if len(seg)==len(local):keep &= ~np.isin(seg,list(self.c['study']['segmentation_ids'].values()))
            p,r=pose_parts(message['pose']);points=local[keep]@r.T+p
            self.navigation_batches.append(points)
            self.run.row('navigation_selected_scans',{'time_ns':self.navigation_stamp,'points':len(points),
                'file':f'raw/Drone1/{self.navigation_stamp}.npz'})
        if self.navigation_path is None:
            goal=self.route_goal('Drone1',poses['Drone1']['position'])
            if goal is not None:return {'Drone1':goal,'Car1':None},False
            if self.scan_done_since is None:self.scan_done_since=now
            if now-self.scan_done_since<2:return {'Drone1':None,'Car1':None},False
            observations=voxel(np.vstack(self.navigation_batches),.12)
            start=poses['Car1']['position'];floor=self.c['ground_z_ned']
            points,prior=clear_ego_footprint(observations,start,poses['Car1']['yaw'],floor,self.c['navigation']['resolution_m'])
            np.savez_compressed(self.run.path/'navigation_observations.npz',points_ned=observations,ego_footprint_prior_ned=prior)
            planner=ObservedGroundPlanner(points,start,self.c['navigation'],floor)
            np.savez_compressed(self.run.path/'navigation_grid.npz',ground_counts=planner.ground,obstacle_counts=planner.obstacle,
                 observed=planner.observed,inflated_obstacles=planner.inflated,free=planner.free,clearance_m=planner.clearance,
                 low_xy=planner.low,resolution_m=planner.step,ground_z_ned=floor)
            self.navigation_path=planner.route(start,self.c['navigation']['goal_ned'])
            self.routes['Car1']=self.navigation_path;self.run.json('planned_path.json',self.navigation_path)
            self.run.json('planning_provenance.json',{'map_source':'UAV LiDAR from this run only',
                'world_prior':'Known settled task-region height because existing rover physics is planar',
                'local_prior':'Settled ego footprint only; unknown outside observed cells remains blocked',
                'planner':'Observed-ground A*, obstacle inflation and line-of-sight shortening',
                'original_scene_mesh_used':False})
            print(f'Planned {len(self.navigation_path)} waypoints from {len(observations)} UAV points',flush=True)
        pos=poses['Car1']['position'];goal,index=path_lookahead(self.navigation_path,pos)
        self.indices['Car1']=index
        distance=float(np.linalg.norm(pos[:2]-np.array(self.c['navigation']['goal_ned'])[:2]))
        reached=distance<1.0
        if self.front_obstacle(poses['Car1']):
            goal=None;self.run.row('navigation_events',{'time':now,'event':'local_obstacle_stop'})
        return {'Drone1':None,'Car1':None if reached else goal},reached

    def tracking_goals(self,now,poses):
        opt=self.c['tracking'];self.phase_change('tracking',now)
        observations={name:self.b.observation(name,opt['target_id']) for name in ('Drone1','Car1')}
        fresh=[]
        for name,obs in observations.items():
            if obs is not None and obs['time']>self.last_observations.get(name,-1):
                fresh.append(obs);self.last_observations[name]=obs['time']
        now=self.b.now();estimate,measured=self.tracker.update(now,fresh)
        if estimate is not None:
            if not self.estimated_trail or np.linalg.norm(estimate[:2]-self.estimated_trail[-1][:2])>.25:
                self.estimated_trail.append(estimate.copy())
        goals={'Drone1':None,'Car1':None}
        if estimate is not None:
            drone_goal=estimate.copy();drone_goal[2]=self.c['ground_z_ned']-self.c['uav_altitude_agl_m'];goals['Drone1']=drone_goal
            trail_goal=trailing_point(self.estimated_trail,opt['follow_distance_m'])
            if np.linalg.norm(estimate[:2]-poses['Car1']['position'][:2])>opt['follow_distance_m']-.3:
                goals['Car1']=trail_goal
        # Truth is used below only for evaluation and Target's own route controller.
        truth=poses['Target']['position'];goals['Target']=self.route_goal('Target',truth)
        row={'time':now,'truth_time':poses['Target']['time'],'gt_x':truth[0],'gt_y':truth[1],'gt_z':truth[2],
             'fused_x':estimate[0] if estimate is not None else None,'fused_y':estimate[1] if estimate is not None else None,
             'fused_z':estimate[2] if estimate is not None else None,'measurement_update':measured,
             'uav_visible':observations['Drone1'] is not None,'ugv_visible':observations['Car1'] is not None}
        # Evaluation-only region; never used by perception or route following.
        region=opt.get('occlusion_region',opt.get('canopy'))
        if region is None:raise ValueError('Tracking requires an evaluation occlusion region')
        under=bool(np.all(abs(truth[:2]-np.array(region['center_ned'][:2]))<=np.array(region['scale'][:2])/2))
        row['target_in_occlusion_region']=under
        for name,prefix in [('Drone1','uav'),('Car1','ugv')]:
            obs=observations[name];diag=self.b.observation_diagnostics[name]
            row.update({prefix+'_observation_time':obs['time'] if obs else None,prefix+'_reason':diag['reason'],
                        prefix+'_pixels':diag['pixels']})
            for i,axis in enumerate('xyz'):row[prefix+'_estimate_'+axis]=obs['position'][i] if obs else None
        if under and not row['uav_visible'] and row['ugv_visible'] and row['uav_reason']=='target_not_in_image':
            self.ground_only_frames+=1;self.had_air_occlusion=True
        if self.had_air_occlusion and not under and row['uav_visible']:self.reacquired=True
        self.run.row('tracking',row)
        if goals['Target'] is None:
            if self.target_done_since is None:self.target_done_since=now
        else:self.target_done_since=None
        return goals,bool(self.target_done_since is not None and now-self.target_done_since>5)

    def formation_goals(self,now,poses):
        self.phase_change('coordinated_routes',now)
        goals={name:self.route_goal(name,p['position']) for name,p in poses.items()}
        for name,goal in goals.items():
            if goal is None and name not in self.route_completion_marked:
                self.laps[name]=self.laps.get(name,0)+1
                self.route_completion_marked.add(name)
        ground_finished=all(goals[n] is None for n in poses if self.b.kinds[n]=='ugv')
        # A UAV already on a repeated lap must finish it before the run ends.
        completed=all(goal is None for goal in goals.values())
        if not ground_finished:
            for name in poses:
                if self.b.kinds[name]=='uav' and goals[name] is None:
                    self.indices[name]=1;goals[name]=self.routes[name][1]
                    self.route_completion_marked.discard(name)
        return goals,completed

    async def study_loop(self):
        deadline=time.monotonic()+self.c['duration_sec']*6+60
        last_sim=-1.;last_progress=time.monotonic();completed=False
        while time.monotonic()<deadline:
            tick=time.monotonic();now=self.b.now()
            if now>last_sim:last_sim=now;last_progress=tick
            if tick-last_progress>10:raise TimeoutError('Simulation clock stalled')
            if now-self.start_sim>self.c['duration_sec']:break
            poses={n:self.b.pose(n) for n in self.b.kinds};self.record(poses);self.safety(poses)
            task=self.c['task']
            if task=='navigation':goals,completed=self.navigation_goals(now,poses)
            elif task=='tracking':goals,completed=self.tracking_goals(now,poses)
            else:goals,completed=self.formation_goals(now,poses)
            for name,goal in goals.items():
                self.run.row('controls',{'time':now,'name':name,'goal_x':goal[0] if goal is not None else None,
                    'goal_y':goal[1] if goal is not None else None,'goal_z':goal[2] if goal is not None else None,
                    'route_index':self.indices.get(name,0),'phase':self.phase})
            await asyncio.gather(*(self.go(name,poses[name],goal,self.c['tracking'].get('target_speed_mps',.75) if name=='Target' else None) for name,goal in goals.items()))
            if now-self.last_performance>1:self.performance(now);self.last_performance=now
            if now-self.last_progress>10:
                print(f'{task} t={now-self.start_sim:.1f}s routes={self.indices} distances={self.horizontal_lengths}',flush=True);self.last_progress=now
            fixed_window=self.c['study'].get('fixed_measurement_sec')
            if fixed_window is not None:
                if now-self.start_sim>=fixed_window:break
            elif completed:break
            await asyncio.sleep(max(0,1/self.c['control_hz']-(time.monotonic()-tick)))
        if not completed:return 'incomplete_route_or_goal'
        if self.c['task']=='tracking':
            self.run.json('estimated_target_trail.json',self.estimated_trail)
            if self.horizontal_lengths.get('Target',0)<self.c['study']['minimum_motion_m']:return 'incomplete_target_motion'
            if self.ground_only_frames<self.c['study']['minimum_ground_only_frames'] or not self.reacquired:return 'incomplete_occlusion_validation'
        if self.c['task']=='navigation' and self.horizontal_lengths.get('Car1',0)<self.c['study']['minimum_motion_m']:return 'incomplete_motion'
        return 'completed'
