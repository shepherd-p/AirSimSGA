import asyncio
import math
import time
from pathlib import Path
import numpy as np
from .backend import Backend
from .math3d import voxel, icp, wrap
from .planning import SurfaceMap, Tracker, rover_command, clear_ego_footprint
from .storage import Run, ROOT, save_pcd
from .contact import is_ground_contact


class Experiment:
    def __init__(self, config, output=None):
        self.c, self.run = config, Run(config, output)
        self.b = Backend(config)
        self.clouds = {}
        self.last_saved, self.last_images, self.collision_seen = {}, {}, {}
        self.indices = {}
        self.initial, self.last_sensor_time = {}, {}
        self.min_separation = float('inf')
        self.collision_count = 0
        self.path_lengths, self.previous_positions = {}, {}
        self.horizontal_lengths = {}
        self.process = None
        self.fps_supported = True
        self.video_writers = {}
        self.navigation_path = None
        self.tracker = Tracker(max_age=config['sensor_max_age_sec'], prediction_horizon=config['tracking']['prediction_horizon_sec'])
        self.last_observations = {}
        self.last_observation_packets = {}
        self.tracking_visible, self.tracking_frames, self.tracking_squared = 0, 0, []
        self.dense_recorder = None
        if config.get('dense_capture'):
            from .dense_capture import DenseRecorder
            self.dense_recorder = DenseRecorder(self.b, self.run, config['dense_capture'])

    async def execute(self):
        status, error = 'failed', None
        try:
            self.b.connect()
            self.run.json('previous_scene_ids.json', self.b.previous_scene_ids)
            self.b.load()
            self.run.json('loaded_scene.json', self.b.world.get_configuration())
            await self.b.ready()
            await self.b.settle()
            if self.dense_recorder:
                for name,sid in self.c['dense_capture']['robot_segmentation_ids'].items():
                    if not self.b.world.set_segmentation_id_by_name(name,sid,False,True):
                        raise RuntimeError(f'Cannot mask moving robot {name} in dense reconstruction')
            # Runtime coordinate and terrain verification precedes longer missions.
            for name in self.b.kinds:
                self.initial[name] = self.b.pose(name)['position']
            self.run.json('initial_poses.json', self.initial)
            if self.c['task'] == 'tracking':
                ok = self.b.world.set_segmentation_id_by_name('Target', self.c['tracking']['target_id'], False, True)
                if not ok:
                    raise RuntimeError('Target segmentation assignment failed')
                canopy = self.c['tracking'].get('canopy',{})
                if canopy.get('enabled'):
                    name = self.b.create_canopy(canopy)
                    self.run.json('runtime_canopy.json',{'name':name,**canopy})
            await self.b.start()
            self.start_sim = self.b.now()
            self.start_wall = time.monotonic()
            self._find_process()
            self.configure_routes()
            print(f"Running {self.c['task']}: {self.run.path}", flush=True)
            status = await self.loop()
            self.finalize_data()
            if self.c['task']=='mapping' and getattr(self,'mapping_icp_status','unknown') != 'ok':
                status = 'incomplete_registration'
        except asyncio.CancelledError:
            status, error = 'interrupted', 'Cancelled by user'
            raise
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
            print(error, flush=True)
        finally:
            cleanup_errors = await self.b.cleanup()
            for writer in self.video_writers.values():
                writer.release()
            self.run.finish(status, error=error, cleanup_errors=cleanup_errors,
                            sensor_message_counts=dict(self.b.buffer.counts),
                            collision_count=self.collision_count,
                            minimum_separation_m=self.min_separation,
                            path_lengths_m=self.path_lengths,
                            horizontal_path_lengths_m=self.horizontal_lengths,
                            rgbd_frame_counts=self.dense_recorder.counts if self.dense_recorder else {},
                            wall_duration_sec=time.monotonic()-getattr(self, 'start_wall', time.monotonic()))
        return self.run.path, status

    def configure_routes(self):
        self.routes = {n: np.asarray(route, dtype=float) for n,route in self.c['routes'].items()}
        if self.c['task'] == 'formation' and not self.c['formation'].get('explicit_routes',False):
            side = self.c['formation']['square_side_m']
            radius = self.c['formation']['circle_radius_m']
            for name,p in self.initial.items():
                if self.b.kinds[name] == 'uav':
                    z = p[2]-self.c['uav_altitude_agl_m']
                    self.routes[name] = np.array([[p[0],p[1],z], [p[0]+side,p[1],z],
                                                  [p[0]+side,p[1]+side,z], [p[0],p[1]+side,z], [p[0],p[1],z]])
                else:
                    center = np.array(self.c['formation']['circle_center_ned'])
                    angle = math.atan2(p[1]-center[1],p[0]-center[0])
                    self.routes[name] = np.array([center+[radius*math.cos(a),radius*math.sin(a),0]
                                                  for a in np.linspace(angle,angle+2*math.pi,49)])
        if self.c['task'] == 'tracking':
            self.routes['Target'] = np.array(self.c['tracking']['target_route'], dtype=float)
        self.run.json('reference_routes.json', self.routes)

    def route_goal(self, name, p, repeat=False):
        route = self.routes[name]
        index = self.indices.get(name, 0)
        dimensions = 3 if self.b.kinds[name] == 'uav' else 2
        tolerance = .6 if self.c['task'] == 'navigation' and name == 'Car1' else self.c['waypoint_tolerance_m']
        while index < len(route) and np.linalg.norm(p[:dimensions]-route[index,:dimensions]) < tolerance:
            index += 1
        if index >= len(route) and repeat:
            index = 0
        self.indices[name] = index
        return route[index] if index < len(route) else None

    async def go(self, name, pose, goal, desired_speed=None):
        if self.b.kinds[name] == 'uav':
            velocity = np.zeros(3) if goal is None else .7*(np.asarray(goal)-pose['position'])
            norm = np.linalg.norm(velocity)
            speed = self.c['uav_speed_mps']
            if norm > speed:
                velocity *= speed/norm
            self.run.row('actuators', {'time':pose['time'],'name':name,'engine':None,'steering_rad':None,'brake':None,
                                      'vx':velocity[0],'vy':velocity[1],'vz':velocity[2]})
            await self.b.drone_velocity(name, velocity)
        else:
            command = (0.,0.,1.) if goal is None else rover_command(
                pose['position'], pose['yaw'], goal, np.linalg.norm(pose['velocity'][:2]),
                desired_speed=desired_speed or self.c['ugv_speed_mps'])
            self.run.row('actuators', {'time':pose['time'],'name':name,'engine':command[0],'steering_rad':command[1],'brake':command[2],
                                      'vx':None,'vy':None,'vz':None})
            await self.b.rover(name, *command)

    def record(self, poses):
        import cv2
        for name,p in poses.items():
            if self.dense_recorder:
                self.dense_recorder.record(name)
            pos = p['position']
            previous = self.previous_positions.get(name)
            self.path_lengths[name] = self.path_lengths.get(name, 0.) + (float(np.linalg.norm(pos-previous)) if previous is not None else 0.)
            self.horizontal_lengths[name] = self.horizontal_lengths.get(name, 0.) + (float(np.linalg.norm(pos[:2]-previous[:2])) if previous is not None else 0.)
            self.previous_positions[name] = pos.copy()
            self.run.row('trajectories', {'time':p['time'], 'name':name, 'x':pos[0], 'y':pos[1], 'z':pos[2],
                                        'yaw_rad':p['yaw'], 'speed_mps':np.linalg.norm(p['velocity']), 'latency_ms':p['latency_ms']})
            msg, cloud = self.b.cloud(name)
            if self.c.get('full_lidar_capture'):
                if msg is not None and len(cloud):
                    self.last_sensor_time[name] = time.monotonic()
                msg = None  # The independent queue owns raw scan recording.
            if msg is not None and len(cloud) and msg['time_stamp'] != self.last_saved.get(name):
                self.last_saved[name] = msg['time_stamp']
                self.last_sensor_time[name] = time.monotonic()
                folder = self.run.path/'raw'/name
                folder.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(folder/f"{msg['time_stamp']}.npz", local_points=np.asarray(msg['point_cloud']).reshape(-1,3), world_points=cloud)
                self.run.row('scans', {'name':name, 'time_ns':msg['time_stamp'], 'points':len(cloud),
                                      'pose':__import__('json').dumps(msg['pose']),
                                      'file':str((folder/f"{msg['time_stamp']}.npz").relative_to(self.run.path))})
                old = self.clouds.get(name, np.empty((0,3)))
                self.clouds[name] = voxel(np.vstack([old,cloud]), .3)
            if time.monotonic()-self.last_images.get(name,0) > 1.:
                imsg,img = self.b.image(name, 'scene_camera')
                if img is not None:
                    folder = self.run.path/'images'/name
                    folder.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(folder/f"{imsg['time_stamp']}.jpg"), img)
                    self.run.row('images', {'name':name,'time_ns':imsg['time_stamp'],
                                           'file':str((folder/f"{imsg['time_stamp']}.jpg").relative_to(self.run.path))})
                    if name not in self.video_writers:
                        writer = cv2.VideoWriter(str(self.run.path/f'{name}.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 1., (img.shape[1],img.shape[0]))
                        if writer.isOpened():
                            self.video_writers[name] = writer
                    if name in self.video_writers:
                        self.video_writers[name].write(img)
                    self.last_images[name] = time.monotonic()

    def safety(self, poses):
        names = list(poses)
        for i,a in enumerate(names):
            for b in names[i+1:]:
                separation = float(np.linalg.norm(poses[a]['position']-poses[b]['position']))
                self.min_separation = min(self.min_separation, separation)
                if separation < self.c['minimum_separation_m']:
                    raise RuntimeError(f'Minimum separation violated: {a}/{b} {separation:.2f} m')
        for name in names:
            if time.monotonic()-self.last_sensor_time.get(name, self.start_wall) > 4.:
                raise RuntimeError(f'Stale/empty LiDAR: stopping {name}')
            event = self.b.collision(name)
            if event is not None and event['time_stamp'] != self.collision_seen.get(name):
                self.collision_seen[name] = event['time_stamp']
                ground = is_ground_contact(event)
                self.run.row('collisions', {'name':name,'time_ns':event['time_stamp'],
                                          'object':event.get('object_name',''), 'ground_contact':ground,
                                          'penetration':event.get('penetration_depth')})
                if not ground:
                    self.collision_count += 1
                    if self.c['stop_on_collision']:
                        raise RuntimeError(f'Obstacle collision: {name} / {event.get("object_name")}')

    async def loop(self):
        task = self.c['task']
        deadline = self.start_wall + self.c['duration_sec']*3 + 30
        last_sim, progress = self.b.now(), time.monotonic()
        navigation_points = np.empty((0,3))
        last_depth = None
        completed = False
        while time.monotonic() < deadline:
            tick = time.monotonic()
            now = self.b.now()
            elapsed = now-self.start_sim
            if now < last_sim:
                raise RuntimeError('Scene or simulation clock reset during experiment')
            if now > last_sim:
                last_sim, progress = now, tick
            if tick-progress > 8:
                raise RuntimeError('Simulation clock stalled')
            if elapsed >= self.c['duration_sec']:
                break
            poses = {name:self.b.pose(name) for name in self.b.kinds}
            self.record(poses)
            self.safety(poses)
            goals = {}
            if task == 'tracking':
                goals = self.tracking_goals(now, poses)
            elif task == 'navigation':
                opt = self.c['navigation']
                msg, points = self.b.cloud('Drone1') if opt.get('map_source') == 'uav_lidar' else self.b.depth_points('Drone1')
                if msg is not None and msg['time_stamp'] != last_depth:
                    last_depth = msg['time_stamp']
                    navigation_points = voxel(np.vstack([navigation_points,points]), .12)
                if elapsed < opt['scan_sec']:
                    goals['Drone1'] = self.route_goal('Drone1', poses['Drone1']['position'], repeat=True)
                    goals['Car1'] = None
                else:
                    goals['Drone1'] = None
                    if self.navigation_path is None:
                        save_pcd(self.run.path/'navigation_observations.pcd', navigation_points)
                        start = poses['Car1']['position'].copy()
                        start[2] += self.c['rover_surface_offset_m']
                        map_points, prior = clear_ego_footprint(navigation_points, start, poses['Car1']['yaw'], start[2],opt['resolution_m'])
                        save_pcd(self.run.path/'ego_footprint_prior.pcd',prior)
                        self.run.json('planning_provenance.json',{'map':opt.get('map_source','uav_perspective_depth'),
                                      'prior':'settled ego footprint only; current robot returns cleared',
                                      'unknown_outside_footprint':'blocked'})
                        grid = SurfaceMap(map_points, resolution=opt['resolution_m'], height_bin=opt['height_bin_m'],
                                          max_step=opt['max_step_m'], max_slope=opt['max_slope'],
                                          clearance=opt['clearance_m'], radius=opt['vehicle_radius_m'])
                        self.navigation_path = grid.route(start, opt['goal_ned'])
                        self.routes['Car1'] = self.navigation_path
                        self.run.json('planned_path.json', self.navigation_path)
                        self.run.json('surface_nodes.json', list(grid.nodes.values()))
                    goals['Car1'] = self.route_goal('Car1', poses['Car1']['position'])
                    if self.front_obstacle(poses['Car1']):
                        goals['Car1'] = None
                        self.run.row('navigation_events', {'time':now,'event':'local_obstacle_stop'})
                    elif goals['Car1'] is None:
                        completed = True
            else:
                goals = {name:self.route_goal(name,p['position'], repeat=(task == 'mapping' and self.b.kinds[name]=='uav')) for name,p in poses.items()}
                completed = task == 'formation' and all(g is None for g in goals.values())
            for name,goal in goals.items():
                p = poses[name]
                error = np.linalg.norm((np.asarray(goal)-p['position'])[:2]) if goal is not None else None
                self.run.row('controls', {'time':now,'name':name,'goal_x':goal[0] if goal is not None else None,
                                         'goal_y':goal[1] if goal is not None else None,'goal_z':goal[2] if goal is not None else None,
                                         'distance_to_waypoint_m':error,'route_index':self.indices.get(name,0)})
            await asyncio.gather(*(self.go(name,poses[name],goal, .7 if name == 'Target' else None) for name,goal in goals.items()))
            self.performance(now)
            if completed:
                break
            await asyncio.sleep(max(0., 1/self.c['control_hz']-(time.monotonic()-tick)))
        if time.monotonic() >= deadline:
            raise TimeoutError('Wall time watchdog exceeded; experiment incomplete')
        if task == 'navigation':
            return 'completed' if completed else 'incomplete_goal_not_reached'
        if task == 'formation':
            return 'completed' if completed else 'incomplete_trajectories'
        if task == 'tracking' and not self.tracking_visible:
            return 'incomplete_no_target_observations'
        if task == 'mapping' and any(self.horizontal_lengths.get(n,0) < 2 for n in self.b.kinds):
            return 'incomplete_insufficient_motion'
        return 'completed'

    def front_obstacle(self, pose):
        msg, points = self.b.cloud('Car1')
        if msg is None:
            return True
        d = points-pose['position']
        yaw = pose['yaw']
        forward = d[:,0]*math.cos(yaw)+d[:,1]*math.sin(yaw)
        right = -d[:,0]*math.sin(yaw)+d[:,1]*math.cos(yaw)
        floor = self.c['rover_surface_offset_m']
        mask = (forward>0)&(forward<self.c['navigation']['stop_distance_m'])&(np.abs(right)<1.2)&(d[:,2]<floor-.4)&(d[:,2]>floor-2.)
        return int(mask.sum()) >= 3

    def tracking_goals(self, now, poses):
        opt = self.c['tracking']
        observations = {name:self.b.observation(name,opt['target_id']) for name in ('Drone1','Car1')}
        fresh = []
        for name,o in observations.items():
            if o is not None and o['time'] > self.last_observations.get(name,-1):
                fresh.append(o)
                self.last_observations[name] = o['time']
            packet = self.b.observation_packets.get(name)
            if packet is not None and packet['time'] > self.last_observation_packets.get(name,-1):
                self.last_observation_packets[name] = packet['time']
                folder = self.run.path/'observations'/name
                folder.mkdir(parents=True,exist_ok=True)
                stamp = str(round(packet['time']*1e9))
                np.savez_compressed(folder/f'{stamp}.npz',**packet['raw'])
                self.run.json(f'observations/{name}/{stamp}.json',packet['capture'])
        now = self.b.now()  # Messages collected during recording must not look like future observations.
        estimate, measured = self.tracker.update(now,fresh)
        self.tracking_frames += 1
        self.tracking_visible += int(measured)
        # Truth is accessed only here for evaluation, never used to compute goals.
        truth_pose = self.b.pose('Target')
        truth = truth_pose['position']
        error = float(np.linalg.norm(estimate[:2]-truth[:2])) if estimate is not None else None
        if error is not None:
            self.tracking_squared.append(error**2)
        record = {'time':now,'truth_time':truth_pose['time'],'gt_x':truth[0],'gt_y':truth[1],'gt_z':truth[2],
                  'fused_x':estimate[0] if estimate is not None else None,
                  'fused_y':estimate[1] if estimate is not None else None,
                  'fused_z':estimate[2] if estimate is not None else None,
                  'xy_error_m':error,'measurement_update':measured,
                  'uav_visible':observations['Drone1'] is not None,'ugv_visible':observations['Car1'] is not None}
        canopy = opt.get('canopy',{})
        center = np.array(canopy.get('center_ned',[0,0,0]))
        size = np.array(canopy.get('scale',[0,0,0]))
        record['target_origin_under_canopy'] = bool(canopy.get('enabled') and np.all(abs(truth[:2]-center[:2]) <= size[:2]/2))
        for name,prefix in [('Drone1','uav'),('Car1','ugv')]:
            o = observations[name]
            record[prefix+'_error_m'] = float(np.linalg.norm(o['position'][:2]-truth[:2])) if o is not None else None
            record[prefix+'_target_distance_m'] = np.linalg.norm(poses[name]['position']-truth)
            record[prefix+'_observation_time'] = o['time'] if o is not None else None
            record[prefix+'_reason'] = self.b.observation_diagnostics[name]['reason']
            record[prefix+'_pixels'] = self.b.observation_diagnostics[name]['pixels']
        self.run.row('tracking',record)
        goals = {'Target':self.route_goal('Target',truth),'Drone1':None,'Car1':None}
        if estimate is not None:
            # Target surface centroid has a different height from actor origin.
            drone_goal = estimate+np.asarray(opt['uav_offset_ned'])
            drone_goal[2] = self.initial['Drone1'][2]-self.c['uav_altitude_agl_m']
            goals['Drone1'] = drone_goal
            d = estimate[:2]-poses['Car1']['position'][:2]
            distance = np.linalg.norm(d)
            if distance > opt['follow_distance_m']:
                car_goal = estimate.copy()
                car_goal[:2] -= d/max(distance,1e-6)*opt['follow_distance_m']
                goals['Car1'] = car_goal
        return goals

    def _find_process(self):
        import psutil
        pid_file = ROOT/'verification/launched_pid.txt'
        explicit_pid = self.c.get('process_pid')
        if explicit_pid is None and pid_file.exists():
            explicit_pid = int(pid_file.read_text().strip())
        if explicit_pid:
            try:
                p = psutil.Process(explicit_pid)
                if 'unrealeditor' in p.name().lower():
                    self.process = p
                    p.cpu_percent()
            except (psutil.AccessDenied,psutil.NoSuchProcess):
                pass
        matches = []
        for p in psutil.process_iter(['pid','name','cmdline']):
            try:
                command = ' '.join(p.info['cmdline'] or []).lower()
                if 'unrealeditor' in p.info['name'].lower() and self.c['process_hint'].lower() in command:
                    matches.append(p)
            except (psutil.AccessDenied,psutil.NoSuchProcess):
                pass
        if self.process is None and len(matches) == 1:
            self.process = matches[0]
            self.process.cpu_percent()
        self.run.json('performance_source.json', {'pid':self.process.pid if self.process else None,
                                                'cpu_definition':'process total core utilization; may exceed 100%',
                                                'sensor_hz_definition':'received messages / elapsed wall seconds'})

    def performance(self, now):
        fps, frame = None, None
        if self.fps_supported:
            try:
                result = self.b.request('GetPerformanceStats')
                fps, frame = result['fps'], result['frame_time_ms']
            except Exception as exc:
                self.fps_supported = False
                self.run.json('fps_unavailable.json', {'reason':str(exc),'action':'No binary changes; metric left unavailable'})
        wall = time.monotonic()-self.start_wall
        self.run.row('performance', {'time':now,'wall_elapsed_sec':wall, 'fps':fps,'frame_time_ms':frame,
                                    'real_time_factor':(now-self.start_sim)/max(wall,.001),
                                    'memory_mb':self.process.memory_info().rss/1048576 if self.process else None,
                                    'cpu_total_percent':self.process.cpu_percent() if self.process else None})
        for key,count in list(self.b.buffer.counts.items()):
            self.run.row('sensor_counts', {'time':now,'wall_elapsed_sec':wall,'stream':key,'messages_total':count})

    def finalize_data(self):
        for name,points in self.clouds.items():
            save_pcd(self.run.path/f'map_{name}.pcd',points)
        if self.c['task'] == 'mapping':
            a,b = self.clouds.get('Drone1',np.empty((0,3))), self.clouds.get('Car1',np.empty((0,3)))
            result, transform = icp(a,b)
            self.mapping_icp_status = result['status']
            self.run.json('icp.json',result)
            if result['status'] == 'ok':
                aligned = a@transform[:3,:3].T+transform[:3,3]
                save_pcd(self.run.path/'map_fused.pcd',voxel(np.vstack([aligned,b])))
            else:
                save_pcd(self.run.path/'map_fused_unregistered.pcd',voxel(np.vstack([a,b])))
            cells_a = set(map(tuple,np.floor(a[:,:2]).astype(int)))
            cells_b = set(map(tuple,np.floor(b[:,:2]).astype(int)))
            self.run.json('mapping_summary.json', {'uav_observed_cells_1m':len(cells_a),'ugv_observed_cells_1m':len(cells_b),
                                                  'union_observed_cells_1m':len(cells_a|cells_b),'overlap_cells_1m':len(cells_a&cells_b),
                                                  'coverage_definition':'observed XY cells, not absolute map completeness',
                                                  'icp_status':result['status']})
        if self.c['task'] == 'tracking':
            self.run.json('tracking_summary.json', {'frames':self.tracking_frames,'measurement_update_frames':self.tracking_visible,
                                                   'fusion_xy_rmse_m':float(np.sqrt(np.mean(self.tracking_squared))) if self.tracking_squared else None,
                                                   'baseline':'simulator segmentation + perspective depth; surface-centroid vs actor-origin error'})
