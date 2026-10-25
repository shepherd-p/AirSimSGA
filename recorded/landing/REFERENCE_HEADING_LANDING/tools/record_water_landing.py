"""Run existing controllers with separate RGB recording and observer-only configs."""
from __future__ import annotations

import argparse
import asyncio
import csv
import gzip
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = Path(r'F:/EpicGamesEnv/ProjectAirSim_AG_Experiments')
LAB = Path(r'F:/EpicGamesEnv/LandingWaterLab')
sys.path[:0] = [str(ROOT), str(ROOT / 'citysample_scene'), str(ROOT / 'video_tools/vendor'),
               str(ROOT.parent / 'ProjectAirSim/client/python/projectairsim/src')]

from ag.backend import Buffer, Backend
from ag.storage import serial, write_json
from projectairsim.utils import load_scene_config_as_dict, unpack_image
import imageio_ffmpeg


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Encoder:
    def __init__(self, path, width, height, fps=30):
        self.path, self.fps = path, fps
        self.log = path.with_suffix('.encoder.log').open('wb')
        self.process = subprocess.Popen([
            imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'warning', '-y',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{width}x{height}',
            '-r', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '20', '-threads', '2',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)
        ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log)
        self.count = 0

    def write(self, frame):
        self.process.stdin.write(np.ascontiguousarray(frame).tobytes())
        self.count += 1

    def close(self):
        self.process.stdin.close()
        code = self.process.wait(timeout=90)
        self.log.close()
        if code:
            raise RuntimeError(f'Encoder failed {self.path}: {code}')


class Recording:
    def __init__(self, folder, task):
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=False)
        self.q = queue.Queue(512)
        self.errors, self.drops = [], 0
        self.streams = {}
        self.ids = ({'mapping': [('UAV', 240), ('UGV', 241)],
                     'landing': [('UAV', 240), ('USV', 242)],
                     'navigation': [],
                     'tracking': [('Target', 210), ('UGV L', 211), ('UGV R', 212)]}[task])
        self.pending_time = {}
        self.lock = threading.Lock()
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def receive(self, buffer, key, message):
        if not (key.endswith('/scene_camera') or key in {'down', 'overview'}):
            return
        stamp = int(message['time_stamp'])
        with self.lock:
            if stamp <= self.pending_time.get(key, -1):
                return
            self.pending_time[key] = stamp
        try:
            with buffer.lock:
                poses = {k: values[-1][1] for k, values in buffer.data.items()
                         if k.endswith('/pose') and values}
            self.q.put_nowait((buffer, key, message, poses))
        except queue.Full:
            self.drops += 1

    def work(self):
        while True:
            item = self.q.get()
            try:
                if item is None:
                    return
                buffer, key, msg, poses = item
                frame = unpack_image(msg)
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                frame = frame[:, :, :3]
                if key not in self.streams:
                    name = key.replace('/', '_')
                    directory = self.folder / name
                    directory.mkdir()
                    self.streams[key] = s = {
                        'encoder': Encoder(directory / 'original_rgb.mp4', frame.shape[1], frame.shape[0]),
                        'start': int(msg['time_stamp']), 'previous': None, 'previous_record': None,
                        'meta': gzip.open(directory / 'capture_metadata.jsonl.gz', 'wt', encoding='utf-8'),
                        'index_stream': (directory / 'video_frame_index.csv').open('w', newline='', encoding='utf-8'),
                        'source_frames': 0, 'keyframe_time': -1e30, 'directory': directory,
                    }
                    s['index'] = csv.writer(s['index_stream'])
                    s['index'].writerow(['video_frame', 'video_time_sec', 'source_time_ns', 'held_previous_exposure'])
                s = self.streams[key]
                capture = {k: v for k, v in msg.items() if k != 'data'}
                boxes = {}
                segment_key = ('overview_seg' if key == 'overview' else
                               key.replace('scene_camera', 'segmentation_camera'))
                segmentation = buffer.entries(segment_key)
                if key in {'Drone1/scene_camera','Drone2/scene_camera','GateRight/scene_camera','Target/scene_camera'} and self.ids:
                    for _ in range(4):
                        if segmentation and min(abs(int(m['time_stamp'])-int(msg['time_stamp'])) for _,m in segmentation) <= 50_000_000:
                            break
                        time.sleep(.01)
                        segmentation = buffer.entries(segment_key)
                paired_segmentation = None
                if segmentation:
                    seg = min((m for _, m in segmentation), key=lambda m: abs(int(m['time_stamp'])-int(msg['time_stamp'])))
                    if abs(int(seg['time_stamp'])-int(msg['time_stamp'])) <= 50_000_000:
                        paired_segmentation = seg
                        from projectairsim.image_utils import segmentation_id_to_color
                        img = unpack_image(seg)
                        for label, identifier in self.ids:
                            col = segmentation_id_to_color(identifier)
                            yy, xx = np.nonzero(np.all(img[:, :, :3] == [col.b, col.g, col.r], axis=2))
                            if len(xx) >= 5:
                                boxes[label] = [int(xx.min()), int(yy.min()), int(xx.max()), int(yy.max())]
                metadata = {'source_time_ns': int(msg['time_stamp']), 'capture': capture,
                            'payload_sha256': hashlib.sha256(bytes(msg['data'])).hexdigest(),
                            'poses': poses, 'segmentation_boxes': boxes,
                            'segmentation_time_ns': int(paired_segmentation['time_stamp']) if paired_segmentation else None}
                s['meta'].write(json.dumps(serial(metadata), separators=(',', ':')) + '\n')
                target_frame = round((int(msg['time_stamp']) - s['start']) / 1e9 * s['encoder'].fps)
                if target_frame > s['encoder'].count + 600:
                    raise RuntimeError(f'Camera timestamp jump: {key}')
                while s['encoder'].count < target_frame and s['previous'] is not None:
                    i = s['encoder'].count
                    s['encoder'].write(s['previous'])
                    s['index'].writerow([i, i / s['encoder'].fps, s['previous_record']['source_time_ns'], 1])
                if s['encoder'].count <= target_frame:
                    i = s['encoder'].count
                    s['encoder'].write(frame)
                    s['index'].writerow([i, i / s['encoder'].fps, int(msg['time_stamp']), 0])
                s['previous'], s['previous_record'] = frame, metadata
                s['source_frames'] += 1
                if int(msg['time_stamp'])/1e9 - s['keyframe_time'] >= 10:
                    cv2.imwrite(str(s['directory'] / f"original_{int(msg['time_stamp'])}.png"), frame)
                    if paired_segmentation is not None:
                        np.savez_compressed(s['directory']/f"segmentation_{int(msg['time_stamp'])}.npz",
                                            payload=np.frombuffer(bytes(paired_segmentation['data']), dtype=np.uint8),
                                            metadata_json=json.dumps(serial({k:v for k,v in paired_segmentation.items() if k != 'data'})))
                    s['keyframe_time'] = int(msg['time_stamp'])/1e9
                if s['source_frames'] % 50 == 0:
                    print(f"VIDEO {key}: {s['source_frames']} exposures, {s['encoder'].count/s['encoder'].fps:.1f}s", flush=True)
            except Exception as exc:
                self.errors.append(f'{type(exc).__name__}: {exc}')
                print('VIDEO ERROR ' + str(exc), flush=True)
            finally:
                self.q.task_done()

    def close(self):
        self.q.put(None, timeout=60)
        self.worker.join(180)
        if self.worker.is_alive():
            raise RuntimeError('Recording writer did not finish')
        report = {'queue_drops': self.drops, 'errors': self.errors, 'cameras': {},
                  'source': 'Live simulator RGB topics during existing experiment control; no synthetic camera motion of robots',
                  'timing': '30 fps simulation-time playback; missing exposures held, no interpolated robot motion',
                  'encoding': 'H.264 RGB footage; periodic lossless PNG originals and exposure metadata retained'}
        for key, s in self.streams.items():
            s['encoder'].close()
            s['meta'].close()
            s['index_stream'].close()
            report['cameras'][key] = {'file': str(s['encoder'].path.relative_to(self.folder)),
                                     'source_exposures': s['source_frames'], 'frames': s['encoder'].count,
                                     'start_time_ns': s['start'], 'duration_sec': s['encoder'].count/s['encoder'].fps,
                                     'sha256': sha(s['encoder'].path)}
        write_json(self.folder / 'recording_manifest.json', report)
        if self.errors or self.drops:
            raise RuntimeError(f'Incomplete recording: errors={self.errors[:3]}, drops={self.drops}')


def record_callback(recording):
    original = Buffer.callback

    def callback(buffer, key):
        receive = original(buffer, key)

        def wrapped(topic, msg):
            receive(topic, msg)
            if isinstance(msg, dict):
                recording.receive(buffer, key, msg)
        return wrapped
    Buffer.callback = callback


def scene_copy(task, output):
    if task == 'landing':
        base = LAB / 'configs/landing'
        path = 'scene_landing.jsonc'
    elif task == 'mapping':
        base = ROOT / 'configs'
        path = 'scene_mapping_plaza.jsonc'
    else:
        base = ROOT / 'citysample_scene'
        path = ('generated/scene_navigation_aerial_route.jsonc' if task == 'navigation'
                else 'generated/scene_pursuit.jsonc')
    source_scene = base / path
    data, _ = load_scene_config_as_dict(path, str(base))
    if task == 'landing':
        data['clock']['real-time-update-rate'] = 40_000_000
        data['id'] = 'AirSimSGAReferenceHeadingLandingRun'
    if task == 'tracking':
        # Keep the physics step while allowing cameras and feedback more wall time.
        data['clock']['real-time-update-rate'] = data['clock']['step-ns'] * 4
        data['segmentation']['initialize-ids'] = False
    if task == 'mapping':
        observer = json.loads((ROOT/'urban_scene/observer_probe.jsonc').read_text())
        data['actors'].append({'type': 'robot', 'name': 'Observer', 'robot-config': observer,
                               'origin': {'xyz': '0 0 0', 'rpy-deg': '0 0 0'}})
    for actor in data['actors']:
        if task == 'tracking' and actor['name'] in {'Target','GateLeft','GateRight'}:
            from prepare_aerial_route_navigation import center_ground_contacts, CONTACT_PROXY_Z_M
            center_ground_contacts(actor['robot-config'], CONTACT_PROXY_Z_M)
        if task == 'tracking' and actor['name'] == 'GateRight':
            for sensor in actor['robot-config'].get('sensors', []):
                if sensor['type'] == 'camera':
                    sensor['origin'] = {'xyz': '-2.5 0 -1.8', 'rpy-deg': '0 0 125'}
                    sensor['capture-interval'] = .5
                    segment = json.loads(json.dumps(sensor['capture-settings'][0]))
                    segment['image-type'] = 3
                    sensor['capture-settings'].append(segment)
        if task == 'tracking' and actor['name'] == 'GateLeft':
            for sensor in actor['robot-config'].get('sensors', []):
                if sensor['type'] == 'camera':
                    sensor['capture-interval'] = .6
        if actor['name'] == 'Observer':
            for sensor in actor['robot-config']['sensors']:
                if sensor['type'] == 'camera':
                    sensor['capture-interval'] = .4 if task == 'tracking' else .2
                    for settings in sensor['capture-settings']:
                        # RGB recording does not need higher-rate segmentation.
                        settings['width'], settings['height'] = 1280, 720
                        if task == 'landing':
                            settings['fov-degrees'] = 65
            if task in {'navigation', 'tracking'}:
                follow = json.loads(json.dumps(actor['robot-config']['sensors'][0]))
                follow['id'] = 'VideoFollow'
                follow['capture-settings'] = [follow['capture-settings'][0]]
                follow['capture-settings'][0]['width'] = 960
                follow['capture-settings'][0]['height'] = 540
                follow['capture-settings'][0]['fov-degrees'] = 75
                actor['robot-config']['sensors'].append(follow)
    if task == 'landing':
        for actor in data['actors']:
            for sensor in actor['robot-config'].get('sensors', []):
                if sensor.get('type') == 'camera':
                    sensor['capture-interval'] = .025
                    if actor['name'] == 'Drone1':
                        # Original RGB resolution/FOV unchanged; unused depth output is disabled.
                        sensor['capture-settings'] = [item for item in sensor['capture-settings'] if item['image-type'] == 0]
    for actor in data['actors']:
        if actor['name'] == 'Vessel1':
            actor['origin']['rpy-deg'] = '0 0 -90'
            # UE replaces root-link world rotation on every motion update.
            # Keep dynamics on a geometry-free root; rotate a fixed visual child.
            links=actor['robot-config']['links']
            hull=next(link for link in links if link['name']=='hull')
            visual=json.loads(json.dumps(hull['visual']))
            visual['origin']={'xyz':'0 0 0','rpy-deg':'0 0 180'}
            hull.pop('visual')  # Optional visual: empty root has no static mesh.
            links.append({'name':'hull_visual','visual':visual,
                          'inertial':{'mass':0.01,'inertia':{'type':'point-mass'}},
                          'collision':{'enabled':False}})
            actor['robot-config']['joints'].append({'id':'hull_visual_fixed','type':'fixed',
                        'parent-link':'hull','child-link':'hull_visual','axis':'0 0 1'})
            for link in links:
                if link['name'] in {'landing_tag','landing_support'}:
                    origin=link['visual']['origin']
                    xyz=[float(v) for v in origin['xyz'].split()]
                    origin['xyz']=' '.join(map(str,[-xyz[0],-xyz[1],xyz[2]]))
                    origin['rpy-deg']='0 0 0'

            env = actor['robot-config'].get('environment', {})
            env.update(wave_interaction=False, current_interaction=False, wind_interaction=False)
        robot_path = output / ('robot_'+actor['name']+'_video.jsonc')
        write_json(robot_path, actor['robot-config'])
        actor['robot-config'] = str(robot_path)
    path = output / f'scene_{task}_video.jsonc'
    write_json(path, data)
    load_scene_config_as_dict(str(path), str(output))
    write_json(output / 'recording_config_changes.json', {
        'source_scene': str(source_scene), 'original_scene_reference': str(base),
        'changes': ('Landing-only: fixed world overview, requested capture interval 0.025s, 1/8 wall-clock scheduling, unchanged 5ms physical step and control RGB resolution, unused depth disabled, service reply timeout 180s; reference hull attitude preserved with body yaw-90 and fixed hull_visual child yaw180; marker/support mirrored to +3m; original lossless control payloads retained, PNG previews sparsely saved' if task == 'landing' else 'Observer RGB interval0.2s (tracking0.4s), 1280x720; landing overview FOV65; mapping adds non-physics observer, UGV speed0.7m/s, duration420s; city experiments add independent follow camera; tracking right UGV has a rear RGB/segmentation evidence camera at0.5s and left evidence camera at0.6s, neither consumed by its controller; tracking uses quarter-speed wall-time clock with unchanged5ms physics step and resets background stencil IDs to0 before original controller assigns exclusive vehicle IDs210to212'),
        'robot_controller_and_physics': ('Original route/controller; recording-only rover contact-isolation links reuse the existing navigation configuration, not validated full wheel-ground physics'
                                        if task == 'tracking' else 'Resolved scene records actual configuration; reference hull attitude uses body yaw-90 and a fixed hull_visual child yaw180; marker/support transforms are recorded'),
        'tracking_contact_helper': (str(ROOT/'citysample_scene/prepare_aerial_route_navigation.py') if task == 'tracking' else None),
        'tracking_contact_helper_sha256': (sha(ROOT/'citysample_scene/prepare_aerial_route_navigation.py') if task == 'tracking' else None),
        'recorder_source_sha256': sha(__file__), 'scene_sha256': sha(path)})
    return path


async def run(args, scene, folder):
    if args.task == 'landing':
        import importlib.util
        spec=importlib.util.spec_from_file_location('ag.landing_runtime_dynamics',HERE/'landing_runtime_dynamics.py')
        landing_runtime=importlib.util.module_from_spec(spec)
        sys.modules[spec.name]=landing_runtime;spec.loader.exec_module(landing_runtime)
        from ag.landing_evidence import Evidence
        class ExtendedEvidence(Evidence):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                folder=self.path/'source_snapshot/recording_extensions';folder.mkdir()
                hashes=json.loads((self.path/'source_sha256.json').read_text(encoding='utf-8'))
                for p in HERE.glob('*.py'):
                    dest=folder/p.name;dest.write_bytes(p.read_bytes());hashes['recording_extensions/'+p.name]=sha(dest)
                write_json(self.path/'source_sha256.json',hashes)
                self._png_keys={}
                self.manifest['storage_policy']='Every controller-consumed original sensor payload retained losslessly in NPZ with exposure metadata; every detection JSON retained; PNG previews once per simulation second and phase change. Decode NPZ to reproduce omitted PNGs.'
                write_json(self.path/'manifest.json',self.manifest)
            def frame(self,camera,message,image,detection=None,phase=''):
                from ag.landing_evidence import annotate
                stamp=int(message['time_stamp']);key=(camera,stamp)
                if key in self.frames:return f'frames/{camera}/{stamp}.npz'
                folder=self.path/'frames'/camera;folder.mkdir(parents=True,exist_ok=True)
                meta={k:v for k,v in message.items() if k!='data'}
                np.savez_compressed(folder/f'{stamp}.npz',payload=np.frombuffer(bytes(message['data']),dtype='u1'),metadata_json=np.array(json.dumps(serial(meta),allow_nan=False)))
                prev=self._png_keys.get(camera)
                keep=prev is None or stamp-prev[0]>=1_000_000_000 or prev[1]!=phase
                if keep:
                    if not cv2.imwrite(str(folder/f'{stamp}.png'),image):raise OSError('Cannot save PNG')
                    if detection is not None:
                        if not cv2.imwrite(str(folder/f'{stamp}_annotated.png'),annotate(image,detection,phase)):raise OSError('Cannot save annotation PNG')
                    self._png_keys[camera]=(stamp,phase)
                if detection is not None:write_json(folder/f'{stamp}.json',detection)
                self.row('frames',{'camera':camera,'time_ns':stamp,'phase':phase,'raw_image':f'frames/{camera}/{stamp}.png' if keep else '', 'payload':f'frames/{camera}/{stamp}.npz','detection':f'frames/{camera}/{stamp}.json' if detection is not None else '', 'visible':int(detection['visible']) if detection is not None else ''})
                self.frames.add(key)
                return f'frames/{camera}/{stamp}.npz'
        landing_runtime.Evidence=ExtendedEvidence
        # Lossless compression of original sensor payloads reduces disk use.
        np.savez = np.savez_compressed
        original = landing_runtime.LandingBackend.load_landing

        def load_landing(b):
            b.config['scene'] = str(scene)
            result = original(b)
            b.client.socket_services.recv_timeout = 180000
            if b.client.request({'method': b.world.parent_topic+'/SetWaterPresentationMode', 'params': {'mode': 32}, 'version': 1.0}) is False:
                raise RuntimeError('Improved water presentation was rejected')
            return result
        landing_runtime.LandingBackend.load_landing = load_landing
        # Fixed world camera removes changing observer motion from the boat view.
        def fixed_observer(b, uav, target):
            if getattr(b, '_fixed_overview_placed', False):
                return
            from urban_scene.probe_urban_scene import camera_pose
            from projectairsim.types import Pose
            pose = camera_pose([0., -12., -22.], [29., 15., 4.], Pose)
            if b.request('SetPose', {'pose': pose, 'wait_for_pose_update': False}, 'Observer/sensors/Overview') is False:
                raise RuntimeError('Fixed observer camera placement was rejected')
            b._fixed_overview_placed = True
        landing_runtime.LandingBackend.observer = fixed_observer
        cfg = json.loads((LAB/'configs/landing/study.json').read_text())
        cfg['wall_timeout_sec'] = 1800.
        cfg['tag_offset_hull_m']=[3.,0.,-2.77]
        cfg['tag_geometry_status']='Boat mesh is rendered as fixed child yaw180, not as the overwritten root visual; marker/support offsets +3m preserve the reference landing platform placement'
        cfg.update(port_topics=9189,port_services=9190,physics_build_receipt=str(LAB/'build_receipt.json'),sim_config_path=str(LAB/'configs/landing'))
        cfg['provenance'] = dict(cfg['provenance'],
            physics_source=str(LAB/'source'),
            bow_alignment='Reference image hull posture is preserved by body yaw-90 and fixed hull_visual child yaw180. Root visual yaw is overwritten by UE on motion and cannot apply this correction. Positive surge moves world -Y, away from shore; visual mesh bow -X maps to body +X, and marker/support remain at body +3m where their reference image placement is preserved. Direction is checked against consecutive fixed-camera RGB.',
            vessel_model='Planar surge/sway/yaw mass, added inertia, Coriolis and linear/quadratic damping, RK4 at 5ms; constant near-level hull anchor; not six-DOF wave/buoyancy validation',
            water_visuals='Simulation-clock small ripples and speed-driven foam; visual waves do not apply physical wave forces',
            vessel_control='Positive body-forward thrust; resistance feedforward from configured drag and speed feedback',
            video_protocol='Live rerun, fixed world observer, 1/8 wall-clock schedule, original 5 ms physical step, 40 Hz requested RGB/observer sampling, no interpolated motion')
        cfg['scene'] = str(scene)
        result = await landing_runtime.run_trial(cfg, folder/'experiment', .5, 1)
        write_json(folder/'experiment_result.json', result)
        return result
    if args.task == 'mapping':
        from ag import experiments
        from ag.plaza import PlazaExperiment
        from citysample_runtime import CitySampleBackend
        from urban_scene.probe_urban_scene import camera_pose
        from projectairsim.types import Pose

        class MappingVideoBackend(CitySampleBackend):
            drone_velocity = Backend.drone_velocity

            def load(self):
                super().load()
                pose = camera_pose([-35, -12, -58], [30, 58, -6], Pose)
                self.client.request({'method': self.world.parent_topic+'/robots/Observer/sensors/Overview/SetPose',
                                     'params': {'pose': pose, 'wait_for_pose_update': True}, 'version': 1.0})
        experiments.Backend = MappingVideoBackend
        cfg = json.loads((ROOT/'configs/mapping_plaza.json').read_text())
        cfg['scene'] = str(scene)
        cfg['duration_sec'] = 420
        cfg['ugv_speed_mps'] = .7
        experiment = PlazaExperiment(cfg, str(folder/'experiment'))
        path, status = await experiment.execute()
        result = {'status': status, 'output': str(path)}
        write_json(folder/'experiment_link.json', {'path': str(path), 'task': 'mapping'})
        write_json(folder/'experiment_result.json', result)
        return result
    module = importlib.import_module('run_aerial_route_navigation' if args.task == 'navigation' else 'run_pursuit')
    from citysample_runtime import CitySampleBackend
    from urban_scene.probe_urban_scene import camera_pose
    from projectairsim import World, Drone, Rover
    from projectairsim.types import Pose

    class CityVideoBackend(CitySampleBackend):
        def observation(b, name, identifier):
            result = super().observation(name, identifier)
            if args.task == 'tracking' and name == 'Target':
                if not hasattr(b, 'video_diagnostics'):
                    b.video_diagnostics = (folder/'blocker_observation_diagnostics.jsonl').open('w', encoding='utf-8')
                b.video_diagnostics.write(json.dumps(serial({
                    'sim_time_sec': b.now(), 'diagnostic': b.observation_diagnostics.get(name),
                    'observation': ({k:v for k,v in result.items() if k not in {'raw','capture'}} if result else None)
                }))+'\n')
                b.video_diagnostics.flush()
            return result

        def load(b):
            b.config['scene'] = str(scene)
            b.world = World(b.client, str(scene), sim_config_path=str(scene.parent), delay_after_load_sec=2)
            resolved = b.world.get_configuration()
            if args.task == 'tracking':
                b.config['recording_physics_scope'] = 'Existing navigation center-ground-contact isolation links atNEDz=-0.45m; simple-drive kinematics; no full wheel-ground contact validation'
                run_path = Path(json.loads((folder/'experiment_link.json').read_text())['path'])
                write_json(run_path/'config.json', b.config)
                (folder/'contact_helper_source.py').write_bytes((ROOT/'citysample_scene/prepare_aerial_route_navigation.py').read_bytes())
                original_ids = b.world.get_segmentation_id_map()
                if not b.world.set_segmentation_id_by_name('.*', 0, True, True):
                    raise RuntimeError('Could not reset conflicting background segmentation IDs')
                write_json(folder/'segmentation_reservation.json', {
                    'background_id': 0, 'reserved_vehicle_ids': {'Target': 210, 'GateLeft': 211, 'GateRight': 212},
                    'previous_conflicting_objects': {str(i): [k for k,v in original_ids.items() if v == i]
                                                     for i in (210,211,212)},
                    'method': 'Scene-local stencil reset before original experiment vehicle ID assignment'})
            leaves = {0: 'scene_camera', 2: 'depth_camera', 3: 'segmentation_camera'}
            for actor in resolved['actors']:
                name = actor['name']
                root = b.world.parent_topic + '/robots/' + name
                if name != 'Observer':
                    kind = 'uav' if name.startswith('Drone') else 'ugv'
                    b.kinds[name] = kind
                    b.robots[name] = (Drone if kind == 'uav' else Rover)(b.client, b.world, name)
                    b.subscribe(name+'/pose', root+'/actual_pose')
                    b.subscribe(name+'/collision', root+'/collision_info')
                for sensor in actor['robot-config'].get('sensors', []):
                    if not sensor.get('enabled'):
                        continue
                    base = root+'/sensors/'+sensor['id']+'/'
                    if sensor['type'] == 'lidar':
                        b.subscribe(name+'/lidar', base+'lidar')
                    if sensor['type'] == 'camera':
                        for setting in sensor['capture-settings']:
                            leaf = leaves.get(setting['image-type'])
                            if leaf and setting.get('streaming-enabled'):
                                key = name + ('/'+sensor['id'] if name == 'Observer' else '') + '/' + leaf
                                b.subscribe(key, base+leaf)
            b.follow_task = asyncio.create_task(b.follow())
            b.world.resume()
            return resolved

        async def follow(b):
            while True:
                await asyncio.sleep(.6 if args.task == 'tracking' else .35)
                raw = b.buffer.latest(('Car1' if args.task == 'navigation' else 'Target')+'/pose', 2)
                if raw is None:
                    continue
                from ag.math3d import pose_parts
                p, r = pose_parts(raw)
                yaw = math.atan2(r[1, 0], r[0, 0])
                offset = (np.array([-15*math.cos(yaw)-10*math.sin(yaw),
                                    -15*math.sin(yaw)+10*math.cos(yaw), -16])
                          if args.task == 'navigation' else np.array([-24, -30, -42]))
                pose = camera_pose(p+offset, p+np.array([0, 0, -1]), Pose)
                b.client.request({'method': b.world.parent_topic+'/robots/Observer/sensors/VideoFollow/SetPose',
                                  'params': {'pose': pose, 'wait_for_pose_update': False}, 'version': 1.0})

        async def cleanup(b):
            if getattr(b, 'follow_task', None):
                b.follow_task.cancel()
                await asyncio.gather(b.follow_task, return_exceptions=True)
            result = await super().cleanup()
            if getattr(b, 'video_diagnostics', None):
                b.video_diagnostics.close()
            return result
    module.CitySampleBackend = CityVideoBackend
    new_run = module.new_run

    def create_run(task, suffix):
        path = new_run(task, suffix)
        write_json(folder/'experiment_link.json', {'path': str(path), 'task': task})
        return path
    module.new_run = create_run
    return (await module.run(700) if args.task == 'navigation' else
            await module.run(0, 180, False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', choices=['landing', 'mapping', 'navigation', 'tracking'])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    os.chdir(ROOT)
    ports = [9189, 9190] if args.task == 'landing' else [8989, 8990]
    deadline = time.monotonic() + 1200
    last_log = 0
    while True:
        try:
            for port in ports:
                with socket.create_connection(('127.0.0.1', port), timeout=1):
                    pass
            break
        except OSError:
            if time.monotonic() > deadline:
                raise TimeoutError('Simulator did not finish loading')
            if time.monotonic()-last_log > 30:
                print('Waiting for simulator ports '+str(ports), flush=True)
                last_log = time.monotonic()
            time.sleep(2)
    args.output.mkdir(parents=True, exist_ok=False)
    class Console:
        def __init__(self, original, logfile):
            self.original, self.logfile = original, logfile

        def write(self, value):
            self.original.write(value)
            self.logfile.write(value)
            self.logfile.flush()

        def flush(self):
            self.original.flush()
            self.logfile.flush()
    sys.stdout = Console(sys.stdout, (args.output/'recording_console.log').open('w', encoding='utf-8'))
    scene = scene_copy(args.task, args.output)
    (args.output/'record_experiment_source.py').write_bytes(Path(__file__).read_bytes())
    receipt_path = args.output.parent/('mapping_launch_receipt.json' if args.task == 'mapping' else 'citysample_launch_receipt.json')
    if args.task in {'mapping','navigation','tracking'} and receipt_path.exists():
        (args.output/'simulator_launch_receipt.json').write_bytes(receipt_path.read_bytes())
    recording = Recording(args.output/'camera_originals', args.task)
    record_callback(recording)
    try:
        result = asyncio.run(run(args, scene, args.output))
        print('EXPERIMENT RESULT ' + json.dumps(serial(result)), flush=True)
    finally:
        recording.close()
    print('VIDEO OUTPUT ' + str(args.output), flush=True)


if __name__ == '__main__':
    main()
