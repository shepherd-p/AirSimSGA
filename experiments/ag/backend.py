"""Client-only integration. Loading a scene is explicit and never a preflight side effect."""
import asyncio
from collections import defaultdict, deque
import math
import socket
import sys
import threading
import time
from pathlib import Path
import numpy as np
from .math3d import pose_parts, depth_world, lidar_world
from .storage import ROOT


class Buffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = defaultdict(lambda: deque(maxlen=20))
        self.counts = defaultdict(int)
        self.sink = None

    def callback(self, key):
        def receive(_, message):
            if isinstance(message, dict):
                with self.lock:
                    self.data[key].append((time.monotonic(), message))
                    self.counts[key] += 1
                if self.sink is not None:
                    self.sink(key, message)
        return receive

    def entries(self, key):
        with self.lock:
            return list(self.data[key])

    def latest(self, key, max_age=.6):
        entries = self.entries(key)
        return entries[-1][1] if entries and time.monotonic()-entries[-1][0] <= max_age else None

    def pair(self, a, b, max_age, tolerance):
        aa, bb = self.entries(a), self.entries(b)
        now = time.monotonic()
        aa = [m for t,m in aa if now-t <= max_age]
        bb = [m for t,m in bb if now-t <= max_age]
        if not aa or not bb:
            return None
        # Prefer the newest pair, provided capture times agree within tolerance.
        for x in reversed(aa):
            y = min(bb, key=lambda m: abs(int(m['time_stamp'])-int(x['time_stamp'])))
            if abs(int(y['time_stamp'])-int(x['time_stamp']))/1e9 <= tolerance:
                return x, y
        return None


class Backend:
    def __init__(self, config):
        self.config = config
        self.buffer = Buffer()
        self.client = self.world = None
        self.robots, self.kinds, self.armed = {}, {}, set()
        self.topics = {}
        self.last_pose = {}
        self.controlled = set()
        self.subscriptions = []
        self.previous_scene_ids = []
        self.spawned = []
        self.lock_file = None
        self.observation_diagnostics = {}
        self.observation_packets = {}

    def connect(self):
        self.lock_file = (ROOT/'verification/client.lock').open('a+b')
        self.lock_file.seek(0)
        if sys.platform == 'win32':
            import msvcrt
            try:
                msvcrt.locking(self.lock_file.fileno(),msvcrt.LK_NBLCK,1)
            except OSError:
                self.lock_file.close()
                self.lock_file = None
                raise RuntimeError('Another experiment/preflight client from this kit is already connected')
        else:
            import fcntl
            fcntl.flock(self.lock_file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for port in (self.config['port_services'], self.config['port_topics']):
            with socket.create_connection((self.config['ip'], port), timeout=3):
                pass
        source = Path(self.config['sdk_path'])
        if not (source/'projectairsim').is_dir():
            raise FileNotFoundError(source)
        sys.path.insert(0, str(source))
        from projectairsim import ProjectAirSimClient
        self.client = ProjectAirSimClient(address=self.config['ip'],
                                         port_topics=self.config['port_topics'],
                                         port_services=self.config['port_services'])
        self.client.connect()
        self.client.socket_services.recv_timeout = 60000
        self.client.get_topic_info()
        if not self.client.state or not self.client.topics:
            raise RuntimeError('No ProjectAirSim topic inventory arrived')
        self.previous_scene_ids = sorted({t.split('/')[2] for t in self.client.topics if t.startswith('/Sim/') and len(t.split('/')) > 2})

    def subscribe(self, key, topic):
        if topic not in self.client.topics:
            raise RuntimeError(f'Required topic unavailable: {topic}')
        self.topics[key] = topic
        self.client.subscribe(topic, self.buffer.callback(key))
        self.subscriptions.append(topic)

    async def inspect(self, seconds=5):
        """Only topic inventory/subscriptions; no World(), scene reload, arming or movement."""
        topics = [t for t in self.client.topics if any(t.endswith('/'+leaf) for leaf in
                  ('lidar', 'scene_camera', 'depth_camera', 'segmentation_camera', 'actual_pose'))]
        for t in topics:
            self.subscribe(t, t)
        await asyncio.sleep(seconds)
        return {'scene_ids': self.previous_scene_ids, 'loaded_scene': False,
                'streams': {t: {'messages': self.buffer.counts[t],
                                'receive_hz': self.buffer.counts[t]/seconds} for t in topics},
                'all_selected_streams_active': bool(topics) and all(self.buffer.counts[t] > 0 for t in topics)}

    def load(self):
        from projectairsim import World, Drone, Rover
        self.world = World(self.client, self.config['scene'], sim_config_path=str(ROOT/'configs'), delay_after_load_sec=1)
        scene = self.world.get_configuration()
        for item in scene['actors']:
            name = item['name']
            kind = 'uav' if name.startswith('Drone') else 'ugv'
            self.kinds[name] = kind
            self.robots[name] = (Drone if kind == 'uav' else Rover)(self.client, self.world, name)
            root = f'{self.world.parent_topic}/robots/{name}'
            camera = 'DownCamera' if kind == 'uav' else 'FrontCamera'
            self.subscribe(name+'/pose', root+'/actual_pose')
            self.subscribe(name+'/collision', root+'/collision_info')
            self.subscribe(name+'/lidar', root+'/sensors/lidar1/lidar')
            for leaf in ('scene_camera', 'depth_camera', 'segmentation_camera'):
                self.subscribe(name+'/'+leaf, root+f'/sensors/{camera}/'+leaf)
        actual = set(self.world.list_actors())
        if set(self.kinds) != actual:
            raise RuntimeError(f'Actor mismatch: configured {set(self.kinds)}, live {actual}')
        self.world.resume()

    async def ready(self, seconds=12):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            missing = [k for k in self.topics if not k.endswith('/collision') and self.buffer.latest(k, 2.) is None]
            if not missing:
                return
            await asyncio.sleep(.2)
        raise RuntimeError('No fresh messages for: '+', '.join(missing))

    async def settle(self):
        previous = {n:self.pose(n)['position'] for n in self.kinds}
        stable_since = None
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            await asyncio.sleep(.3)
            current = {n:self.pose(n)['position'] for n in self.kinds}
            stable = all(np.linalg.norm(current[n]-previous[n]) < .025 for n in current)
            previous = current
            if stable:
                if stable_since is None:
                    stable_since = self.now()
                if self.now()-stable_since >= .7:
                    return
            else:
                stable_since = None
        raise RuntimeError('Robots did not settle on terrain before arming')

    async def wait_for_pose_sync(self, timeout_sec=45, tolerance_m=0.5, stable_sec=1.0):
        """Wait for physics and service poses to agree before sending controls."""
        deadline = time.monotonic() + timeout_sec
        stable_since = None
        last_errors = {}
        while time.monotonic() < deadline:
            errors = {}
            for name in self.kinds:
                message = self.buffer.latest(name+'/pose', 1.0)
                if message is None:
                    errors[name] = None
                    continue
                service_position = self.pose(name)['position']
                topic_position = np.array([message['position'][axis] for axis in 'xyz'])
                errors[name] = float(np.linalg.norm(service_position-topic_position))
            last_errors = errors
            if errors and all(error is not None and error <= tolerance_m for error in errors.values()):
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic()-stable_since >= stable_sec:
                    return {'status':'passed','position_error_m':errors,
                            'tolerance_m':tolerance_m,'stable_sec':stable_sec}
            else:
                stable_since = None
            await asyncio.sleep(0.2)
        raise RuntimeError(f'Physics/service pose did not synchronize: {last_errors}')

    def create_canopy(self, options):
        from projectairsim.types import Pose, Vector3
        pose = Pose({'translation': Vector3(dict(zip('xyz',options['center_ned']))),
                     'rotation': {'w':1.,'x':0.,'y':0.,'z':0.}})
        assets = self.world.list_assets('^Cube$')
        if 'Cube' not in assets:
            raise RuntimeError('The runtime asset registry does not contain Cube')
        name = self.world.spawn_object('AGTrackingCanopy', 'Cube',
                                      pose, options['scale'], False)
        if not name:
            raise RuntimeError('Cannot spawn runtime occlusion canopy with the existing binary')
        self.spawned.append(name)
        if not self.world.set_segmentation_id_by_name(name, 43, False, True):
            raise RuntimeError('Runtime canopy could not be included in segmentation occlusion pass')
        return name

    def request(self, suffix, params=None, name=None):
        root = self.world.parent_topic + (f'/robots/{name}' if name else '')
        return self.client.request({'method': root+'/'+suffix, 'params': params or {}, 'version': 1.0})

    def now(self):
        return self.world.get_sim_time()/1e9

    def pose(self, name):
        started = time.perf_counter()
        p, r = pose_parts(self.world.get_object_pose(name))
        stamp = self.now()
        old = self.last_pose.get(name)
        v = (p-old[0])/(stamp-old[1]) if old and stamp-old[1] > .01 else (old[2] if old else np.zeros(3))
        self.last_pose[name] = p, stamp, v
        return {'name': name, 'time': stamp, 'position': p, 'yaw': math.atan2(r[1,0], r[0,0]),
                'velocity': v, 'latency_ms': (time.perf_counter()-started)*1000}

    async def settle_task(self, coroutine, timeout=10):
        task = await asyncio.wait_for(coroutine, timeout)
        value = await asyncio.wait_for(task, timeout)
        if value is False:
            raise RuntimeError('ProjectAirSim rejected control request')
        return value

    async def start(self):
        for name, robot in self.robots.items():
            if robot.enable_api_control() is False:
                raise RuntimeError(f'API control rejected for {name}')
            self.controlled.add(name)
            if robot.arm() is False:
                raise RuntimeError(f'Arming rejected for {name}')
            self.armed.add(name)
        drones = [r for n,r in self.robots.items() if self.kinds[n] == 'uav']
        results = await asyncio.gather(*(self.settle_task(r.takeoff_async(timeout_sec=30), 90) for r in drones),return_exceptions=True)
        for result in results:
            if isinstance(result,BaseException):
                raise result
        self.client.socket_services.recv_timeout = 12000

    async def rover(self, name, engine=0., steering=0., brake=1.):
        await self.settle_task(self.robots[name].set_rover_controls(float(engine), float(steering), float(brake)), 4)

    async def drone_velocity(self, name, velocity, yaw_rate=0.):
        await self.settle_task(self.robots[name].move_by_velocity_async(
            *[float(x) for x in velocity], duration=.12, yaw_is_rate=True, yaw=float(yaw_rate)), 4)

    def image(self, name, leaf):
        from projectairsim.utils import unpack_image
        msg = self.buffer.latest(name+'/'+leaf, self.config['sensor_max_age_sec'])
        return (msg, unpack_image(msg)) if msg is not None else (None, None)

    @staticmethod
    def depth_m(message, image):
        if message['encoding'] != '16UC1':
            raise RuntimeError('Unsupported depth encoding; units must be verified before use')
        return image.astype(float)/1000.

    def cloud(self, name):
        msg = self.buffer.latest(name+'/lidar', self.config['sensor_max_age_sec'])
        if msg is None:
            return None, None
        return msg, lidar_world(msg)

    def depth_points(self, name):
        msg, img = self.image(name, 'depth_camera')
        if msg is None:
            return None, None
        return msg, depth_world(self.depth_m(msg, img), msg, stride=3)

    def observation(self, name, target_id):
        from projectairsim.utils import unpack_image
        from projectairsim.image_utils import segmentation_id_to_color
        pair = self.buffer.pair(name+'/depth_camera', name+'/segmentation_camera',
                                self.config['sensor_max_age_sec'], self.config['sensor_pair_tolerance_sec'])
        self.observation_diagnostics[name] = {'reason':'no_fresh_synchronized_pair','pixels':0}
        self.observation_packets.pop(name,None)
        if pair is None:
            return None
        dm, sm = pair
        depth, seg = self.depth_m(dm, unpack_image(dm)), unpack_image(sm)
        color = segmentation_id_to_color(target_id)
        rgb = np.array([color.r, color.g, color.b], dtype=np.uint8)
        order = self.config['tracking'].get('segmentation_color_order','bgr')
        mask = np.all(seg[:,:,:3] == (rgb[::-1] if order == 'bgr' else rgb), axis=2)
        mask &= (depth > .1) & (depth < 65.)
        count = int(mask.sum())
        self.observation_packets[name] = {'time':int(dm['time_stamp'])/1e9,
                                         'raw':{'depth_m':depth,'segmentation':seg,'mask':mask},
                                         'capture':{k:v for k,v in dm.items() if k != 'data'}}
        self.observation_diagnostics[name] = {'reason':'target_not_in_image','pixels':count}
        if count < self.config['tracking']['minimum_pixels']:
            return None
        pts = depth_world(depth, dm, stride=1, mask=mask,
                          pixel_signs=self.config.get('depth_pixel_signs', (1, 1)))
        if len(pts) == 0:
            return None
        self.observation_diagnostics[name]['reason'] = 'visible'
        return {'time': int(dm['time_stamp'])/1e9, 'position': np.median(pts, axis=0),
                'confidence': min(1., count/500), 'pixels': count,
                'pair_delta_ms': abs(int(dm['time_stamp'])-int(sm['time_stamp']))/1e6,
                'raw': {'depth_m':depth, 'segmentation':seg, 'mask':mask},
                'capture': {k:v for k,v in dm.items() if k != 'data'}}

    def collision(self, name):
        result = self.buffer.latest(name+'/collision', 2.)
        if result is None:
            return None
        return result

    async def cleanup(self):
        errors = []
        if self.client is not None and self.client.socket_services is not None:
            try:
                self.client.socket_services.recv_timeout = 2000
            except Exception:
                pass
        for name in sorted(self.controlled):
            try:
                if self.kinds[name] == 'ugv':
                    await self.rover(name)
                elif name in self.armed:
                    self.robots[name].cancel_last_task()
                    await self.settle_task(self.robots[name].hover_async(), 5)
                    if self.config.get('cleanup') == 'land':
                        await self.settle_task(self.robots[name].land_async(timeout_sec=20), 25)
                        self.robots[name].disarm()
            except Exception as exc:
                errors.append(f'{name}: {exc}')
        for name in self.spawned:
            try:
                self.world.destroy_object(name)
            except Exception as exc:
                errors.append(f'Remove runtime canopy {name}: {exc}')
        self.close()
        return errors

    def close(self):
        if self.client is not None:
            try:
                self.client.disconnect()
            except Exception:
                pass
        if self.lock_file is not None:
            self.lock_file.close()
            self.lock_file = None
