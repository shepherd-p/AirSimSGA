"""Copy selected existing robot definitions into this kit; never write to UE/SDK."""
import copy
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf-8')


def protected_files():
    for base in (WORKSPACE/'CityPark1', WORKSPACE/'ProjectAirSim/client/python/projectairsim/src'):
        for p in base.rglob('*'):
            if p.is_file() and not any(x in p.parts for x in ('Intermediate', 'Saved', '__pycache__', '.git')):
                if p.suffix.lower() in ('.dll', '.uproject', '.uplugin', '.ini', '.jsonc', '.json', '.py', '.cpp', '.h', '.hpp', '.cs'):
                    yield p


def snapshot():
    baseline = ROOT/'verification/protected_files.json'
    if baseline.exists():
        raise FileExistsError('Baseline already exists; refusing to replace original fingerprints')
    dump(baseline, {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in protected_files()})


def generate():
    source = WORKSPACE/'CityPark1/sim_config'
    base = json.loads((source/'scene_citypark_sga.jsonc').read_text(encoding='utf-8-sig'))
    for kind, original, camera in [('uav', 'robot_quadrotor_fastphysics_sga.jsonc', 'DownCamera'),
                                   ('ugv', 'robot_rover_fastphysics_sga.jsonc', 'FrontCamera')]:
        robot = json.loads((source/original).read_text(encoding='utf-8-sig'))
        robot['sensors'] = [s for s in robot['sensors'] if s['id'] in (camera, 'lidar1')]
        for s in robot['sensors']:
            if s['type'] == 'camera':
                s['capture-interval'] = .1
                if kind == 'ugv':
                    s['origin']['xyz'] = '2.5 0 -1.8'
                for capture in s['capture-settings']:
                    capture.update({'width': 320, 'height': 240, 'streaming-enabled': True})
            if s['type'] == 'lidar':
                s['points-per-second'] = 20000
                if kind == 'uav':
                    s['vertical-fov-lower-deg'] = -85.
        dump(ROOT/f'configs/robot_{kind}.jsonc', robot)

    def actor(name, kind, p, yaw=0):
        return {'type': 'robot', 'name': name,
                'origin': {'xyz': ' '.join(map(str, p)), 'rpy-deg': f'0 0 {yaw}'},
                'robot-config': f'robot_{kind}.jsonc'}

    pair = [actor('Drone1', 'uav', [20,58,2.8]), actor('Car1', 'ugv', [16,60,2.8], 0)]
    formation = [actor(f'Drone{i+1}', 'uav', [27+10*(i%2), 52+10*(i//2), 2.8]) for i in range(4)]
    formation += [actor(f'Car{i+1}', 'ugv', [18+5*math.cos(i*2*math.pi/3),57+5*math.sin(i*2*math.pi/3),2.8],90+120*i) for i in range(3)]
    for task in ('mapping', 'navigation', 'tracking', 'formation'):
        scene = {k: copy.deepcopy(v) for k, v in base.items() if k != 'actors'}
        scene['id'] = 'AG'+task.title()
        scene['actors'] = copy.deepcopy(formation if task == 'formation' else pair)
        if task == 'tracking':
            scene['actors'][0]['origin']['xyz'] = '14 54 2.8'
            scene['actors'][1]['origin'].update({'xyz':'18 52.5 2.8','rpy-deg':'0 0 90'})
            scene['actors'].append(actor('Target', 'ugv', [18,59,2.8], 90))
        dump(ROOT/f'configs/scene_{task}.jsonc', scene)
        config = {
            'task': task, 'sdk_path': str(WORKSPACE/'ProjectAirSim/client/python/projectairsim/src'),
            'scene': f'scene_{task}.jsonc', 'ip': '127.0.0.1', 'port_topics': 8989, 'port_services': 8990,
            'duration_sec': 90, 'control_hz': 5, 'sensor_max_age_sec': .6,
            'sensor_pair_tolerance_sec': .12, 'uav_altitude_agl_m': 12,
            'uav_speed_mps': 2., 'ugv_speed_mps': 1., 'ground_z_ned': 4.48,
            'rover_surface_offset_m': 0., 'stop_on_collision': True,
            'waypoint_tolerance_m': 1.5, 'minimum_separation_m': 3.,
            'process_hint': 'CityPark1', 'cleanup': 'hover',
            'routes': {'Drone1': [[20,58,-7.52],[28,58,-7.52],[28,62,-7.52],[20,62,-7.52]],
                       'Car1': [[23,60,4.48],[25,60,4.48]]},
            'navigation': {'scan_sec': 20, 'map_source':'uav_lidar', 'goal_ned': [25,60,4.48], 'resolution_m': .75,
                           'height_bin_m': .35, 'max_step_m': .25, 'max_slope': .30,
                           'clearance_m': 1.6, 'vehicle_radius_m': 1.1, 'stop_distance_m': 4.},
            'tracking': {'target_id': 42, 'follow_distance_m': 6., 'uav_offset_ned': [0,0,-12],
                         'prediction_horizon_sec': 2., 'target_route': [[18,65,4.48]],
                         'minimum_pixels': 8, 'segmentation_color_order': 'bgr',
                         'canopy': {'enabled': True, 'center_ned': [18,62,-1.5], 'scale': [7,2.5,.5]}},
            'formation': {'square_side_m': 6., 'circle_radius_m': 5., 'circle_center_ned':[18,57,4.48], 'laps': 1},
            'scenario_status': 'Upper plaza terrain chosen from live LiDAR; fast-physics rover has planar grounded motion',
        }
        dump(ROOT/f'configs/{task}.json', config)


if __name__ == '__main__':
    if not (ROOT/'verification/protected_files.json').exists():
        snapshot()
    generate()
    print('Independent configs generated; original project files untouched.')
