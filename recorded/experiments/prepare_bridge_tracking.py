"""Create a bridge-area tracking candidate in the isolated simulator workspace.

The bridge box is evaluation-only. A completed run still needs measured camera
occlusion and reacquisition; scene geometry alone cannot establish either.
"""
import json
from pathlib import Path

from ag.storage import ROOT, write_json
from prepare_cooperative_studies import look_at


def generate():
    source = ROOT / 'configs'
    destination = Path('F:/EpicGamesEnv/MovingDeckLab/configs/bridge_tracking')
    destination.mkdir(parents=True, exist_ok=True)
    config = json.loads((source / 'tracking_study.json').read_text(encoding='utf-8'))
    scene = json.loads((source / config['scene']).read_text(encoding='utf-8'))

    for actor in scene['actors']:
        name = actor['name']
        robot = json.loads((source / actor['robot-config']).read_text(encoding='utf-8'))
        for sensor in robot['sensors']:
            if sensor['type'] == 'camera':
                sensor['capture-interval'] = 0.5 if name != 'Observer' else 1.5
                for capture in sensor['capture-settings']:
                    capture['width'] = 640 if name != 'Observer' else 1280
                    capture['height'] = 360 if name != 'Observer' else 720
        if name == 'Observer':
            for sensor, position, target in zip(robot['sensors'],
                    [[-65, 88, -32], [-35, 110, -48]],
                    [[-35, 110, 2], [-35, 110, 2]]):
                sensor['origin'] = {'xyz': ' '.join(map(str, position)),
                    'rpy-deg': ' '.join(map(str, look_at(position, target)))}
        write_json(destination / actor['robot-config'], robot)
        if name == 'Drone1':
            actor['origin'] = {'xyz': '-35 90 -20', 'rpy-deg': '0 0 0'}
        elif name == 'Car1':
            actor['origin'] = {'xyz': '-35 90 2.8', 'rpy-deg': '0 0 90'}
        elif name == 'Target':
            actor['origin'] = {'xyz': '-35 98 2.8', 'rpy-deg': '0 0 90'}

    scene['id'] = 'AGBridgeTrackingCandidate'
    tracking = config['tracking']
    tracking.pop('canopy', None)
    tracking['occlusion_region'] = {
        'center_ned': [-35, 110, 2], 'scale': [10, 15, 12],
        'source': 'CityPark native road bridge; evaluation window only'
    }
    tracking['target_route'] = [
        [-35, 100, 4.48], [-35, 105, 4.48], [-35, 110, 4.48],
        [-35, 119, 4.48], [-35, 128, 4.48], [-35, 137, 4.48]
    ]
    tracking['target_speed_mps'] = 0.7
    config.update(sim_config_path=str(destination), port_topics=9089, port_services=9090,
        process_hint='MovingDeckLab', controlled_actor_count=3, duration_sec=160,
        uav_altitude_agl_m=14, ugv_speed_mps=1.0,
        scenario_status='Bridge-road candidate; natural occlusion and traversability require live validation')
    config['routes']['Drone1'] = [[-35, 90, -13], [-35, 112, -13]]
    config['routes']['Car1'] = [[-35, 90, 4.48]]
    config['study'].update(minimum_motion_m=25, minimum_ground_only_frames=3,
        frame_interval_sec=0.4)
    config['region'] = {'id': 'native_bridge_tracking',
        'survey': 'outputs/region_survey/20260928_064720/tracking_bridge_rgb.png',
        'survey_use': 'Scenario placement only; not a tracking observation',
        'natural_occluder': 'CityPark road bridge; no spawned canopy'}
    write_json(destination / config['scene'], scene)
    write_json(destination / 'study.json', config)
    print(destination / 'study.json')


if __name__ == '__main__':
    generate()
