"""Generate separate dense-capture configs; the four baseline configs remain intact."""
import copy
import json
from ag.storage import ROOT, write_json


def generate():
    cfg = ROOT/'configs'
    for kind in ('uav','ugv'):
        robot = json.loads((cfg/f'robot_{kind}.jsonc').read_text(encoding='utf-8'))
        for sensor in robot['sensors']:
            if sensor['type']=='camera':
                sensor['capture-interval'] = .2
                for capture in sensor['capture-settings']:
                    capture.update(width=640,height=480)
            elif sensor['type']=='lidar':
                sensor['range'] = 60
                sensor['points-per-second'] = 40000
                sensor['number-of-channels'] = 32
        write_json(cfg/f'robot_{kind}_dense.jsonc', robot)
    scene = json.loads((cfg/'scene_mapping.jsonc').read_text(encoding='utf-8'))
    scene['id'] = 'AGMappingDense'
    for actor in scene['actors']:
        actor['robot-config'] = 'robot_uav_dense.jsonc' if actor['name']=='Drone1' else 'robot_ugv_dense.jsonc'
    write_json(cfg/'scene_mapping_dense.jsonc', scene)
    config = json.loads((cfg/'mapping.json').read_text(encoding='utf-8'))
    config.update(scene='scene_mapping_dense.jsonc',duration_sec=100,uav_altitude_agl_m=28.48,
                  sensor_max_age_sec=1.5,process_pid=None)
    config['routes']['Drone1'] = [[20,58,-24],[10,48,-24],[30,48,-24],[30,68,-24],[10,68,-24],[20,58,-24]]
    config['dense_capture'] = {'min_interval_sec':.4,'depth_max_m':45.,'fov_deg':90.,
                             'edge_threshold_m':.6,'robot_segmentation_ids':{'Drone1':240,'Car1':241}}
    config['scenario_status'] = 'Dense RGB-D coverage above upper plaza; planar ground vehicle route unchanged'
    write_json(cfg/'mapping_dense.json',config)
    print('Independent RGB-D mapping configs created')


if __name__=='__main__':
    generate()
