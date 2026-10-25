"""Prepare isolated moving-deck configs; never edit the Unreal project or DLLs."""
import copy
import json
from pathlib import Path
import commentjson
from ag.storage import ROOT, write_json


def prepare():
    workspace = ROOT.parent
    source = workspace / 'CityPark1/sim_config'
    folder = ROOT / 'configs/landing'
    folder.mkdir(parents=True, exist_ok=True)
    drone = commentjson.loads((source / 'robot_quadrotor_fastphysics_sga.jsonc').read_text(encoding='utf-8'))
    drone['sensors'] = [s for s in drone['sensors'] if s['id'] == 'DownCamera' or s['type'] in ('imu', 'gps', 'barometer', 'magnetometer')]
    camera = next(s for s in drone['sensors'] if s['id'] == 'DownCamera')
    camera['capture-interval'] = .1
    camera['capture-settings'] = [dict(camera['capture-settings'][0], width=1920, height=1440,
                                      **{'fov-degrees': 120, 'streaming-enabled': True})]
    camera['capture-settings'].append(dict(camera['capture-settings'][0], **{'image-type': 1}))
    vessel = commentjson.loads((source / 'robot_vessel_fastphysics_sga.jsonc').read_text(encoding='utf-8'))
    vessel['sensors'] = []
    # Keep the original visible tag and calibrated legacy offset. Explicitly
    # enable its collision; runtime contact, not this flag, proves support.
    tag = next(l for l in vessel['links'] if l['name'] == 'landing_tag')
    tag['inertial'] = {'mass': .01, 'inertia': {'type': 'point-mass'}}
    tag['collision'] = {'enabled': False}
    # The inherited strip stretches its marker texture. Static RGB-D measured
    # width/height = 0.26212/0.22177; correct only the isolated marker's Y scale.
    tag['visual']['geometry']['scale'] = '0.45 0.5319 0.3'
    tag['visual']['origin'] = {'xyz': '-3.0 0.0 -2.77', 'rpy-deg': '0 0 180'}
    support = {'name': 'landing_support', 'inertial': {'mass': .1, 'inertia': {'type': 'point-mass'}},
       'collision': {'enabled': True, 'restitution': .02, 'friction': .85},
       'visual': {'origin': {'xyz': '-3.0 0.0 -2.75', 'rpy-deg': '0 0 180'},
         'geometry': {'type': 'unreal_mesh', 'scale': '2.2 1.6 1.0',
           'name': '/Game/Models/Vessel/Apriltag/Game/CityPark/Maps/_GENERATED/Stephen/SM_Landing_Tag'}}}
    vessel['links'].append(support)
    vessel['joints'].append({'id': 'hull_support', 'type': 'fixed', 'parent-link': 'hull',
                            'child-link': 'landing_support', 'axis': '0 0 1'})
    # The legacy working landing config disables hull collision, leaving a
    # dedicated support panel. Hull collisions on the lake floor destabilize
    # the existing water-height bridge before initialization.
    vessel['links'][0]['collision']['enabled'] = False
    observer = {'physics-type': 'non-physics',
                'links': [{'name': 'Frame', 'collision': {'enabled': False}}],
                'sensors': [{'id': 'Overview', 'type': 'camera', 'enabled': True, 'parent-link': 'Frame',
                             'origin': {'xyz': '0 0 0', 'rpy-deg': '0 0 0'}, 'capture-interval': .5,
                             'capture-settings': [{'image-type': i, 'width': 1920, 'height': 1080,
                               'fov-degrees': 75, 'capture-enabled': True, 'streaming-enabled': True,
                               'pixels-as-float': False, 'compress': False} for i in (0, 3)]}]}
    scene = commentjson.loads((source / 'scene_citypark_sga_landing.jsonc').read_text(encoding='utf-8'))
    scene['id'] = 'AGMovingDeckEvidence'
    scene['clock']['pause-on-start'] = True
    scene['actors'] = [a for a in scene['actors'] if a['name'] != 'Car1']
    for actor in scene['actors']:
        actor['robot-config'] = 'robot_uav.jsonc' if actor['name'] == 'Drone1' else 'robot_usv.jsonc'
    scene['actors'].append({'name': 'Observer', 'type': 'robot', 'robot-config': 'robot_observer.jsonc',
                            'origin': {'xyz': '0 0 0', 'rpy-deg': '0 0 0'}})
    for name, data in [('robot_uav.jsonc', drone), ('robot_usv.jsonc', vessel),
                       ('robot_observer.jsonc', observer), ('scene_landing.jsonc', scene)]:
        write_json(folder / name, data)
    config = {
      'task': 'moving_deck_landing', 'sdk_path': str(workspace/'ProjectAirSim/client/python/projectairsim/src'),
      'ip': '127.0.0.1', 'port_topics': 8989, 'port_services': 8990,
      'scene': 'scene_landing.jsonc', 'sim_config_path': str(folder),
      'uav': 'Drone1', 'usv': 'Vessel1', 'camera': 'DownCamera',
      'dt_sec': .1, 'timeout_sec': 110., 'wall_timeout_sec': 900.,
      'tag_family': 'tag36h11', 'tag_ids': [1], 'tag_size_m': .4, 'camera_fov_deg': 120.,
      'tag_offset_hull_m': [-3., 0., -2.77],
      'tag_geometry_status': 'RGB-D calibrated: 71 stationary observations, median side 0.39996 m; centre agrees within 3 mm',
      'calibration_file': str(ROOT/'outputs/landing/20260928_062726_457859/trial_001_v0.00_predictive/derived/tag_calibration.json'),
      'speeds_mps': [0., .5], 'repeats': 1, 'controller': 'predictive',
      'approach_altitude_m': 4.5, 'max_speed_mps': 3., 'kp': .8,
      'feedforward': 1., 'prediction_sec': .15, 'velocity_alpha': .25,
      'descend_rate_mps': .3, 'flare_rate_mps': .10, 'flare_height_m': 1.4,
      'descent_error_m': .22, 'descent_relative_speed_mps': .3,
      'image_max_age_sec': .25, 'lost_target_abort_sec': 4.,
      'tag_min_margin': 20., 'tag_max_hamming': 0,
      'contact_horizontal_radius_m': .55, 'contact_height_range_m': [-.05, .65],
      'contact_relative_speed_mps': .2, 'hold_sec': 3., 'collision_max_age_sec': .3,
      'min_moving_speed_mps': .15, 'max_sample_gap_sec': .4,
      'usv_segmentation_id': 242, 'uav_segmentation_id': 240,
      'vessel_force_per_mps': 125., 'vessel_speed_kp': 80., 'vessel_max_force': 450.,
      'observer_interval_sec': .5,
      'trial_bounds_ned': {'x': [15., 45.], 'y': [-35., 45.], 'z': [-15., 11.]},
      'provenance': {
        'navigation': 'Simulator ego pose and vessel truth for coarse approach only',
        'visual_control': 'AprilTag camera measurement + simulator camera/ego pose; no target truth after acquisition',
        'vessel_model': 'Existing compiled service bridge; source implements thrust-sum kinematic motion and water-height lock, not validated hydrodynamics',
        'contact': 'Fresh vessel collision event + motor-off relative stability; missing contact is not success',
        'reference_programs': [str(workspace/'AirSimSGA_ExperimentKit/sga_experiments/projectairsim_real.py'),
           str(workspace/'CityPark2/PythonClient/citypark2_uav1_vessel3_moving_landing_predictive.py')]}}
    write_json(folder/'study.json', config)
    return folder/'study.json'


if __name__ == '__main__':
    print(prepare())
