"""Independent east-garden navigation candidate; survey is placement-only."""
import json
from pathlib import Path
from ag.storage import ROOT, write_json
from prepare_cooperative_studies import look_at


def generate():
    directory=Path('F:/EpicGamesEnv/MovingDeckLab/configs/garden_navigation')
    directory.mkdir(parents=True,exist_ok=True)
    base=ROOT/'configs'
    config=json.loads((base/'navigation_study.json').read_text(encoding='utf-8'))
    scene=json.loads((base/'scene_navigation_study.jsonc').read_text(encoding='utf-8'))
    for actor in scene['actors']:
        filename=actor['robot-config']
        robot=json.loads((base/filename).read_text(encoding='utf-8'))
        if actor['name']=='Observer':
            for sensor,position,target in zip(robot['sensors'],[[8,117,-32],[32,153,-49]],[[32,153,-1],[32,153,-1]]):
                sensor['origin']={'xyz':' '.join(map(str,position)),
                                  'rpy-deg':' '.join(map(str,look_at(position,target)))}
        write_json(directory/filename,robot)
    for actor in scene['actors']:
        if actor['name']=='Drone1':actor['origin']={'xyz':'42 142 -4','rpy-deg':'0 0 0'}
        elif actor['name']=='Car1':actor['origin']={'xyz':'42 132 -4','rpy-deg':'0 0 90'}
    scene['id']='AGEastGardenNavigation'
    config.update(sim_config_path=str(directory),port_topics=9089,port_services=9090,
        process_hint='MovingDeckLab',controlled_actor_count=2,ground_z_ned=-.688,
        duration_sec=240,uav_altitude_agl_m=18,uav_speed_mps=3.,
        scenario_status='Candidate east-garden paths and planters, distinct from accepted lakeside mapping plaza; live clearance/goal validation pending')
    config['routes']={'Drone1':[[42,142,-23],[20,125,-20],[20,178,-20],
        [32,178,-20],[43,178,-20],[43,125,-20],[32,125,-20],[32,153,-24]],'Car1':[]}
    config['navigation'].update(goal_ned=[24,171,-.688],roi_xy=[[17,121],[47,183]],
        floor_tolerance_m=.2,vehicle_radius_m=1.55,minimum_cell_points=3)
    config['study']['minimum_motion_m']=30.
    config['region']={'id':'east_garden_navigation','north':[17,47],'east':[121,183],
        'survey':'outputs/region_survey/20260928_064720/east_garden_rgb.png',
        'survey_use':'Placement and flat-floor hypothesis only; not an input to the online UAV LiDAR planner',
        'selection_reason':'South-west road survey has about 2 m elevation variation over 50 m; existing planar rover cannot represent it faithfully. East garden offers a separately located paved terrace with actual planters.'}
    write_json(directory/config['scene'],scene)
    write_json(directory/'study.json',config)
    print(directory/'study.json')


if __name__=='__main__':generate()
