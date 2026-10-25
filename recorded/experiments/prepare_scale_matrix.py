"""Fixed per-platform workload, configurable fleet-size validation candidates."""
import copy
import json
from pathlib import Path
from ag.storage import ROOT, write_json
from prepare_cooperative_studies import look_at


def generate():
    root=Path('F:/EpicGamesEnv/MovingDeckLab/configs/scale_matrix')
    root.mkdir(parents=True,exist_ok=True)
    base=ROOT/'configs'
    matrix=[]
    for aircraft,cars in [(1,1),(2,2),(3,2),(4,4)]:
        directory=root/f'uav{aircraft}_ugv{cars}';directory.mkdir(exist_ok=True)
        config=json.loads((base/'formation_study.json').read_text(encoding='utf-8'))
        scene=json.loads((base/'scene_formation_study.jsonc').read_text(encoding='utf-8'))
        actors=[];routes={};ids={}
        for kind,count,prefix in [('ugv',cars,'Car'),('uav',aircraft,'Drone')]:
            robot=json.loads((base/f'robot_{kind}_formation_study.jsonc').read_text(encoding='utf-8'))
            write_json(directory/f'robot_{kind}.jsonc',robot)
            for index in range(count):
                east=127+13*index+(6 if kind=='uav' else 0)
                name=prefix+str(index+1)
                actors.append({'name':name,'type':'robot','robot-config':f'robot_{kind}.jsonc',
                    'origin':{'xyz':f'40.5 {east} -4','rpy-deg':'0 0 90'}})
                z=-10.688 if kind=='uav' else -.688
                routes[name]=[[40.5,east,z],[40.5,east+10,z]]
                ids[name]=(210 if kind=='uav' else 230)+index
        observer=json.loads((base/'robot_observer_formation_study.jsonc').read_text(encoding='utf-8'))
        for sensor,position,target in zip(observer['sensors'],[[8,127,-32],[38,158,-54]],[[40,158,-4],[40,158,-4]]):
            sensor['origin']={'xyz':' '.join(map(str,position)),
                'rpy-deg':' '.join(map(str,look_at(position,target)))}
        write_json(directory/'observer.jsonc',observer)
        actors.append({'name':'Observer','type':'robot','robot-config':'observer.jsonc',
            'origin':{'xyz':'0 0 0','rpy-deg':'0 0 0'}})
        scene.update(id=f'AGScale{aircraft}UAV{cars}UGV',actors=actors)
        write_json(directory/'scene.jsonc',scene)
        config.update(scene='scene.jsonc',sim_config_path=str(directory),port_topics=9089,port_services=9090,
            process_hint='MovingDeckLab',controlled_actor_count=aircraft+cars,routes=routes,
            ground_z_ned=-.688,uav_altitude_agl_m=10,duration_sec=90,
            scenario_status='Scale-matrix candidate; paths and common sensing fixed by platform type; live validation pending')
        config['formation']['explicit_routes']=True
        config['study'].update(segmentation_ids=ids,minimum_motion_m=9.,fixed_measurement_sec=60.,lossless_payload_compression=True)
        config['scale_protocol']={'uav_count':aircraft,'ugv_count':cars,
            'translation_per_platform_m':10,'uav_height_agl_m':10,'measurement_sec':60,
            'load_control':'Same robot file per type, camera/LiDAR resolution/rate and per-agent route length across counts. Observer load constant.',
            'metric_scope':'Includes common takeoff and route-start overhead; source timestamps define actual duration. No algorithm superiority claim.',
            'validation_status':'not_run'}
        write_json(directory/'study.json',config)
        matrix.append(str(directory/'study.json'))
    write_json(root/'matrix.json',{'configs':matrix,'status':'prepared_not_run'})
    print(root/'matrix.json')


if __name__=='__main__':generate()
