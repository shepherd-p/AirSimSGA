"""Independent configurations for the three remaining paper-style experiments."""
import copy
import json
import math
from ag.storage import ROOT,write_json
from prepare_plaza_configs import rounded_loop


def look_at(position,target):
    x,y,z=[b-a for a,b in zip(position,target)]
    return [0,-math.degrees(math.atan2(z,math.hypot(x,y))),math.degrees(math.atan2(y,x))]


def generate():
    cfg=ROOT/'configs'
    for task in ('navigation','tracking','formation'):
        c=json.loads((cfg/f'{task}.json').read_text(encoding='utf-8'))
        scene=json.loads((cfg/f'scene_{task}.jsonc').read_text(encoding='utf-8'))
        scene['id']='AG'+task.title()+'Study'
        c.update(scene=f'scene_{task}_study.jsonc',duration_sec=210 if task!='formation' else 100,
                 full_lidar_capture=True,uav_speed_mps=2.5,ugv_speed_mps=1.0,
                 waypoint_tolerance_m=1.0,sensor_max_age_sec=1.5,sensor_pair_tolerance_sec=.03)
        c['study']={'version':1,'frame_interval_sec':.4 if task=='tracking' else .5,
                    'overview_interval_sec':1.,'minimum_motion_m':20.,
                    'segmentation_ids':{},'pose_prior':'simulator ego localization',
                    'camera_capture':'Selected synchronized original RGB/depth/segmentation payloads',
                    'rendering_mode':'Existing UE instance, actual mode recorded separately; cameras active'}
        for kind in ('uav','ugv'):
            robot=json.loads((cfg/f'robot_{kind}_plaza.jsonc').read_text(encoding='utf-8'))
            for sensor in robot['sensors']:
                if sensor['type']=='camera':
                    sensor['capture-interval']=.2
                    for capture in sensor['capture-settings']:
                        capture.update(width=640 if task=='formation' else 960,height=360 if task=='formation' else 540)
                elif sensor['type']=='lidar':
                    sensor['points-per-second']=90000 if task=='navigation' else 20000
                    sensor['report-frequency']=20 if task=='navigation' else 10
                    if task!='navigation':sensor['number-of-channels']=1 if kind=='uav' else 32
            write_json(cfg/f'robot_{kind}_{task}_study.jsonc',robot)
        for index,actor in enumerate(scene['actors']):
            name=actor['name'];kind='uav' if name.startswith('Drone') else 'ugv'
            actor['robot-config']=f'robot_{kind}_{task}_study.jsonc'
            c['study']['segmentation_ids'][name]=42 if name=='Target' else (240 if name=='Drone1' else 241 if name=='Car1' else 220+index)
        if task=='navigation':
            scene['actors'][0]['origin']['xyz']='20 58 2.8'
            scene['actors'][1]['origin']['xyz']='16 57 2.8'
            c['routes']={'Drone1':[[20,58,-24],[25,52,-24],[44,52,-24],[44,64,-24],[25,64,-24],[33,57,-26]],'Car1':[]}
            c['navigation'].update(goal_ned=[47,57,4.48],resolution_m=.5,vehicle_radius_m=1.55,
                                   stop_distance_m=2.8,scan_sec=30,
                                   roi_xy=[[10,46],[54,69]],floor_tolerance_m=.2,
                                   obstacle_height_range_m=[.4,2.8],minimum_cell_points=3)
        elif task=='tracking':
            target_robot=json.loads((cfg/'robot_ugv_tracking_study.jsonc').read_text(encoding='utf-8'))
            for sensor in target_robot['sensors']:
                if sensor['type']=='camera':sensor['enabled']=False
            write_json(cfg/'robot_target_tracking_study.jsonc',target_robot)
            scene['actors'][2]['robot-config']='robot_target_tracking_study.jsonc'
            for actor,position,yaw in zip(scene['actors'],['20 55 2.8','18.5 50.5 2.8','25 50.5 2.8'],[0,0,0]):
                actor['origin']={'xyz':position,'rpy-deg':f'0 0 {yaw}'}
            c.update(uav_altitude_agl_m=14.,ugv_speed_mps=1.1)
            c['tracking'].update(target_route=rounded_loop(21,48,50.5,63,4,4.48),
                                  target_speed_mps=.75,follow_distance_m=6.,uav_offset_ned=[0,0,-14],minimum_pixels=15)
            c['tracking']['canopy']={'enabled':True,'center_ned':[40,50.5,-.7],'scale':[5,6,.35]}
            c['study'].update(minimum_motion_m=60.,minimum_ground_only_frames=3)
        else:
            c['study'].update(minimum_motion_m=20.,minimum_laps=1)
            c['formation']['laps']=1
            # Clearance checked against the navigation run's observed ground.
            # The original circle intersected Bench03HISMA_519 near (23, 57).
            c['formation']['circle_center_ned']=[16.,54.,4.48]
            c['formation']['circle_radius_m']=5.
            for index,actor in enumerate(a for a in scene['actors'] if a['name'].startswith('Car')):
                angle=index*2*math.pi/3
                actor['origin']={'xyz':f'{16+5*math.cos(angle)} {54+5*math.sin(angle)} 2.8',
                                 'rpy-deg':f'0 0 {math.degrees(angle)+90}'}
        # Real camera sensors attached to invisible, non-physical observer.
        views=[('Overview',[5,33,-30],[29,57,-1]),('Top',[28,57,-45],[28,57,4.48])]
        observer={'physics-type':'non-physics','links':[{'name':'Frame','collision':{'enabled':False}}],'sensors':[]}
        for name,position,target in views:
            observer['sensors'].append({'id':name,'type':'camera','enabled':True,'parent-link':'Frame',
                'capture-interval':1.,'origin':{'xyz':' '.join(map(str,position)),
                'rpy-deg':' '.join(map(str,look_at(position,target)))},
                'capture-settings':[{'image-type':i,'width':1920,'height':1080,'fov-degrees':80,
                    'capture-enabled':True,'streaming-enabled':True,'pixels-as-float':False,'compress':False} for i in (0,3)]})
        write_json(cfg/f'robot_observer_{task}_study.jsonc',observer)
        scene['actors'].append({'name':'Observer','type':'robot','robot-config':f'robot_observer_{task}_study.jsonc',
                                'origin':{'xyz':'0 0 0','rpy-deg':'0 0 0'}})
        write_json(cfg/c['scene'],scene);write_json(cfg/f'{task}_study.json',c)
        print(f'{task}: {c["scene"]}',flush=True)


if __name__=='__main__':generate()
