"""Separate CityPark plaza loop and scan-pattern configuration."""
import json
import numpy as np
from ag.storage import ROOT, write_json


def rounded_loop(x0, x1, y0, y1, radius, z):
    points = []
    for cx, cy, a0 in [(x1-radius,y0+radius,-90),(x1-radius,y1-radius,0),
                       (x0+radius,y1-radius,90),(x0+radius,y0+radius,180)]:
        for a in np.linspace(a0,a0+90,7):
            a = np.deg2rad(a)
            points.append([cx+radius*np.cos(a),cy+radius*np.sin(a),z])
    # Start at the beginning of the long bottom straight, facing +N.
    start = [x0+radius,y0,z]
    return [start]+points+[start]


def generate():
    cfg = ROOT/'configs'
    for kind in ('uav','ugv'):
        robot = json.loads((cfg/f'robot_{kind}_dense.jsonc').read_text(encoding='utf-8'))
        for s in robot['sensors']:
            if s['type'] == 'lidar':
                s.update({'lidar-type':'generic_rosette' if kind=='uav' else 'generic_cylindrical',
                          'number-of-channels':1 if kind=='uav' else 64,
                          'points-per-second':120000,'report-frequency':20,
                          'vertical-fov-lower-deg':-85 if kind=='uav' else -25,
                          'vertical-fov-upper-deg':-15 if kind=='uav' else 40,
                          'horizontal-rotation-frequency':11.6639 if kind=='uav' else 10,
                          'vertical-rotation-frequency':170 if kind=='uav' else 0})
                if kind=='ugv':
                    s['origin']['xyz']='0 0 -1.8'
        write_json(cfg/f'robot_{kind}_plaza.jsonc',robot)
    scene = json.loads((cfg/'scene_mapping_dense.jsonc').read_text(encoding='utf-8'))
    scene['id']='AGMappingPlaza'
    for a in scene['actors']:
        a['robot-config']='robot_uav_plaza.jsonc' if a['name']=='Drone1' else 'robot_ugv_plaza.jsonc'
        if a['name']=='Car1':
            a['origin']['xyz']='25 50.5 2.8'
    write_json(cfg/'scene_mapping_plaza.jsonc',scene)
    c=json.loads((cfg/'mapping_dense.json').read_text(encoding='utf-8'))
    c.update(scene='scene_mapping_plaza.jsonc', duration_sec=230, full_lidar_capture=True,
             closed_loop=True, waypoint_tolerance_m=1.2, uav_speed_mps=2.5,
             scenario_status='Plaza central flat perimeter UGV loop; larger aerial loop over terraces and shore')
    c['dense_capture']['min_interval_sec']=.85
    c['routes']['Car1']=rounded_loop(21,48,50.5,63,4,4.48)
    # First climb vertically, then join and close the aerial loop.
    c['routes']['Drone1']=[[20,58,-24]]+rounded_loop(5,55,38,78,6,-24)
    write_json(cfg/'mapping_plaza.json',c)


if __name__=='__main__':
    generate()
