"""Record candidate task regions in the current UE level, without moving robots.

Loads an independent camera-only ProjectAirSim actor configuration. Survey
imagery and surface-height queries are for scenario design, not navigation input.
"""
import argparse
import asyncio
import csv
import json
import math
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
from ag.backend import Backend
from ag.storage import ROOT, write_json


async def survey(output,detail=False):
    c=json.loads((ROOT/'configs/mapping_plaza.json').read_text(encoding='utf-8'))
    robot={'physics-type':'non-physics','links':[{'name':'Frame','collision':{'enabled':False}}],
       'sensors':[{'id':'Survey','type':'camera','enabled':True,'parent-link':'Frame','capture-interval':.5,
         'origin':{'xyz':'0 60 -140','rpy-deg':'0 -90 0'},
         'capture-settings':[{'image-type':i,'width':1920,'height':1440,'fov-degrees':90,
             'capture-enabled':True,'streaming-enabled':True,'pixels-as-float':False,'compress':False} for i in (0,1)]}]}
    scene=json.loads((ROOT/'configs/scene_mapping_plaza.jsonc').read_text(encoding='utf-8'))
    scene['id']='AGTaskRegionSurvey';scene['actors']=[{'name':'Observer','type':'robot','robot-config':'robot_region_survey.jsonc',
               'origin':{'xyz':'0 0 0','rpy-deg':'0 0 0'}}]
    write_json(ROOT/'configs/robot_region_survey.jsonc',robot)
    write_json(ROOT/'configs/scene_region_survey.jsonc',scene)
    output.mkdir(parents=True,exist_ok=False)
    b=Backend(c);records=[]
    try:
        b.connect()
        from projectairsim import World
        from projectairsim.types import Pose
        from projectairsim.utils import unpack_image
        b.world=World(b.client,'scene_region_survey.jsonc',sim_config_path=str(ROOT/'configs'),delay_after_load_sec=1)
        for key,leaf in [('rgb','scene_camera'),('depth','depth_planar_camera')]:
            b.subscribe(key,f'{b.world.parent_topic}/robots/Observer/sensors/Survey/{leaf}')
        b.world.resume()
        views=[('overview',[0,60,-140]),('west',[-85,60,-85]),('east',[110,60,-85]),
               ('south',[0,-80,-85]),('north',[0,185,-85])]
        if detail:
            views=[('navigation_road',[-35,-10,-40]),('tracking_bridge',[-35,110,-40]),('east_garden',[35,150,-40])]
        for label,position in views:
            pose=Pose({'translation':dict(zip('xyz',position)),
                       'rotation':dict(zip('wxyz',[math.sqrt(.5),0,-math.sqrt(.5),0]))})
            b.request('SetPose',{'pose':pose,'wait_for_pose_update':True},name='Observer/sensors/Survey')
            await asyncio.sleep(2.)
            pair=b.buffer.pair('rgb','depth',3.,.02)
            if pair is None:raise RuntimeError('No synchronized survey RGB/depth pair')
            metadata={}
            for key,msg in zip(['rgb','depth'],pair):
                cv2.imwrite(str(output/f'{label}_{key}.png'),unpack_image(msg))
                meta={k:v for k,v in msg.items() if k!='data'}
                np.savez(output/f'{label}_{key}.npz',payload=np.frombuffer(bytes(msg['data']),dtype='u1'),metadata_json=np.array(json.dumps(meta)))
                metadata[key]=meta
            records.append({'label':label,'position_ned':position,'camera':metadata})
            print(f'Surveyed {label}: {output/(label+"_rgb.png")}',flush=True)
        write_json(output/'survey.json',{'purpose':'Task area selection; same UE level, camera-only configuration',
              'views':records,'not_for_navigation_planner':True})
    finally:
        if b.world:
            try:b.world.pause()
            except Exception:pass
        b.close()
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path)
    p.add_argument('--detail',action='store_true')
    a=p.parse_args();out=a.output or ROOT/'outputs/region_survey'/datetime.now().strftime('%Y%m%d_%H%M%S')
    print(asyncio.run(survey(out,a.detail)))
