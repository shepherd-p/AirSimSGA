"""Photograph the real CityPark scene with physically flying UAV and parked UGV.

Loads independent configs. Only the invisible observer camera is positioned by
pose; Drone1 takes off under its existing controller. This is a separately
captured scene illustration, not a frame from a previous mapping experiment.
"""
import argparse
import asyncio
import json
import math
import os
import time
from datetime import datetime
from pathlib import Path
import numpy as np
from ag.backend import Backend
from ag.storage import ROOT,write_json
from ag.scene_figure import prepare_scene_figure


def configs():
    cfg=ROOT/'configs'
    robot={'physics-type':'non-physics','links':[{'name':'Frame','collision':{'enabled':False}}],
           'sensors':[{'id':'Overview','type':'camera','enabled':True,'parent-link':'Frame',
                       'capture-interval':.5,'origin':{'xyz':'0 0 0','rpy-deg':'0 0 0'},
                       'capture-settings':[{'image-type':i,'width':2560,'height':1440,'fov-degrees':80,
                         'capture-enabled':True,'streaming-enabled':True,'pixels-as-float':False,'compress':False}
                         for i in (0,3)]}]}
    write_json(cfg/'robot_overview_camera.jsonc',robot)
    scene=json.loads((cfg/'scene_mapping_plaza.jsonc').read_text(encoding='utf-8'))
    scene['id']='AGPlazaOverview'
    scene['actors'].append({'name':'Observer','type':'robot','robot-config':'robot_overview_camera.jsonc',
                            'origin':{'xyz':'0 0 0','rpy-deg':'0 0 0'}})
    write_json(cfg/'scene_plaza_overview.jsonc',scene)
    c=json.loads((cfg/'mapping_plaza.json').read_text(encoding='utf-8'))
    c['scene']='scene_plaza_overview.jsonc'
    return c


async def capture(out,views):
    import cv2
    c=configs();b=Backend(c)
    try:
        b.connect()
        from projectairsim import World,Drone,Rover
        from projectairsim.types import Pose
        from projectairsim.utils import unpack_image
        from projectairsim.image_utils import segmentation_id_to_color
        b.world=World(b.client,c['scene'],sim_config_path=str(ROOT/'configs'),delay_after_load_sec=1)
        for name,kind,cls in [('Drone1','uav',Drone),('Car1','ugv',Rover)]:
            b.kinds[name]=kind;b.robots[name]=cls(b.client,b.world,name)
            for leaf,key in [('actual_pose','pose'),('collision_info','collision')]:
                b.subscribe(name+'/'+key,f'{b.world.parent_topic}/robots/{name}/{leaf}')
        camera=f'{b.world.parent_topic}/robots/Observer/sensors/Overview'
        b.subscribe('overview_rgb',camera+'/scene_camera');b.subscribe('overview_seg',camera+'/segmentation_camera')
        b.world.resume();await b.ready(25);await b.settle()
        for n,sid in [('Drone1',240),('Car1',241)]:
            if not b.world.set_segmentation_id_by_name(n,sid,False,True):raise RuntimeError('Robot segmentation assignment failed')
        await b.start()
        # A closer hover makes the real drone identifiable in the scene figure.
        # This photograph is independent of the recorded mapping mission.
        for goal in ([20,58,-15],[22,49,-15],[22,49,-8]):
            deadline=time.monotonic()+50
            while time.monotonic()<deadline:
                pose=b.pose('Drone1');delta=np.array(goal)-pose['position']
                if np.linalg.norm(delta)<.4:break
                velocity=.8*delta;speed=np.linalg.norm(velocity)
                if speed>2.5:velocity*=2.5/speed
                await b.drone_velocity('Drone1',velocity);await asyncio.sleep(.1)
            else:raise TimeoutError('Drone did not reach overview hover position')
        await b.settle_task(b.robots['Drone1'].hover_async(),8)
        await b.rover('Car1')
        captures=[]
        for index,(position,target) in enumerate(views):
            delta=np.asarray(target)-position
            yaw=math.atan2(delta[1],delta[0]);pitch=-math.atan2(delta[2],np.linalg.norm(delta[:2]))
            cy,sy=np.cos(yaw/2),np.sin(yaw/2);cp,sp=np.cos(pitch/2),np.sin(pitch/2)
            pose=Pose({'translation':dict(zip('xyz',position)),'rotation':dict(zip('wxyz',map(float,[cy*cp,-sy*sp,cy*sp,sy*cp])))})
            if not b.request('SetPose',{'pose':pose,'wait_for_pose_update':True},name='Observer/sensors/Overview'):
                raise RuntimeError('Overview camera pose update rejected')
            # Invisible observer has identity world pose, so relative camera pose
            # equals the requested world pose. The two vehicles are not teleported.
            await asyncio.sleep(2)
            until=time.monotonic()+15;pair=None
            while time.monotonic()<until:
                pair=b.buffer.pair('overview_rgb','overview_seg',2.,.02)
                if pair:break
                await asyncio.sleep(.2)
            if pair is None:raise RuntimeError('No synchronized overview camera pair')
            rgb,seg=map(unpack_image,pair)
            boxes={}
            for n,sid in [('Drone1',240),('Car1',241)]:
                color=segmentation_id_to_color(sid)
                yy,xx=np.nonzero(np.all(seg[:,:,:3]==[color.b,color.g,color.r],axis=2))
                if len(xx):boxes[n]={'pixels':len(xx),'bbox':[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())]}
            path=out/f'scene_view_{index+1}.png';cv2.imwrite(str(path),rgb)
            cv2.imwrite(str(out/f'segmentation_view_{index+1}.png'),seg)
            captures.append({'image':path.name,'requested_camera_position_ned':position,'target_ned':target,
               'capture_metadata':{k:v for k,v in pair[0].items() if k!='data'},'visible_robots':boxes,
               'robot_poses':{n:b.world.get_object_pose(n) for n in b.kinds}})
            print(path,boxes,flush=True)
        write_json(out/'scene_capture.json',{'provenance':'Live Unreal camera; separate illustration session, no robot compositing or pose teleport',
                     'config':c,'captures':captures})
        prepare_scene_figure(out)
        print(f'Selected scene figure: {out / "scene_overview.png"}',flush=True)
    finally:
        errors=await b.cleanup();write_json(out/'cleanup.json',{'errors':errors})


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path)
    p.add_argument('--prepare-existing',type=Path,help='Only select and label an existing photo session; no simulator connection')
    a=p.parse_args();os.chdir(ROOT)
    if a.prepare_existing:
        print(json.dumps(prepare_scene_figure(a.prepare_existing),indent=2))
        raise SystemExit(0)
    out=a.output_dir or ROOT/'outputs/scene_overview'/datetime.now().strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=True)
    views=[([2,22,-24],[26,56,0]),([7,30,-17],[26,55,0]),([12,35,-12],[26,54,0])]
    asyncio.run(capture(out,views))
