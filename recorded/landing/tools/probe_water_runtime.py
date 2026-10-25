"""Same-pose live water rendering comparison and live thrust/coast probe."""
import asyncio, csv, json, time, importlib.util, sys
from pathlib import Path
import cv2
from record_water_landing import LAB, scene_copy, unpack_image
from ag.backend import Backend
from urban_scene.probe_urban_scene import camera_pose
from projectairsim import World
from projectairsim.types import Pose

out=LAB/'results/water_probe';out.mkdir(exist_ok=False)
scene=scene_copy('landing',out)
c=json.loads((LAB/'configs/landing/study.json').read_text())
c.update(port_topics=9189,port_services=9190)
b=Backend(c)
b.connect()
try:
    b.world=World(b.client,str(scene),sim_config_path=str(out),delay_after_load_sec=1)
    b.client.socket_services.recv_timeout=180000
    topic=b.world.parent_topic+'/robots/Observer/sensors/Overview/scene_camera'
    b.subscribe('overview',topic)
    pose=camera_pose([0.,-12.,-22.],[29.,15.,4.],Pose)
    b.request('SetPose',{'pose':pose,'wait_for_pose_update':False},'Observer/sensors/Overview')
    def step(s):
        b.world.continue_for_sim_time(int(s*1e9),True)
        time.sleep(.2)
    step(.5)
    records=[]
    for mode in [0,1,2,4,8,16,32]:
        b.request('SetWaterPresentationMode',{'mode':mode})
        step(.5)
        msg=b.buffer.latest('overview',10.)
        if msg is None:raise RuntimeError('No live RGB')
        cv2.imwrite(str(out/f'water_mode_{mode}.png'),unpack_image(msg)[:,:,:3])
        records.append({'mode':mode,'capture':{k:v for k,v in msg.items() if k!='data'},'sim_time':b.now(),'vessel_dynamics':b.request('GetVesselDynamics',{},'Vessel1')})
    (out/'render_comparison.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    b.request('SetWaterPresentationMode',{'mode':32})
    rows=[]
    for phase,seconds,forces in [('powered',12,[0,34.375,34.375]),('coast',8,[0,0,0])]:
        b.request('SetThrusterForces',{'forces':forces},'Vessel1')
        for i in range(int(seconds/.25)):
            step(.25)
            values=b.request('GetVesselDynamics',{},'Vessel1')
            rows.append([phase,b.now()]+values)
            if i%8==0:
                msg=b.buffer.latest('overview',10)
                cv2.imwrite(str(out/f'{phase}_{i:03d}.png'),unpack_image(msg)[:,:,:3])
    with (out/'live_dynamics.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['phase','sim_time','m11','m22','m33','du','duu','dv','dvv','dr','drr','x','y','yaw','u','v','r','f0','f1','f2']);w.writerows(rows)
    print('WATER_PROBE_DONE',flush=True)
finally:
    try:b.request('SetThrusterForces',{'forces':[0,0,0]},'Vessel1')
    except Exception:pass
    b.client.disconnect()
    if b.lock_file:b.lock_file.close()
