"""Short live forward-motion test, recorded against a fixed camera and shore."""
import csv,json,time
from pathlib import Path
import cv2
from record_water_landing import LAB,scene_copy,unpack_image,Recording,record_callback
from ag.backend import Backend
from urban_scene.probe_urban_scene import camera_pose
from projectairsim import World
from projectairsim.types import Pose
out=Path(__file__).resolve().parents[1]/'results/reference_probe_fixed_02'
out.mkdir(exist_ok=False)
scene=scene_copy('landing',out)
for p in out.glob('robot_*_video.jsonc'):
    robot=json.loads(p.read_text())
    for sensor in robot.get('sensors',[]):
        if sensor.get('type')=='camera':
            sensor['enabled']=('Observer' in p.name)
            sensor['capture-interval']=.025
            sensor['capture-settings']=[x for x in sensor['capture-settings'] if x['image-type']==0]
    p.write_text(json.dumps(robot))
c=json.loads((LAB/'configs/landing/study.json').read_text())
c.update(port_topics=9189,port_services=9190)
b=Backend(c)
recording=Recording(out/'camera_originals','landing')
record_callback(recording)
b.connect()
try:
    b.world=World(b.client,str(scene),sim_config_path=str(out),delay_after_load_sec=1)
    b.client.socket_services.recv_timeout=180000
    b.subscribe('overview',b.world.parent_topic+'/robots/Observer/sensors/Overview/scene_camera')
    b.request('SetPose',{'pose':camera_pose([0.,-12.,-22.],[29.,15.,4.],Pose),'wait_for_pose_update':False},'Observer/sensors/Overview')
    b.request('SetWaterPresentationMode',{'mode':32})
    def step(seconds):
        b.world.continue_for_sim_time(int(seconds*1e9),True)
        time.sleep(.15)
    step(.5)
    rows=[];shots=[]
    b.request('SetThrusterForces',{'forces':[0,80,80]},'Vessel1')
    for i in range(49):
        if i:step(.25)
        state=b.request('GetVesselDynamics',{},'Vessel1')
        rows.append([i*.25,b.now()]+state)
        if i in [0,16,32,48]:
            msg=b.buffer.latest('overview',10.)
            frame=unpack_image(msg)[:,:,:3]
            cv2.imwrite(str(out/f'live_t{i*.25:04.1f}.jpg'),frame)
            shots.append(frame)
            print(f'PROBE {i*.25:.1f}s x={state[9]:.3f} y={state[10]:.3f} yaw={state[11]:.6f}',flush=True)
    with (out/'live_dynamics.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['elapsed_sec','sim_time','m11','m22','m33','du','duu','dv','dvv','dr','drr','x','y','yaw','u','v','r','f0','f1','f2']);w.writerows(rows)
    import numpy as np
    sheet=np.zeros((720,1280,3),dtype='uint8')
    for n,im in enumerate(shots):
        small=cv2.resize(im,(640,360));cv2.putText(small,f'Live fixed camera: {n*4} s',(15,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
        sheet[n//2*360:(n//2+1)*360,n%2*640:(n%2+1)*640]=small
    cv2.imwrite(str(out/'continuous_motion_contact_sheet.jpg'),sheet)
    print('REFERENCE_PROBE_DONE',flush=True)
finally:
    try:b.request('SetThrusterForces',{'forces':[0,0,0]},'Vessel1')
    except Exception:pass
    b.client.disconnect()
    if b.lock_file:b.lock_file.close()
    recording.close()
