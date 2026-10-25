"""Isolated actual-simulator moving-USV landing and evidence collection."""
import asyncio
import json
import math
import time
import traceback
from pathlib import Path
import numpy as np
from .backend import Backend
from .landing_core import ContactGate, VisualTargetFilter, control_velocity, deck_position, deck_relative, rotor_measurement
from .landing_evidence import Evidence
from .landing_vision import DeckDetector
from .math3d import pose_parts, rotation, wrap
from .storage import write_json


class LandingBackend(Backend):
    def load_landing(self):
        from projectairsim import World, Drone
        self.world = World(self.client, self.config['scene'], sim_config_path=self.config['sim_config_path'], delay_after_load_sec=1)
        self.drone = Drone(self.client, self.world, self.config['uav'])
        self.client.socket_services.recv_timeout = 15000
        for name in (self.config['uav'], self.config['usv']):
            for leaf, key in [('actual_pose', 'pose'), ('collision_info', 'collision')]:
                self.subscribe(name+'/'+key, f'{self.world.parent_topic}/robots/{name}/{leaf}')
        self.subscribe('down', f"{self.world.parent_topic}/robots/{self.config['uav']}/sensors/{self.config['camera']}/scene_camera")
        self.subscribe('rotor', f"{self.world.parent_topic}/robots/{self.config['uav']}/rotor_info")
        if self.config.get('record_depth', False):
            self.subscribe('depth', f"{self.world.parent_topic}/robots/{self.config['uav']}/sensors/{self.config['camera']}/depth_planar_camera")
        for leaf, key in [('scene_camera','overview'), ('segmentation_camera','overview_seg')]:
            self.subscribe(key, f'{self.world.parent_topic}/robots/Observer/sensors/Overview/{leaf}')
        for name, sid in [(self.config['uav'],self.config['uav_segmentation_id']), (self.config['usv'],self.config['usv_segmentation_id'])]:
            if not self.world.set_segmentation_id_by_name(name, sid, False, True):
                raise RuntimeError('Cannot assign scene visibility identifiers')

    def state(self, name):
        raw = self.buffer.latest(name+'/pose', 5.)
        if raw is None:
            raise RuntimeError('No timestamped physics pose for '+name)
        p, r = pose_parts(raw)
        stamp = int(raw['time_stamp'])/1e9
        last = self.last_pose.get(name)
        v = (p-last[0])/(stamp-last[1]) if last and stamp-last[1] > .001 else np.zeros(3)
        self.last_pose[name] = p, stamp, v
        q = raw.get('rotation', raw.get('orientation'))
        return {'position':p, 'rotation':r, 'quaternion':q, 'velocity':v,
                'time':stamp, 'yaw':math.atan2(r[1,0],r[0,0])}

    def forces(self, force):
        response = self.request('SetThrusterForces', {'forces':[0., float(force), float(force)]}, self.config['usv'])
        if response is False:
            raise RuntimeError('Vessel propulsion service rejected command; no SetPose fallback')

    async def step(self, seconds):
        await asyncio.to_thread(self.world.continue_for_sim_time, int(seconds*1e9), True)
        await asyncio.sleep(.015)

    def observer(self, uav, target):
        from projectairsim.types import Pose
        center = .5*(np.asarray(uav)+np.asarray(target))
        distance = np.linalg.norm(np.asarray(uav)-target)
        pos = center+np.array([-9.-.3*distance, -12.-.2*distance, -8.-.2*distance])
        delta = center-pos
        yaw = math.atan2(delta[1],delta[0])
        pitch = -math.atan2(delta[2],np.linalg.norm(delta[:2]))
        cy,sy,cp,sp = math.cos(yaw/2),math.sin(yaw/2),math.cos(pitch/2),math.sin(pitch/2)
        pose = Pose({'translation':dict(zip('xyz',map(float,pos))),
                     'rotation':dict(zip('wxyz',[cy*cp,-sy*sp,cy*sp,sy*cp]))})
        if self.request('SetPose', {'pose':pose,'wait_for_pose_update':False}, 'Observer/sensors/Overview') is False:
            raise RuntimeError('Observer camera positioning rejected')

    async def release(self, motors_off):
        errors=[]
        if self.world is not None:
            try:
                self.forces(0.)
                if not motors_off:
                    self.world.resume()
                    task = await self.drone.hover_async()
                    await asyncio.wait_for(task, 8)
                self.world.pause()
            except Exception as exc:
                errors.append(str(exc))
        self.buffer.sink = None
        for topic in self.subscriptions:
            try:self.client.unsubscribe(topic)
            except Exception as exc:errors.append(str(exc))
        if self.client:
            try:self.client.disconnect()
            except Exception as exc:errors.append(str(exc))
        if self.lock_file:
            self.lock_file.close()
        return errors


def collision_evidence(message, now, config, deck):
    if not message:
        return False, None, ''
    stamp = int(message.get('time_stamp', 0))/1e9
    name = str(message.get('object_name',''))
    identity = config['usv'] in name or int(message.get('segmentation_id',-1)) == config['usv_segmentation_id']
    normal = message.get('normal', {})
    normal_z = float(normal.get('z', 0)) if isinstance(normal,dict) else float(normal[2])
    point = message.get('impact_point', {})
    if isinstance(point,dict):
        point = [point.get(k, float('nan')) for k in 'xyz']
    point = np.asarray(point,float)
    on_deck = np.isfinite(point).all() and np.linalg.norm(point[:2]-np.asarray(deck)[:2]) < 1.5 and abs(point[2]-deck[2]) < .75
    # Collision with a hull side, ground, or water is not deck support.
    valid_support = not message.get('support_managed', False) or message.get('support_valid', False)
    return bool(identity and on_deck and normal_z < -.5 and valid_support), stamp, name


async def run_trial(config, output, desired_speed, repeat, probe=False):
    c=dict(config)
    c['desired_speed_mps']=desired_speed
    c['record_depth']=bool(probe or c.get('record_depth', False))
    b=LandingBackend(c)
    evidence=Evidence(output,c,{'speed_mps':desired_speed,'repeat':repeat,'controller':c['controller'],'probe':probe})
    result={'status':'failed','success':False,'reason':'timeout','probe':probe}
    motors_off=False
    pending=[]
    try:
        b.connect()
        evidence.manifest['previous_scene_ids']=b.previous_scene_ids
        b.load_landing()
        b.buffer.sink=evidence.topic
        # Initialization only, before the measured trial. Match the existing
        # controller's water-surface initialization and record it explicitly.
        pose=b.world.get_object_pose(c['usv'])
        pose['translation']['z']=8.8
        if b.request('SetPose',{'pose':pose,'reset_kinematics':True},c['usv']) is False:
            raise RuntimeError('Initial vessel surface placement rejected')
        b.forces(0.)
        # Save static vessel/tag geometry and inherited configuration before motion.
        initial={n:b.world.get_object_pose(n) for n in (c['uav'],c['usv'])}
        write_json(evidence.path/'initial_state.json',initial)
        write_json(evidence.path/'resolved_scene.json',b.world.get_configuration())
        b.drone.enable_api_control()
        if b.drone.arm() is False:
            raise RuntimeError('Arming rejected')
        takeoff=await b.drone.takeoff_async(timeout_sec=40)
        takeoff_started=time.monotonic()
        while not takeoff.done():
            if time.monotonic()-takeoff_started>90:
                raise TimeoutError('Takeoff timed out')
            b.forces(0.)
            await b.step(.1)
        if takeoff.result() is False:
            raise RuntimeError('Takeoff command failed')
        b.world.pause()
        detector=DeckDetector(c)
        filt=VisualTargetFilter(c['velocity_alpha'])
        gate=ContactGate(c,desired_speed > 0)
        start=b.now(); wall_start=time.monotonic()
        phase='approach'; acquired=False; last_image=-1; last_detection=None
        obstructed_since=None
        last_overview=-1.; last_progress=-1.; last_deck=None
        frame=None; detection=None
        confirmed=0; best_horizontal=float('inf'); count=0
        while b.now()-start < c['timeout_sec']:
            now=b.now()
            if time.monotonic()-wall_start > c['wall_timeout_sec']:
                result['reason']='wall_timeout';break
            sample_start=now
            u=b.state(c['uav']);v=b.state(c['usv'])
            if not np.isfinite(u['position']).all() or not np.isfinite(v['position']).all():
                raise RuntimeError('Simulator returned nonfinite actor state; trial is invalid')
            deck=deck_position(v['position'],v['quaternion'],c['tag_offset_hull_m'])
            rel=deck_relative(u['position'],v['position'],v['quaternion'],c['tag_offset_hull_m'])
            deck_velocity=(deck-last_deck[0])/(now-last_deck[1]) if last_deck and now>last_deck[1] else np.zeros(3)
            last_deck=(deck,now)
            boat_speed=float(np.linalg.norm(v['velocity'][:2]))
            relspeed=float(np.linalg.norm(u['velocity']-deck_velocity))
            horizontal=float(np.linalg.norm(rel[:2]));height=float(-rel[2])
            best_horizontal=min(best_horizontal,horizontal)
            if any(not c['trial_bounds_ned'][axis][0] <= u['position'][j] <= c['trial_bounds_ned'][axis][1]
                   or not c['trial_bounds_ned'][axis][0] <= v['position'][j] <= c['trial_bounds_ned'][axis][1]
                   for j,axis in enumerate('xyz')):
                result['reason']='outside_trial_bounds';break
            message=b.buffer.latest('down',10.)
            new_frame=message is not None and int(message['time_stamp']) > last_image
            if new_frame:
                from projectairsim.utils import unpack_image
                frame=unpack_image(message)
                detection=detector.detect(message,frame)
                last_image=int(message['time_stamp'])
                fresh=0 <= now-last_image/1e9 <= c['image_max_age_sec']
                if detection['visible'] and fresh:
                    accepted=filt.update(last_image/1e9,detection['selected']['world_position_ned'])
                    if accepted:
                        confirmed+=1;last_detection=now
                    else:
                        confirmed=0
                else:
                    confirmed=0
                if confirmed >= 3:
                    acquired=True
            contact=b.buffer.latest(c['uav']+'/collision',10.)
            vessel_contact,collision_time,collision_name=collision_evidence(contact,now,c,deck)
            contact_fresh=vessel_contact and collision_time is not None and 0<=now-collision_time<=c['collision_max_age_sec']
            unexpected_contact=(contact is not None and collision_time is not None and 0<=now-collision_time<.3
                 and collision_name==c['usv'] and height>1. and np.linalg.norm(u['velocity'])<.08)
            if unexpected_contact:
                obstructed_since=now if obstructed_since is None else obstructed_since
            else:obstructed_since=None
            if acquired and contact_fresh and horizontal<c['contact_horizontal_radius_m'] and c['contact_height_range_m'][0]<=height<=c['contact_height_range_m'][1] and not motors_off:
                response=b.drone.disarm()
                motors_off=response is not False
                evidence.row('events',{'sim_time_sec':now,'event':'motor_cut_after_deck_contact','detail':json.dumps({'accepted':motors_off,'object':collision_name})})
                phase='contact_hold'
            continuous_support = None
            if c.get('require_continuous_support', False):
                continuous_support = bool(contact and contact.get('support_managed') and
                    contact.get('support_valid') and contact_fresh)
            rotors=rotor_measurement(b.buffer.latest('rotor',5.),now,c)
            measured_stop=rotors['stopped'] if c.get('require_rotor_verification',False) else None
            success,dwell=gate.update(now,horizontal,height,relspeed,boat_speed,collision_time,vessel_contact,motors_off,continuous_support,measured_stop)
            velocity=np.zeros(3)
            if motors_off:
                phase='contact_hold'
            elif acquired and filt.time is not None:
                loss_age=now-filt.time
                if loss_age <= c['image_max_age_sec']:
                    velocity,phase=control_velocity(c,u,filt,now,c['controller'])
                elif loss_age <= .65 and filt.predict(now)[2]-u['position'][2] < .9:
                    # Bounded short prediction when the tag fills the camera at contact.
                    velocity,phase=control_velocity(c,u,filt,now,c['controller'])
                    velocity[2]=min(velocity[2],.08);phase='flare_prediction'
                else:
                    phase='target_lost_hold'
                    velocity[:2]=filt.velocity[:2] if loss_age<1. else 0.
                    if loss_age>c['lost_target_abort_sec']:
                        result['reason']='visual_target_lost';break
            else:
                delta=deck-u['position'];delta[2]-=c['approach_altitude_m']
                velocity=.7*delta
                norm=np.linalg.norm(velocity[:2])
                if norm>c['max_speed_mps']:velocity[:2]*=c['max_speed_mps']/norm
                velocity[2]=np.clip(velocity[2],-1.,1.)
                phase='approach'
            if probe and acquired:
                # Calibration probe holds altitude and does not attempt a landing.
                delta=deck-u['position'];delta[2]-=c['approach_altitude_m']
                velocity=np.clip(.7*delta,-.7,.7);phase='vision_probe_truth_hold'
            force=float(np.clip((.5*(70.*desired_speed+135.*desired_speed*abs(desired_speed)))+c['vessel_speed_kp']*(desired_speed-boat_speed),0.,c['vessel_max_force'])) if desired_speed>0 else 0.
            b.forces(force)
            if not motors_off:
                task=await b.drone.move_by_velocity_async(*map(float,velocity), duration=max(.3,3*c['dt_sec']),yaw_is_rate=True,yaw=0.)
                pending.append(task)
                remaining=[]
                for item in pending:
                    if item.done():item.result()
                    else:remaining.append(item)
                pending=remaining
            evidence.row('commands',{'sim_time_sec':now,'phase':phase,'vx':velocity[0],'vy':velocity[1],'vz':velocity[2],
              'vessel_port_force':force,'vessel_starboard_force':force,'motors_off':int(motors_off)})
            row={'sim_time_sec':now,'elapsed_sec':now-start,'sample_end_sec':b.now(),'phase':phase,
                 'uav_time_sec':u['time'],'usv_time_sec':v['time'],'camera_time_ns':last_image,
                 'camera_age_sec':now-last_image/1e9 if last_image>=0 else None,
                 'tag_visible':int(bool(detection and detection['visible'])),
                 'visual_measurement_time_sec':filt.time,'desired_speed_mps':desired_speed,'boat_speed_mps':boat_speed,
                 'horizontal_error_m':horizontal,'height_over_deck_m':height,'relative_speed_mps':relspeed,
                 'relative_yaw_deg':math.degrees(wrap(u['yaw']-v['yaw'])),
                 'vessel_contact':int(vessel_contact),'contact_fresh':int(contact_fresh),'collision_time_sec':collision_time,
                 'collision_object':collision_name,'motors_off':int(motors_off),'dwell_sec':dwell,'success':int(success)}
            row.update(support_managed=int(bool(contact and contact.get('support_managed'))),
                       support_valid=int(bool(contact and contact.get('support_valid'))),
                       continuous_support='' if continuous_support is None else int(continuous_support),
                       contact_component=contact.get('component_name','') if contact else '',
                       support_gap_m=contact.get('support_gap') if contact else None)
            row.update(rotor_time_ns=rotors['time_ns'],rotor_max_speed_radps=rotors['max_speed_radps'],
                       rotor_total_thrust_n=rotors['total_thrust_n'],rotors_stopped=int(rotors['stopped']))
            for prefix,vector in [('uav',u['position']),('usv',v['position']),('deck',deck),
                                  ('visual_deck',filt.position if filt.position is not None else [None]*3),
                                  ('relative_deck',rel),('uav_velocity',u['velocity']),('deck_velocity',deck_velocity)]:
                row.update({prefix+'_'+axis:value for axis,value in zip('xyz',vector)})
            bow=rotation(v['quaternion'])@np.asarray([1.,0.,0.])
            signed_speed=float(np.dot(v['velocity'][:2],bow[:2]))
            row.update(usv_yaw_rad=v['yaw'],usv_visual_bow_x=bow[0],usv_visual_bow_y=bow[1],usv_velocity_x=v['velocity'][0],usv_velocity_y=v['velocity'][1],usv_bow_forward_speed_mps=signed_speed,usv_bow_velocity_cosine=signed_speed/boat_speed if boat_speed>.01 else '')
            evidence.row('trajectory',row)
            if new_frame:evidence.frame('down',message,frame,detection,phase)
            if probe and new_frame:
                entries=b.buffer.entries('depth')
                if entries:
                    depth=min((m for _,m in entries),key=lambda m:abs(int(m['time_stamp'])-last_image))
                    if abs(int(depth['time_stamp'])-last_image)<=20_000_000:
                        from projectairsim.utils import unpack_image
                        evidence.frame('depth',depth,unpack_image(depth),phase=phase)
            if now-last_overview >= c['observer_interval_sec']:
                b.observer(u['position'],deck)
                ov=b.buffer.latest('overview',10.)
                if ov:
                    from projectairsim.utils import unpack_image
                    evidence.frame('overview',ov,unpack_image(ov),phase=phase)
                    segs=b.buffer.entries('overview_seg')
                    if segs:
                        seg=min((m for _,m in segs),key=lambda m:abs(int(m['time_stamp'])-int(ov['time_stamp'])))
                        if abs(int(seg['time_stamp'])-int(ov['time_stamp']))<=20_000_000:
                            seg_image=unpack_image(seg)
                            evidence.frame('overview_seg',seg,seg_image,phase=phase)
                            from projectairsim.image_utils import segmentation_id_to_color
                            boxes={}
                            for name,sid in [(c['uav'],c['uav_segmentation_id']),(c['usv'],c['usv_segmentation_id'])]:
                                color=segmentation_id_to_color(sid)
                                yy,xx=np.nonzero(np.all(seg_image[:,:,:3]==[color.b,color.g,color.r],axis=2))
                                if len(xx):boxes[name]={'pixels':int(len(xx)),'bbox':[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())]}
                            evidence.row('scene_visibility',{'time_ns':int(ov['time_stamp']),'robots':json.dumps(boxes)})
                last_overview=now
            count+=1
            if now-last_progress>=5:
                print(f"t={now-start:.1f} phase={phase} tag={row['tag_visible']} xy={horizontal:.3f} h={height:.3f} boat={boat_speed:.3f} contact={int(contact_fresh)}",flush=True)
                last_progress=now
            if success:
                result.update(status='completed',success=True,reason='verified_contact_and_motor_off_dwell');break
            if obstructed_since is not None and now-obstructed_since>2.:
                result['reason']='vessel_collision_above_deck';break
            if probe and acquired and now-start>22:
                result.update(status='probe_completed',success=False,reason='visual_acquisition_probe_only');break
            if height < -.3 and acquired:
                result['reason']='passed_below_deck_without_verified_support';break
            if motors_off and gate.last_contact is not None and now-gate.last_contact>c['hold_sec']+3:
                result['reason']='deck_contact_not_sustained';break
            await b.step(c['dt_sec'])
        result.update(samples=count,best_horizontal_error_m=best_horizontal,tag_acquired=acquired,
                      motors_off=motors_off,elapsed_sec=b.now()-start)
    except Exception as exc:
        result.update(status='error',success=False,reason=str(exc))
        (evidence.path/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        print(traceback.format_exc(),flush=True)
    finally:
        for task in pending:
            if not task.done():task.cancel()
        if pending:
            try:await asyncio.wait_for(asyncio.gather(*pending,return_exceptions=True),3.)
            except asyncio.TimeoutError:pass
        result['cleanup_errors']=await b.release(motors_off)
        evidence.close(result)
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return result
