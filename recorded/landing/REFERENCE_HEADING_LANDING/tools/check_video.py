"""Audit this live landing rerun; source timestamps, physical motion, and video decode."""
import csv,gzip,json,hashlib,sys
from pathlib import Path
import cv2
import numpy as np

root=Path(__file__).resolve().parents[1]
take=root/sys.argv[1]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
 return h.hexdigest()
def readcsv(p):
 with p.open(newline='',encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def videoinfo(p):
 cap=cv2.VideoCapture(str(p))
 info={'fps':cap.get(cv2.CAP_PROP_FPS),'width':int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
       'height':int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),'declared_frames':int(cap.get(cv2.CAP_PROP_FRAME_COUNT))}
 n=0
 while True:
  ok,im=cap.read()
  if not ok:break
  n+=1
 cap.release()
 info.update(decoded_frames=n,sha256=sha(p),duration_sec=n/info['fps'])
 assert n==info['declared_frames'] and n>0,(p,info)
 return info
result=json.loads((take/'experiment_result.json').read_text(encoding='utf-8-sig'))
assert result['success'] and result['reason']=='verified_contact_and_motor_off_dwell',result
rows=readcsv(take/'experiment/trajectory.csv')
time=np.array([float(r['sim_time_sec']) for r in rows]); start,end=time[0],time[-1]
boat=np.array([[float(r['usv_'+a]) for a in 'xyz'] for r in rows])
axis=boat[-1,:2]-boat[0,:2]; axis/=np.linalg.norm(axis)
forward=np.diff(boat[:,:2],axis=0)@axis
last=rows[-1]
assert last['success']=='1' and last['support_valid']=='1' and last['rotors_stopped']=='1',last
report={'take':str(take),'result':result,'simulation_duration_sec':end-start,
 'boat':{'travel_distance_m':float(np.linalg.norm(np.diff(boat[:,:2],axis=0),axis=1).sum()),
 'minimum_progress_per_control_sample_m':float(forward.min()),
 'reverse_samples_over_1mm':int((forward < -.001).sum()),
 'max_step_m':float(np.linalg.norm(np.diff(boat[:,:2],axis=0),axis=1).max()),
 'final_speed_mps':float(last['boat_speed_mps'])},
 'landing':{'final_horizontal_error_m':float(last['horizontal_error_m']),
 'best_horizontal_error_m':min(float(r['horizontal_error_m']) for r in rows),
 'final_supported_dwell_sec':float(last['dwell_sec']),
 'final_rotor_total_thrust_n':float(last['rotor_total_thrust_n']),
 'final_rotor_max_speed_radps':float(last['rotor_max_speed_radps'])},
 'cameras':{},'no_frame_interpolation':True,
 'rates_definition':'Unique camera exposure timestamps per simulation second; not measured Unreal viewport/render FPS or physics FPS.',
 'protocol':json.loads((root/'protocol.json').read_text(encoding='utf-8-sig'))}
# Tie body-forward metrics to an effective rendered child transform.
# The old root visual yaw was overwritten by UE's root-pose update.
scene=json.loads((take/'scene_landing_video.jsonc').read_text(encoding='utf-8-sig'))
actor=next(a for a in scene['actors'] if a['name']=='Vessel1')
robot=json.loads((take/'robot_Vessel1_video.jsonc').read_text(encoding='utf-8-sig'))
links={link['name']:link for link in robot['links']}
assert actor['origin']['rpy-deg']=='0 0 -90'
assert 'visual' not in links['hull']
assert links['hull_visual']['visual']['origin']=={'xyz':'0 0 0','rpy-deg':'0 0 180'}
assert links['hull_visual']['collision']['enabled'] is False
assert any(j['parent-link']=='hull' and j['child-link']=='hull_visual' and j['type']=='fixed' for j in robot['joints'])
assert all(float(r['usv_velocity_y'])<0 for r in rows if float(r['boat_speed_mps'])>.05)
assert boat[-1,1]<boat[0,1]-1
resolved=json.loads((take/'experiment/resolved_scene.json').read_text(encoding='utf-8-sig'))
loaded_actor=next(a for a in resolved['actors'] if a['name']=='Vessel1')
loaded_robot=loaded_actor['robot-config']
if isinstance(loaded_robot,str):
 loaded_robot=json.loads(Path(loaded_robot).read_text(encoding='utf-8-sig'))
loaded_links={link['name']:link for link in loaded_robot['links']}
assert 'visual' not in loaded_links['hull']
assert loaded_links['hull_visual']['visual']['origin']=={'xyz':'0 0 0','rpy-deg':'0 0 180'}
report['rendered_hull_transform']={'geometry_free_root':True,'fixed_visual_child':'hull_visual','fixed_visual_yaw_deg':180,'body_yaw_deg':-90,'world_motion':'negative Y, away from shore','root_pose_override_avoided':True,'live_rgb_visual_check':'direction_visual_check.json'}

moving=[r for r in rows if float(r['boat_speed_mps'])>.05]
assert moving,'No moving vessel samples'
cosines=np.array([float(r['usv_bow_velocity_cosine']) for r in moving])
signed=np.array([float(r['usv_bow_forward_speed_mps']) for r in moving])
assert (signed>0).all() and cosines.min()>.999, 'Vessel motion does not follow visible bow'
report['bow_alignment']={'moving_samples':len(moving),'minimum_bow_velocity_cosine':float(cosines.min()),'max_bow_velocity_angle_deg':float(np.degrees(np.arccos(np.clip(cosines,-1,1))).max()),'negative_bow_velocity_samples':int((signed<0).sum()),'configuration':json.loads((root/'bow_alignment_revision.json').read_text(encoding='utf-8-sig'))}

manifest=json.loads((take/'camera_originals/recording_manifest.json').read_text(encoding='utf-8-sig'))
assert manifest['queue_drops']==0 and not manifest['errors'],manifest
for key,entry in manifest['cameras'].items():
 folder=take/'camera_originals'/key
 with gzip.open(folder/'capture_metadata.jsonl.gz','rt',encoding='utf-8') as f:
  meta=[json.loads(l) for l in f]
 times=np.array([m['source_time_ns']/1e9 for m in meta]); gaps=np.diff(times)
 assert (gaps>0).all()
 active=[m for m in meta if start+.15 <= m['source_time_ns']/1e9 <= end]
 at=np.array([m['source_time_ns']/1e9 for m in active]); ag=np.diff(at)
 idx=readcsv(folder/'video_frame_index.csv')
 activeidx=[r for r in idx if start+.15<=int(r['source_time_ns'])/1e9<=end]
 ci={'source_exposures_total':len(meta),'independent_exposure_rate_full_sim_hz':(len(times)-1)/(times[-1]-times[0]),
     'independent_exposure_rate_trial_sim_hz':(len(at)-1)/(at[-1]-at[0]),
     'median_trial_exposure_gap_sim_sec':float(np.median(ag)),
     'p95_trial_exposure_gap_sim_sec':float(np.percentile(ag,95)),
     'max_trial_exposure_gap_sim_sec':float(ag.max()),
     'held_output_frames_in_trial':sum(int(r['held_previous_exposure']) for r in activeidx),
     'output_frames_in_trial':len(activeidx),'decoded_video':videoinfo(folder/'original_rgb.mp4')}
 if key=='overview':
  pos=np.array([[m['capture']['pos_'+a] for a in 'xyz'] for m in active])
  quat=np.array([[m['capture']['rot_'+a] for a in 'wxyz'] for m in active])
  ci['camera_position_range_m']=np.ptp(pos,axis=0).tolist()
  ci['camera_quaternion_component_range']=np.ptp(quat,axis=0).tolist()
  assert np.ptp(pos,axis=0).max()<1e-4 and np.ptp(quat,axis=0).max()<1e-5
 report['cameras'][key]=ci
report['presentation']=videoinfo(take/'presentation.mp4')
if (root/'fixed_camera_overview_30fps.mp4').exists():
 report['clean_fixed_camera_overview']=videoinfo(root/'fixed_camera_overview_30fps.mp4')
original=Path(r'F:/EpicGamesEnv/ProjectAirSim_AG_Experiments/EXPERIMENT_VIDEOS_20261002')
report['previous_landing_camera_rates']={}
for key in ['overview','down']:
 p=original/'04_moving_usv_landing_take3/camera_originals'/key/'capture_metadata.jsonl.gz'
 with gzip.open(p,'rt',encoding='utf-8') as f:
  previous=np.array([json.loads(line)['source_time_ns']/1e9 for line in f])
 report['previous_landing_camera_rates'][key]={'metadata_path':str(p),'metadata_sha256':sha(p),'independent_exposure_rate_sim_hz':(len(previous)-1)/(previous[-1]-previous[0])}
expected=json.loads((Path(r'F:/EpicGamesEnv/ProjectAirSim_AG_Experiments/LANDING_VIDEO_RERUN_20261004')/'protected_original_videos.json').read_text(encoding='utf-8-sig'))
report['original_videos_unchanged']={name:sha(original/name)==expectedhash for name,expectedhash in expected.items()}
assert all(report['original_videos_unchanged'].values())
(root/'quality_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(report,indent=2,ensure_ascii=False))
