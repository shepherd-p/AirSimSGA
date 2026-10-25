"""Summarize measured motor-off motion and support; never infer success from plots."""
import argparse
import csv
import gzip
import json
from pathlib import Path
import numpy as np
from ag.storage import write_json

def diagnose(path):
    rows=list(csv.DictReader((path/'trajectory.csv').open(encoding='utf-8')))
    hold=[r for r in rows if r['motors_off']=='1']
    out={'samples':len(rows),'motor_off_samples':len(hold),
         'recorded_result':json.loads((path/'manifest.json').read_text(encoding='utf-8')).get('success',False)}
    out['motor_off_samples_definition']='Samples after accepted Disarm; actual stopped samples reported separately when recorded'
    stopped=[r for r in hold if r.get('rotors_stopped')=='1']
    out['measured_stopped_rotor_samples']=len(stopped)
    if stopped:
        out['stopped_rotor_max_speed_radps']=max(float(r['rotor_max_speed_radps']) for r in stopped)
        out['stopped_rotor_max_total_thrust_n']=max(float(r['rotor_total_thrust_n']) for r in stopped)
        out['measured_stopped_duration_sec']=float(stopped[-1]['sim_time_sec'])-float(stopped[0]['sim_time_sec'])
    if len(hold)>1:
        pos=lambda prefix:np.array([[float(r[prefix+'_'+a]) for a in 'xyz'] for r in hold])
        t=np.array([float(r['sim_time_sec']) for r in hold]);u=pos('uav');v=pos('usv')
        out.update(motor_off_duration_sec=float(t[-1]-t[0]),
            uav_world_displacement_m=float(np.linalg.norm(u[-1]-u[0])),
            usv_world_displacement_m=float(np.linalg.norm(v[-1]-v[0])),
            contact_horizontal_error_m=float(hold[0]['horizontal_error_m']),
            max_hold_horizontal_error_m=max(float(r['horizontal_error_m']) for r in hold),
            max_hold_relative_speed_mps=max(float(r['relative_speed_mps']) for r in hold),
            min_hold_boat_speed_mps=min(float(r['boat_speed_mps']) for r in hold),
            continuous_support_fraction=sum(r.get('continuous_support')=='1' for r in hold)/len(hold),
            contact_components=sorted({r.get('contact_component','') for r in hold}))
    contacts=[]
    with gzip.open(path/'raw_topics.jsonl.gz','rt',encoding='utf-8') as f:
        for line in f:
            item=json.loads(line)
            if item['topic'].endswith('/collision'):
                m=item['message']
                contacts.append((m.get('object_name'),m.get('component_name'),m.get('support_valid')))
    out['raw_contact_messages']=len(contacts)
    out['raw_contact_components']=sorted({str(a)+'/'+str(b) for a,b,_ in contacts})
    write_json(path/'derived/contact_diagnosis.json',out)
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('path',type=Path);a=p.parse_args()
    trials=[a.path] if (a.path/'manifest.json').exists() else sorted(a.path.glob('trial_*'))
    for trial in trials:print(json.dumps({'trial':str(trial),**diagnose(trial)},indent=2))
