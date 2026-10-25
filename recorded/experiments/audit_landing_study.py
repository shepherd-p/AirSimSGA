"""Independently recheck integrity, camera payloads, and touchdown criteria."""
import argparse
import csv
import gzip
import json
import sys
from pathlib import Path
import cv2
import numpy as np
from ag.landing_core import ContactGate, rotor_measurement
from ag.landing_evidence import digest
from ag.landing_vision import DeckDetector
from ag.storage import write_json


def audit_trial(path,replay_count=20):
    manifest=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
    c=json.loads((path/'config.json').read_text(encoding='utf-8'))
    inventory=json.loads((path/'evidence_sha256.json').read_text(encoding='utf-8'))
    bad=[name for name,h in inventory.items() if not (path/name).is_file() or digest(path/name)!=h]
    receipt=json.loads((path/'receipt.json').read_text(encoding='utf-8'))
    if receipt['manifest_sha256']!=digest(path/'evidence_sha256.json'):bad.append('evidence_sha256.json')
    rows=list(csv.DictReader((path/'trajectory.csv').open(encoding='utf-8'))) if (path/'trajectory.csv').exists() else []
    gate=ContactGate(c,manifest['trial']['speed_mps']>0)
    recomputed=False;success_rows=[]
    rotor_required=bool(c.get('require_rotor_verification'))
    rotor_checks=[]
    rotor_messages={}
    if rotor_required:
        with gzip.open(path/'raw_topics.jsonl.gz','rt',encoding='utf-8') as stream:
            for line in stream:
                record=json.loads(line)
                if record['topic']=='rotor':
                    m=record['message'];rotor_messages[int(m['time_stamp'])]=m
    for row in rows:
        measured_stop=None
        if rotor_required:
            stamp=int(row['rotor_time_ns']) if row.get('rotor_time_ns') else None
            measured=rotor_measurement(rotor_messages.get(stamp),float(row['sim_time_sec']),c)
            measured_stop=measured['stopped']
            same_values=all((measured[k] is None and not row.get(field)) or
                (measured[k] is not None and bool(row.get(field)) and np.isclose(measured[k],float(row[field]),rtol=0,atol=1e-6))
                for k,field in [('max_speed_radps','rotor_max_speed_radps'),('total_thrust_n','rotor_total_thrust_n')])
            rotor_checks.append(same_values and measured_stop==(row.get('rotors_stopped')=='1'))
        success,_=gate.update(float(row['sim_time_sec']),float(row['horizontal_error_m']),
            float(row['height_over_deck_m']),float(row['relative_speed_mps']),float(row['boat_speed_mps']),
            float(row['collision_time_sec']) if row['collision_time_sec'] else None,
            row['vessel_contact']=='1',row['motors_off']=='1',
            row.get('continuous_support')=='1' if c.get('require_continuous_support') else None,measured_stop)
        recomputed|=success
        if success:success_rows.append(float(row['sim_time_sec']))
    frames=list(csv.DictReader((path/'frames.csv').open(encoding='utf-8'))) if (path/'frames.csv').exists() else []
    selected=[f for f in frames if f['camera']=='down']
    if len(selected)>replay_count:
        selected=[selected[i] for i in np.linspace(0,len(selected)-1,replay_count,dtype=int)]
    sys.path.insert(0,c['sdk_path'])
    from projectairsim.utils import unpack_image
    detector=DeckDetector(c)
    replay=[]
    for frame in selected:
        payload=np.load(path/frame['payload'])
        message=json.loads(str(payload['metadata_json']));message['data']=payload['payload'].tobytes()
        decoded=unpack_image(message)
        png=cv2.imread(str(path/frame['raw_image']),cv2.IMREAD_UNCHANGED)
        d=detector.detect(message,decoded)
        old=json.loads((path/frame['detection']).read_text(encoding='utf-8'))
        agree=d['visible']==old['visible']
        if d['visible'] and old['visible']:
            agree=agree and d['selected']['id']==old['selected']['id'] and np.allclose(d['selected']['corners'],old['selected']['corners'],atol=.05)
        replay.append({'time_ns':frame['time_ns'],'png_matches_original_payload':bool(np.array_equal(decoded,png)),
                        'detection_replay_agrees':bool(agree)})
    report={'trial':path.name,'integrity_ok':not bad,'bad_files':bad,'checked_files':len(inventory),
       'claimed_success':manifest.get('success',False),'recomputed_success':bool(recomputed),
       'success_claim_agrees':bool(recomputed)==bool(manifest.get('success',False)),
       'success_times_sec':success_rows,'trajectory_samples':len(rows),'replayed_frames':replay,
       'camera_replay_ok':bool(replay) and all(r['png_matches_original_payload'] and r['detection_replay_agrees'] for r in replay),
       'rotor_verification_required':rotor_required,
       'rotor_raw_replay_ok':bool(rotor_messages) and bool(rotor_checks) and all(rotor_checks) if rotor_required else None,
       'verified_motor_off_landing':bool(recomputed and rotor_required and rotor_messages and all(rotor_checks)),
       'legacy_limitation':None if rotor_required else 'Disarm acknowledgement only; physical rotor shutdown was not verified. Do not treat a v1 success as verified motor-off landing.',
       'scientific_scope':'Integrity and deterministic replay audit; not independent attestation or evidence of full hydrodynamic validity'}
    report['audit_passed']=report['integrity_ok'] and report['success_claim_agrees'] and report['camera_replay_ok'] and (not rotor_required or report['rotor_raw_replay_ok'])
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input-dir',required=True,type=Path);p.add_argument('--replay-count',type=int,default=20)
    a=p.parse_args()
    trials=[a.input_dir] if (a.input_dir/'manifest.json').exists() else sorted(a.input_dir.glob('trial_*'))
    reports=[audit_trial(t,a.replay_count) for t in trials]
    report={'trials':reports,'audit_passed':bool(reports) and all(r['audit_passed'] for r in reports)}
    write_json(a.input_dir/'audit_report.json',report)
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['audit_passed'] else 2)
