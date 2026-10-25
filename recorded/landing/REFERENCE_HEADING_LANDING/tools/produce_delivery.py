"""Produce reviewer videos only after a successful live recorded landing."""
import json,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
take=root/'results/landing_take_01'
result=json.loads((take/'experiment_result.json').read_text(encoding='utf-8-sig'))
assert result['success'] and result['reason']=='verified_contact_and_motor_off_dwell',result
logs=root/'processing_logs';logs.mkdir(exist_ok=True)
steps=[('compose_water_landing.py',['landing',str(take),'--speed','1']),
       ('export_video.py',['results/landing_take_01']),
       ('check_video.py',['results/landing_take_01']),
       ('append_result_card.py',['results/landing_take_01']),
       ('check_video.py',['results/landing_take_01']),
       ('audit_sensor_inputs.py',['results/landing_take_01']),
       ('audit_delivery.py',['results/landing_take_01']),
       ('verify_previous_delivery.py',[]),
       ('extract_posters.py',[]),
       ('publish_water_page.py',['results/landing_take_01']),
       ('seal_delivery.py',[])]
for i,(name,args) in enumerate(steps,1):
    print(f'DELIVERY {i}/{len(steps)} {name}',flush=True)
    logfile=logs/f'{i:02d}_{name}.log'
    with logfile.open('w',encoding='utf-8') as f:
        p=subprocess.run([sys.executable,str(root/'tools'/name)]+args,stdout=f,stderr=subprocess.STDOUT,cwd=root)
    if p.returncode:
        print(logfile.read_text(encoding='utf-8')[-6000:],flush=True)
        raise RuntimeError(f'{name} failed; see {logfile}')
q=json.loads((root/'quality_report.json').read_text(encoding='utf-8-sig'))
print(json.dumps({'page':str(root/'video_overview.html'),'landing':q['landing'],'boat':q['boat'],'bow_alignment':q['bow_alignment'],'cameras':{k:v['independent_exposure_rate_trial_sim_hz'] for k,v in q['cameras'].items()}},indent=2),flush=True)
