"""Compute additional manuscript diagnostics from existing records, not reruns."""
from pathlib import Path
import csv
import hashlib
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT.parent
EXP = WORK.parent / 'ProjectAirSim_AG_Experiments'
OUT = ROOT / 'evidence'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def load(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))

def write(p, obj):
    p.write_text(json.dumps(obj, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')

def range_of(values):
    values=list(values)
    return [min(values), max(values)]

def q(values, fraction):
    a=sorted(values)
    index=(len(a)-1)*fraction
    i=int(index)
    return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(index-i)

sources={}
fleet=[]
fleet_root = OUT/'fleet' if (OUT/'fleet').exists() else EXP/'outputs/urban_scale'
for folder in sorted(fleet_root.iterdir()):
    if not folder.is_dir(): continue
    summary,config=folder/'summary.json',folder/'config.json'
    if not summary.exists() or not config.exists(): continue
    s,c=load(summary),load(config)
    assert s['status']=='completed' and not s['collision_events']
    assert abs(s['real_time_factor']-s['sim_duration_sec']/s['wall_duration_sec'])<1e-10
    resources=folder/'resources.csv'
    with resources.open(encoding='utf-8-sig',newline='') as f: samples=list(csv.DictReader(f))
    rates=[v['scene_camera'] for v in s['stream_hz_per_robot'].values()]
    row={'record_id':folder.name,'fleet_size_each_role':s['fleet_size_each_role'],
         'sim_duration_sec':s['sim_duration_sec'],'wall_duration_sec':s['wall_duration_sec'],
         'real_time_factor':s['real_time_factor'],
         'simulator_cpu_core_equivalents':s['process_cpu_percent_one_core_100']['simulator']/100,
         'client_cpu_core_equivalents':s['process_cpu_percent_one_core_100']['client']/100,
         'simulator_peak_rss_GiB':s['peak_rss_bytes']['simulator']/1024**3,
         'client_peak_rss_GiB':s['peak_rss_bytes']['client']/1024**3,
         'scene_camera_hz_per_actor_range':range_of(rates),
         'pose_hz_range':range_of(v['pose'] for v in s['stream_hz_per_robot'].values()),
         'resource_samples':len(samples),'recorded_host':c['host']}
    fleet.append(row)
    dst=OUT/'fleet'/folder.name
    dst.mkdir(parents=True,exist_ok=True)
    for f in (summary,config,resources,folder/'provenance.json'):
        if f.exists():
            sources[str(f)]=sha(f)
            if (dst/f.name).resolve() != f.resolve():
                (dst/f.name).write_bytes(f.read_bytes())
groups=[]
for n in (1,2,3):
    selected=[v for v in fleet if v['fleet_size_each_role']==n]
    assert len(selected)==2
    groups.append({'fleet_size_each_role':n,'runs':2,
       **{k:range_of(v[k] for v in selected) for k in
          ('sim_duration_sec','wall_duration_sec','real_time_factor','simulator_cpu_core_equivalents','simulator_peak_rss_GiB')},
       'scene_camera_hz_per_actor_range':range_of(x for v in selected for x in v['scene_camera_hz_per_actor_range'])})
write(OUT/'fleet_diagnostics.json',{'definition':'Offline reanalysis of six existing recorded runs; no new simulation executions.',
    'runs':fleet,'groups':groups,'source_sha256':sources})

land=OUT/'landing_trajectory.csv'
with land.open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
last=float(rows[-1]['elapsed_sec'])
dwell=float(rows[-1]['dwell_sec'])
terminal=[r for r in rows if float(r['elapsed_sec'])>=last-dwell-1e-7]
assert len(rows)==399 and len(terminal)>=30
for r in terminal:
    assert all(r[key]=='1' for key in ('support_valid','continuous_support','motors_off','rotors_stopped'))
dt=[float(b['sim_time_sec'])-float(a['sim_time_sec']) for a,b in zip(rows,rows[1:])]
terminal_xy=[[float(r['relative_deck_x']),float(r['relative_deck_y'])] for r in terminal]
slip=max(math.dist(a,b) for a in terminal_xy for b in terminal_xy)
ages=[float(r['camera_age_sec']) for r in rows if r['camera_age_sec']]
result={'definition':'Offline diagnostics of the recorded moving-USV landing; not a new trial or repeatability study.',
 'samples':len(rows),'terminal_samples':len(terminal),'terminal_interval_elapsed_sec':[float(terminal[0]['elapsed_sec']),last],
 'terminal_supported_duration_sec':dwell,'terminal_all_support_and_shutdown_flags_valid':True,
 'terminal_horizontal_error_m_range':range_of(float(r['horizontal_error_m']) for r in terminal),
 'terminal_relative_speed_mps_max':max(float(r['relative_speed_mps']) for r in terminal),
 'terminal_deck_frame_xy_diameter_m':slip,
 'terminal_rotor_max_speed_radps':max(float(r['rotor_max_speed_radps']) for r in terminal),
 'terminal_total_thrust_n_max':max(float(r['rotor_total_thrust_n']) for r in terminal),
 'control_log_interval_sec_range':range_of(dt),
 'camera_age_sec':{'n':len(ages),'median':statistics.median(ages),'p95':q(ages,.95),'max':max(ages)},
 'source_sha256':{str(land):sha(land)}}
write(OUT/'landing_diagnostics.json',result)
print(json.dumps({'fleet_groups':groups,'landing':result},indent=2))
