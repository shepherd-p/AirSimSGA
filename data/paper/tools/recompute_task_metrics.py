"""Recalculate manuscript trajectory metrics from bundled data; standard library only."""
from pathlib import Path
import csv
import json
import math

ROOT=Path(__file__).resolve().parents[1]
E=ROOT/'evidence'
def load(name): return json.loads((E/name).read_text(encoding='utf-8-sig'))
def rows(name):
    with (E/name).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def pos(r,prefix='',dims='xyz'): return tuple(float(r[prefix+k]) for k in dims)
def length(points): return sum(math.dist(a,b) for a,b in zip(points,points[1:]))
def stats(values):
    a=[v for v in values if math.isfinite(v)]
    return {'n':len(a),'rmse':math.sqrt(sum(v*v for v in a)/len(a)),'max':max(a),'final':a[-1]}
def point_segment(p,a,b):
    v=(b[0]-a[0],b[1]-a[1]); l2=sum(x*x for x in v)
    t=max(0,min(1,sum((p[i]-a[i])*v[i] for i in range(2))/max(l2,1e-12)))
    return math.dist(p,(a[0]+t*v[0],a[1]+t*v[1]))

nav_rows=rows('navigation_trajectories_controls.csv')
plan=load('navigation_committed_plan.json')['waypoints_ned']
active=[r for r in nav_rows if r['robot']=='Car1' and r['phase']=='ugv_route_execution']
errors=[]
for r in active:
    err=min(point_segment(pos(r,dims='xy'),a,b) for a,b in zip(plan,plan[1:]))
    assert abs(err-float(r['cross_track_m']))<1e-8
    errors.append(err)
nav={'plan_length_m':length(plan),'controller_distance_m':load('navigation_summary.json')['car_distance_m'],
     'saved_polyline_length_m':length([pos(r) for r in nav_rows if r['robot']=='Car1']),
     'cross_track_m':stats(errors),'collision_events':load('navigation_summary.json')['collision_events']}

bytime={}
for r in rows('tracking_trajectories_controls.csv'):
    bytime.setdefault(float(r['sim_time_sec']),{})[r['robot']]=r
tracking=rows('tracking_tracking.csv'); errors=[]
for r in tracking:
    if not r['estimate_x'] or not r['estimate_y']:continue
    estimate=(float(r['estimate_x']),float(r['estimate_y']))
    if not all(math.isfinite(v) for v in estimate):continue
    actors=bytime[float(r['sim_time_sec'])]
    err=math.dist(estimate,pos(actors['Target'],dims='xy'))
    assert abs(err-float(r['error_m_evaluation_only']))<1e-8
    errors.append(err)
scenario=load('tracking_scenario.json')
last=bytime[max(bytime)]
track={'samples':len(tracking),'valid_estimates':len(errors),
       'estimate_availability':len(errors)/len(tracking),'estimate_error_xy_m':stats(errors),
       'target_distance_m':load('tracking_summary.json')['distances_m']['Target'],
       'final_exit_errors_m':[math.dist(pos(last['Gate'+side],dims='xy'),scenario['gate_ned'][side][:2]) for side in ('Left','Right')]}

landing=rows('landing_trajectory.csv');last=landing[-1]
first=next(r for r in landing if r['support_valid']=='1' and r['vessel_contact']=='1')
land={'samples':len(landing),'contact_elapsed_sec':float(first['elapsed_sec']),
      'contact_error_m':float(first['horizontal_error_m']),'completion_elapsed_sec':float(last['elapsed_sec']),
      'final_error_m':float(last['horizontal_error_m']),'supported_dwell_sec':float(last['dwell_sec']),
      'vessel_distance_m':length([pos(r,'usv_') for r in landing])}

mapping=rows('mapping/trajectories.csv')
robot_key=next(key for key in ('robot','name','vehicle') if key in mapping[0])
paths={robot:length([pos(r) for r in mapping if r[robot_key]==robot]) for robot in ('Drone1','Car1')}
manifest=load('mapping/manifest.json')
for robot,value in paths.items():assert abs(value-manifest['path_lengths_m'][robot])<1e-6
quality=load('mapping/plaza_results/surface/quality.json')
report={'navigation':nav,'tracking':track,'landing':land,
        'mapping':{'trajectory_lengths_m':paths,'integration_frames':quality['integration_frames'],
                   'recorded_surface_evaluation':quality['evaluation']}}
(E/'recomputed_task_metrics.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
