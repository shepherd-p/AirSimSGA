"""Estimate physical tag size from archived, synchronized RGB and planar depth.

This is a separate static calibration. Depth is not used by the visual landing
controller. Original trial files are never rewritten.
"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from ag.math3d import camera_pose, rotation
from ag.storage import write_json


def calibrate(path):
    frames=list(csv.DictReader((path/'frames.csv').open(encoding='utf-8')))
    trajectory=list(csv.DictReader((path/'trajectory.csv').open(encoding='utf-8')))
    depth={int(r['time_ns']):r for r in frames if r['camera']=='depth'}
    if not depth:raise ValueError('No archived calibration depth frames')
    samples=[]
    for r in frames:
        if r['camera']!='down' or r['visible']!='1':continue
        stamp=int(r['time_ns']);nearest=min(depth,key=lambda t:abs(t-stamp))
        if abs(nearest-stamp)>20_000_000:continue
        detection=json.loads((path/r['detection']).read_text(encoding='utf-8'))
        tag=detection['selected']
        z=cv2.imread(str(path/depth[nearest]['raw_image']),cv2.IMREAD_UNCHANGED)
        payload=np.load(path/depth[nearest]['payload'])
        meta=json.loads(str(payload['metadata_json']))
        if meta['encoding']!='16UC1':raise ValueError('Calibration needs verified millimetre planar depth')
        cp,cr=camera_pose(meta)
        fx,fy,cx,cy=detection['intrinsics']
        corners=np.asarray(tag['corners']);center=np.asarray(tag['center'])
        inner=center+.7*(corners-center)
        points=[]
        for u,v in inner:
            x,y=int(round(u)),int(round(v))
            if not 3<=x<z.shape[1]-3 or not 3<=y<z.shape[0]-3:break
            distance=np.median(z[y-2:y+3,x-2:x+3])/1000.
            points.append(cp+cr@np.array([distance,(u-cx)*distance/fx,(v-cy)*distance/fy]))
        if len(points)!=4:continue
        points=np.array(points)
        sizes=np.linalg.norm(np.roll(points,-1,axis=0)-points,axis=1)/.7
        if min(sizes)<.1 or max(sizes)/min(sizes)>1.4:continue
        gt=min(trajectory,key=lambda row:abs(float(row['sim_time_sec'])-stamp/1e9))
        if float(gt['elapsed_sec'])<12 or np.linalg.norm([float(gt['uav_velocity_'+a]) for a in 'xyz'])>.08:
            continue
        samples.append({'time_ns':stamp,'tag_id':tag['id'],'side_m':float(np.mean(sizes)),
              'width_m':float(np.mean(sizes[[0,2]])),'height_m':float(np.mean(sizes[[1,3]])),
              'world_center_ned':points.mean(0).tolist(),
              'legacy_target_ned':[float(gt['deck_'+a]) for a in 'xyz']})
    if len(samples)<8:raise ValueError(f'Insufficient valid calibration measurements: {len(samples)}')
    sides=np.array([s['side_m'] for s in samples])
    centers=np.array([s['world_center_ned'] for s in samples])
    cfg=json.loads((path/'config.json').read_text(encoding='utf-8'))
    initial=json.loads((path/'initial_state.json').read_text(encoding='utf-8'))[cfg['usv']]
    vessel=np.array([initial['translation'][a] for a in 'xyz'])
    local=(centers-vessel)@rotation(initial['rotation'])
    widths=np.array([s['width_m'] for s in samples]);heights=np.array([s['height_m'] for s in samples])
    report={'source_trial':str(path.resolve()),'samples':samples,'sample_count':len(samples),
      'tag_size_median_m':float(np.median(sides)),'tag_size_std_m':float(np.std(sides)),
      'tag_width_m':float(np.median(widths)),'tag_height_m':float(np.median(heights)),
      'tag_center_hull_m':np.median(local,axis=0).tolist(),
      'center_std_m':np.std(local,axis=0).tolist(),
      'method':'Inward AprilTag corners back-projected with synchronized planar depth; 0.7 inset corrected for side length',
      'usage':'Static calibration only; landing control uses RGB AprilTag observations',
      'validated':bool(np.std(sides)<.025 and np.max(np.std(local,axis=0))<.08
                       and abs(np.median(widths)/np.median(heights)-1)<.05)}
    out=path/'derived';out.mkdir(exist_ok=True)
    write_json(out/'tag_calibration.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input-dir',type=Path,required=True)
    a=p.parse_args();report=calibrate(a.input_dir)
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
