"""Overlay proposed regions on the archived real-camera map overview."""
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ag.math3d import camera_pose
from ag.storage import ROOT, write_json


def main():
    survey=ROOT/'outputs/region_survey/20260928_064329'
    detail=ROOT/'outputs/region_survey/20260928_064720'
    out=ROOT/'delivery/task_regions';out.mkdir(parents=True,exist_ok=True)
    payload=np.load(survey/'overview_rgb.npz');meta=json.loads(str(payload['metadata_json']))
    pos,rot=camera_pose(meta);w,h=meta['width'],meta['height'];f=w/2
    regions=[
      {'id':'mapping','label':'A  Mapping (accepted)','north':[10,60],'east':[35,80],'color':'#0d6efd','status':'Existing accepted plaza experiment'},
      {'id':'navigation','label':'B  Garden navigation','north':[17,47],'east':[121,183],'color':'#e57700','status':'Separate east garden; live route validation in progress. Former road candidate has incompatible slope for planar rover.'},
      {'id':'tracking','label':'C  Bridge occlusion tracking','north':[-54,-5],'east':[70,160],'color':'#9b30d0','status':'Surveyed natural bridge candidate; under-bridge clearance and tracking runs pending'},
      {'id':'landing','label':'D  Moving-deck landing','north':[22,36],'east':[-14,27],'color':'#008466','status':'One stationary and one 0.5 m/s trial pass v2 support + physical rotor shutdown audit; repeated statistics pending'}]
    im=Image.open(survey/'overview_rgb.png').convert('RGB');draw=ImageDraw.Draw(im)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',26)
    def pixel(n,e,z=5.):
        p=rot.T@(np.array([n,e,z])-pos)
        return (int(w/2+f*p[1]/p[0]),int(h/2+f*p[2]/p[0]))
    for r in regions:
        n0,n1=r['north'];e0,e1=r['east']
        polygon=[pixel(n,e) for n,e in [(n0,e0),(n1,e0),(n1,e1),(n0,e1),(n0,e0)]]
        draw.line(polygon,fill=r['color'],width=6)
        x=min(p[0] for p in polygon);y=min(p[1] for p in polygon)-37
        box=draw.textbbox((x,y),r['label'],font=font)
        draw.rectangle([box[0]-5,box[1]-5,box[2]+5,box[3]+5],fill='white')
        draw.text((x,y),r['label'],fill=r['color'],font=font)
    draw.rectangle([0,h-52,w,h],fill='white')
    draw.text((20,h-42),'Same CityPark UE level | Proposed task regions, not traversability certification',font=font,fill='#1e293b')
    im.save(out/'region_atlas.png')
    write_json(out/'regions.json',{'coordinate_system':'NED metres','regions':regions,
       'overview_source':str(survey/'overview_rgb.png'),'detail_survey':str(detail),
       'overlay_projection':'Approximate z=5 m plane only for locating regions; original RGB and depth remain available',
       'same_map_limit':'Different task areas do not demonstrate cross-map generalization',
       'scale_test_proposal':{'uav_ugv_counts':[[1,1],[2,2],[3,2],[4,4]],'status':'not run',
           'metrics':['simulation/wall time ratio','control latency p50/p95','per-sensor delivery rates','process CPU/RSS','collisions and minimum separation'],
           'controls':'Fixed per-agent sensing and motion workload, repeat count and hardware; active-agent counts exclude observation camera'}})
    print(out/'region_atlas.png')


if __name__=='__main__':main()
