from pathlib import Path
import numpy as np,cv2,json
root=Path(r'F:/EpicGamesEnv/LandingWaterLab');v=np.array([[float(x) for x in line.split()[1:4]] for line in (root/'assets/boat_mesh_orientation.obj').read_text().splitlines() if line.startswith('v ')])
print('OBJ ranges',v.min(0),v.max(0))
# OBJ exporter uses X, Z, -Y. Longitudinal X sign is preserved.
axis=2 if np.ptp(v[:,2])<np.ptp(v[:,1]) else 1
report={'vertex_count':len(v),'min_xyz':v.min(0).tolist(),'max_xyz':v.max(0).tolist(),'lateral_axis_index':axis,'end_cross_sections':{}}
for end,label in [(v[:,0].min(),'negative_x_end'),(v[:,0].max(),'positive_x_end')]:
 p=v[np.abs(v[:,0]-end)<70]
 report['end_cross_sections'][label]={'longitudinal_end_cm':float(end),'lateral_extent_cm':float(np.ptp(p[:,axis]))}
img=np.full((420,1300,3),245,dtype=np.uint8)
pts=np.column_stack((80+(v[:,0]-v[:,0].min())/np.ptp(v[:,0])*1120,210+v[:,axis]*1.1)).astype(int)
for x,y in pts:
 if 0<=x<1300 and 0<=y<420:img[y,x]=(70,70,70)
cv2.putText(img,'Mesh -X',(40,40),cv2.FONT_HERSHEY_SIMPLEX,.9,(30,30,30),2)
cv2.putText(img,'Mesh +X',(1050,40),cv2.FONT_HERSHEY_SIMPLEX,.9,(30,30,30),2)
cv2.imwrite(str(root/'results/boat_asset_top_view.png'),img)
(root/'results/boat_cross_sections.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report['end_cross_sections'],indent=2))
