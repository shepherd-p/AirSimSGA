"""Verify retained controller input payloads, timestamps and lossless preview decoding."""
import csv,json,sys
from pathlib import Path
import cv2,numpy as np
from record_water_landing import unpack_image
root=Path(__file__).resolve().parents[1];take=root/sys.argv[1]
with (take/'experiment/frames.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
checks=[];selected=[]
for row in rows:
 payload=take/'experiment'/row['payload'];assert payload.is_file(),str(payload)
 with np.load(payload,allow_pickle=False) as data:
  msg=json.loads(str(data['metadata_json']));assert int(msg['time_stamp'])==int(row['time_ns']);assert data['payload'].dtype==np.uint8
  if row['raw_image'] and row['camera']!='depth' and not any(r['camera']==row['camera'] and r['phase']==row['phase'] for r in selected):
   msg['data']=data['payload'].tobytes();img=unpack_image(msg)
   saved=cv2.imread(str(take/'experiment'/row['raw_image']),cv2.IMREAD_UNCHANGED)
   assert saved is not None and np.array_equal(saved,img),row
   selected.append(row)
 if row['detection']:assert (take/'experiment'/row['detection']).is_file()
report={'every_controller_consumed_payload_present':True,'payloads_verified':len(rows),'timestamp_metadata_verified':True,'representative_pngs_exactly_match_lossless_payloads':len(selected),'retained_payload_format':'Lossless compressed NPZ with original sensor bytes and exposure metadata; PNG previews are sparse, not the underlying input payloads'}
(root/'input_payload_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report,indent=2))
