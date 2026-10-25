"""Decode a saved original sensor payload to an image, without touching sealed evidence."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from record_water_landing import unpack_image
p=argparse.ArgumentParser();p.add_argument('payload',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
if args.output.exists():raise FileExistsError(args.output)
with np.load(args.payload,allow_pickle=False) as data:
 msg=json.loads(str(data['metadata_json']));msg['data']=data['payload'].tobytes()
img=unpack_image(msg);args.output.parent.mkdir(parents=True,exist_ok=True)
assert cv2.imwrite(str(args.output),img)
print(args.output)
