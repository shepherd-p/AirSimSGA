"""Select a real camera frame and annotate the two measured robot locations."""
import hashlib
import json
import shutil
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont


def select_capture(captures):
    eligible=[]
    for item in captures:
        width=item['capture_metadata']['width'];height=item['capture_metadata']['height']
        visible=item.get('visible_robots',{})
        valid=True
        for name,minimum in [('Drone1',250),('Car1',1000)]:
            record=visible.get(name,{})
            box=record.get('bbox',[0,0,width,height])
            valid &= (record.get('pixels',0)>=minimum and
                      8<=box[0]<box[2]<width-8 and 8<=box[1]<box[3]<height-8)
        if valid:eligible.append(item)
    if not eligible:
        raise ValueError('No frame shows both complete robots clearly; raw camera frames retained for diagnosis')
    return max(eligible,key=lambda item:item['visible_robots']['Drone1']['pixels'])


def prepare_scene_figure(folder):
    folder=Path(folder)
    record=json.loads((folder/'scene_capture.json').read_text(encoding='utf-8'))
    chosen=select_capture(record['captures'])
    source=folder/chosen['image']
    shutil.copy2(source,folder/'scene_overview.png')
    im=Image.open(source).convert('RGB')
    draw=ImageDraw.Draw(im)
    font=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',42)
    for name,label,color in [('Drone1','UAV','#ed2224'),('Car1','UGV','#1746f5')]:
        x0,y0,x1,y1=chosen['visible_robots'][name]['bbox']
        if name=='Drone1':
            tx,ty=x1+155,y0-88;anchor=(x1+9,(y0+y1)//2)
            line=[anchor,(tx-25,ty+37),(tx,ty+37)]
        else:
            tx,ty=x0-325,y0+25;anchor=(x0-9,(y0+y1)//2)
            line=[anchor,(tx+150,ty+37),(tx+115,ty+37)]
        draw.line(line,fill='white',width=8)
        draw.line(line,fill=color,width=4)
        box=draw.textbbox((tx,ty),label,font=font)
        draw.rounded_rectangle((box[0]-14,box[1]-10,box[2]+14,box[3]+10),radius=8,fill='white')
        draw.text((tx,ty),label,font=font,fill=color)
    im.save(folder/'scene_overview_labeled.png',dpi=(300,300))
    manifest={
        'provenance':record['provenance'],
        'original_image':chosen['image'],
        'original_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'raw_output':'scene_overview.png',
        'labeled_output':'scene_overview_labeled.png',
        'annotation':'UAV/UGV labels only; positions from synchronized segmentation; no robot compositing',
        'selected_capture':chosen,
    }
    (folder/'scene_figure.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return manifest
