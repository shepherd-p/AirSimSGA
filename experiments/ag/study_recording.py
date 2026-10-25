"""Asynchronous original camera payload and pose/collision topic archives."""
from collections import Counter
import csv
import gzip
import json
import queue
import threading
import numpy as np
from .storage import write_json,serial


class TopicRecorder:
    def __init__(self,folder):
        self.folder=folder;self.queue=queue.Queue(8192);self.counts=Counter();self.errors=[]
        self.thread=threading.Thread(target=self.work,daemon=True);self.thread.start()

    def receive(self,key,message):
        if not key.endswith(('/pose','/collision')):return
        try:self.queue.put_nowait((key,message))
        except queue.Full:self.counts['queue_drops']+=1

    def work(self):
        with gzip.open(self.folder/'pose_collision_topics.jsonl.gz','wt',encoding='utf-8',compresslevel=1) as stream:
            while True:
                item=self.queue.get()
                try:
                    if item is None:return
                    key,message=item
                    stream.write(json.dumps(serial({'stream':key,'message':message}),separators=(',',':'))+'\n')
                    self.counts[key]+=1
                except Exception as exc:self.errors.append(str(exc))
                finally:self.queue.task_done()

    def close(self):
        self.queue.put(None,timeout=15);self.thread.join(30)
        if self.thread.is_alive():self.errors.append('Topic worker did not finish')
        report={'counts':dict(self.counts),'errors':self.errors,'source':'Original received actual_pose and collision_info messages during capture'}
        write_json(self.folder/'topic_archive.json',report)
        if self.errors or self.counts['queue_drops']:raise RuntimeError('Topic archive incomplete')
        return report


class CameraRecorder:
    def __init__(self,backend,folder):
        self.b=backend;self.folder=folder;self.queue=queue.Queue(24)
        self.counts=Counter();self.errors=[];self.last={};self.boxes={}
        from projectairsim.image_utils import segmentation_id_to_color
        self.colors={}
        for name,sid in backend.config['study']['segmentation_ids'].items():
            color=segmentation_id_to_color(sid);self.colors[name]=[color.b,color.g,color.r]
        self.writer_lock=threading.Lock()
        self.stream=(self.folder/'camera_frames.csv').open('w',encoding='utf-8',newline='')
        self.writer=csv.DictWriter(self.stream,fieldnames=['camera','time_ns','file','image','width','height','pair_delta_ms','visible_robots'])
        self.writer.writeheader()
        self.workers=[threading.Thread(target=self.work,daemon=True) for _ in range(2)]
        for worker in self.workers:worker.start()

    def collect(self,force=False):
        interval=self.b.config['study']['frame_interval_sec']
        for camera,leaves in self.b.cameras.items():
            if camera=='Target':continue
            primary=camera+'/scene_camera'
            message=self.b.buffer.latest(primary,2.)
            if message is None:continue
            stamp=int(message['time_stamp']);period=1. if camera.startswith('Observer') else interval
            if stamp<=self.last.get(camera,-1) or (not force and (stamp-self.last.get(camera,-10**15))/1e9<period-.02):continue
            bundle={'scene_camera':message}
            for leaf in leaves:
                if leaf=='scene_camera':continue
                candidates=[m for _,m in self.b.buffer.entries(camera+'/'+leaf)]
                if not candidates:break
                closest=min(candidates,key=lambda m:abs(int(m['time_stamp'])-stamp))
                if abs(int(closest['time_stamp'])-stamp)>30_000_000:break
                bundle[leaf]=closest
            if len(bundle)!=len(leaves):
                self.counts['unsynchronized_attempts']+=1;continue
            try:self.queue.put_nowait((camera,stamp,bundle));self.last[camera]=stamp
            except queue.Full:self.counts['queue_drops']+=1

    def work(self):
        import cv2
        from projectairsim.utils import unpack_image
        if True:
            while True:
                item=self.queue.get()
                try:
                    if item is None:return
                    camera,stamp,bundle=item
                    out=self.folder/'sensor_frames'/camera.replace('/','_');out.mkdir(parents=True,exist_ok=True)
                    metadata={leaf:{k:v for k,v in msg.items() if k!='data'} for leaf,msg in bundle.items()}
                    payload={leaf:np.frombuffer(bytes(msg['data']),dtype='u1') for leaf,msg in bundle.items()}
                    payload['metadata_json']=np.array(json.dumps(serial(metadata),separators=(',',':')))
                    # Lossless, uncompressed ZIP avoids compression blocking the
                    # simulator's camera receipt rate. No data values are changed.
                    save=np.savez_compressed if self.b.config['study'].get('lossless_payload_compression',False) else np.savez
                    save(out/f'{stamp}.npz',**payload)
                    rgb=unpack_image(bundle['scene_camera']);boxes={}
                    if 'segmentation_camera' in bundle:
                        seg=unpack_image(bundle['segmentation_camera'])
                        for name,color in self.colors.items():
                            yy,xx=np.nonzero(np.all(seg[:,:,:3]==color,axis=2))
                            if len(xx):boxes[name]={'pixels':int(len(xx)),'bbox':[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())]}
                    if not cv2.imwrite(str(out/f'{stamp}.png'),rgb):raise OSError('PNG write failed')
                    record={'camera':camera,'time_ns':stamp,'file':str((out/f'{stamp}.npz').relative_to(self.folder)),
                        'image':str((out/f'{stamp}.png').relative_to(self.folder)), 'width':rgb.shape[1],'height':rgb.shape[0],
                        'pair_delta_ms':max(abs(int(m['time_stamp'])-stamp) for m in bundle.values())/1e6,
                        'visible_robots':json.dumps(boxes,separators=(',',':'))}
                    with self.writer_lock:
                        self.writer.writerow(record);self.counts[camera]+=1;self.boxes[camera]=boxes
                        if self.counts[camera]%20==0:self.stream.flush()
                except Exception as exc:self.errors.append(f'{type(exc).__name__}: {exc}')
                finally:self.queue.task_done()

    def check(self):
        if self.errors or self.counts['queue_drops']:raise RuntimeError(f'Camera archive failed: queue drops={self.counts["queue_drops"]}; '+str(self.errors[:2]))

    def close(self):
        for _ in self.workers:self.queue.put(None,timeout=30)
        for worker in self.workers:worker.join(90)
        if any(w.is_alive() for w in self.workers):self.errors.append('Camera worker did not finish')
        else:self.stream.close()
        report={'counts':dict(self.counts),'errors':self.errors,'segmentation_bgr':self.colors,
                'source':'Selected synchronized original message payload bytes; PNG is lossless decoded RGB',
                'npz_codec':'Uncompressed lossless ZIP; two independent writers, serialized CSV index',
                'selection':'At most one newest paired exposure per configured interval, not every camera topic message'}
        write_json(self.folder/'camera_archive.json',report);self.check();return report
