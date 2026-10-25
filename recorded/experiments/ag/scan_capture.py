"""Bounded, all-message LiDAR archive; never performs disk IO in SDK callbacks."""
import csv
import json
import queue
import threading
from collections import Counter
import numpy as np
from .math3d import pose_parts
from .storage import write_json


class ScanRecorder:
    def __init__(self, path, capacity=256):
        self.path = path
        self.queue = queue.Queue(capacity)
        self.lock = threading.Lock()
        self.accepting = True
        self.stats = Counter()
        self.errors = []
        self.seen = set()
        self.thread = threading.Thread(target=self._work, name='lidar-archive', daemon=True)
        self.thread.start()

    def receive(self, key, message):
        if not key.endswith('/lidar'):
            return
        with self.lock:
            if not self.accepting:
                return
            self.stats['received'] += 1
            try:
                self.queue.put_nowait((key.split('/')[0], message))
                self.stats['enqueued'] += 1
            except queue.Full:
                self.stats['queue_drops'] += 1

    def _save(self, name, m, writer):
        stamp = int(m['time_stamp'])
        if (name, stamp) in self.seen:
            self.stats['duplicates'] += 1
            return
        self.seen.add((name, stamp))
        local = np.asarray(m['point_cloud'], dtype=np.float32).reshape(-1, 3)
        valid = np.isfinite(local).all(axis=1) & (np.linalg.norm(local, axis=1) > .05)
        seg = np.asarray(m.get('segmentation_cloud', []))
        if len(seg) == len(local):
            valid &= ~np.isin(seg, [240, 241])
        p, r = pose_parts(m['pose'])
        world = (local[valid] @ r.T + p).astype(np.float32)
        folder = self.path/'raw'/name
        folder.mkdir(parents=True, exist_ok=True)
        filename = folder/f'{stamp}.npz'
        np.savez_compressed(filename, local_points=local[valid], world_points=world,
                            time_ns=np.int64(stamp), sensor_position_ned=p, sensor_rotation_ned=r)
        writer.writerow([name, stamp, len(world), json.dumps(m['pose']), str(filename.relative_to(self.path))])
        self.stats['saved'] += 1
        self.stats[name+'_saved'] += 1
        self.stats[name+'_points'] += len(world)
        self.stats['invalid_or_robot_returns'] += len(local)-len(world)

    def _work(self):
        with (self.path/'scans.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['name','time_ns','points','pose','file'])
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        return
                    self._save(*item, writer)
                except Exception as exc:
                    self.errors.append(f'{type(exc).__name__}: {exc}')
                    self.stats['write_errors'] += 1
                finally:
                    self.queue.task_done()

    def close(self):
        with self.lock:
            self.accepting = False
        try:
            self.queue.put(None,timeout=10)
        except queue.Full:
            self.errors.append('Archive worker did not accept shutdown within 10 seconds')
        self.thread.join(timeout=60)
        if self.thread.is_alive():
            self.errors.append('Archive worker did not finish within 60 seconds')
        report = dict(self.stats)
        report.update(errors=self.errors, queue_drops=self.stats['queue_drops'],
                      transport_loss='Not measurable: server packets have no sequential packet ID')
        write_json(self.path/'lidar_capture.json', report)
        if self.errors or self.stats['queue_drops']:
            raise RuntimeError('LiDAR archive incomplete; see lidar_capture.json')
        return report
