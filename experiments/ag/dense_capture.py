"""Capture synchronized RGB/range/segmentation plus camera poses for offline fusion."""
import json
import numpy as np
from .math3d import camera_pose


class DenseRecorder:
    def __init__(self, backend, run, options):
        self.b, self.run, self.opt = backend, run, options
        self.last = {}
        self.counts = {}
        self.skipped = {}

    def record(self, name):
        from projectairsim.utils import unpack_image
        from projectairsim.image_utils import segmentation_id_to_color
        pair = self.b.buffer.pair(name+'/depth_camera', name+'/scene_camera',
                                  self.b.config['sensor_max_age_sec'], .015)
        if pair is None:
            self.skipped[name] = self.skipped.get(name, 0)+1
            return
        dm, cm = pair
        t = int(dm['time_stamp'])/1e9
        p, r = camera_pose(dm)
        cp, cr = camera_pose(cm)
        if np.linalg.norm(cp-p) > .035 or np.linalg.norm(cr-r) > .035:
            return  # Reject color from a materially different camera pose.
        old = self.last.get(name)
        if old:
            if t-old[0] < self.opt['min_interval_sec']:
                return
            angle = np.arccos(np.clip((np.trace(r@old[2].T)-1)/2, -1, 1))
            if np.linalg.norm(p-old[1]) < .25 and angle < np.deg2rad(3) and t-old[0] < 1.5:
                return
        depth = self.b.depth_m(dm, unpack_image(dm)).astype(np.float32)
        color = unpack_image(cm)
        if color.shape[:2] != depth.shape:
            raise ValueError('RGB/depth dimensions differ')
        # Existing ProjectAirSim unpack_image returns OpenCV BGR images.
        rgb = np.ascontiguousarray(color[:, :, :3][:, :, ::-1])
        valid = np.isfinite(depth)&(depth>.5)&(depth<self.opt['depth_max_m'])
        sg = self.b.buffer.pair(name+'/depth_camera', name+'/segmentation_camera',
                                self.b.config['sensor_max_age_sec'], .015)
        masked = 0
        if sg is not None and sg[0]['time_stamp'] == dm['time_stamp']:
            seg = unpack_image(sg[1])
            for sid in self.opt['robot_segmentation_ids'].values():
                c = segmentation_id_to_color(sid)
                hit = np.all(seg[:, :, :3] == np.array([c.b,c.g,c.r]), axis=2)
                masked += int(hit.sum())
                valid &= ~hit
        # Range discontinuities mix foreground/background samples: leave a gap.
        jump = np.zeros(depth.shape, dtype=bool)
        dy = np.abs(depth[1:]-depth[:-1]) > self.opt['edge_threshold_m']
        dx = np.abs(depth[:,1:]-depth[:,:-1]) > self.opt['edge_threshold_m']
        jump[1:] |= dy; jump[:-1] |= dy; jump[:,1:] |= dx; jump[:,:-1] |= dx
        valid &= ~jump
        depth[~valid] = 0
        folder = self.run.path/'rgbd'/name
        folder.mkdir(parents=True, exist_ok=True)
        file = folder/f"{dm['time_stamp']}.npz"
        np.savez_compressed(file, range_m=depth, rgb=rgb,
                            camera_position_ned=p, camera_rotation_ned=r,
                            fov_deg=self.opt['fov_deg'], time_ns=dm['time_stamp'])
        self.run.row('rgbd_frames', {'name':name,'time_ns':dm['time_stamp'],
                    'color_time_ns':cm['time_stamp'],'valid_pixels':int(valid.sum()),
                    'robot_pixels_masked':masked,'file':str(file.relative_to(self.run.path)),
                    'position_ned':json.dumps(p.tolist())})
        self.last[name] = t, p, r
        self.counts[name] = self.counts.get(name, 0)+1
