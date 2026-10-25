"""Actual AprilTag image measurements; never manufacture detections from truth."""
import math
import cv2
import numpy as np
from .math3d import camera_pose


class DeckDetector:
    def __init__(self, config):
        from pupil_apriltags import Detector
        self.c = config
        self.detector = Detector(families=config['tag_family'], nthreads=2,
                                 quad_decimate=1., refine_edges=1)

    def detect(self, message, image):
        h, w = image.shape[:2]
        focal = .5*w/math.tan(math.radians(self.c['camera_fov_deg'])/2)
        intrinsics = [focal, focal, w/2, h/2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        preprocessing=self.c.get('tag_preprocessing','none')
        if preprocessing == 'clahe':
            gray=cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8)).apply(gray)
        elif preprocessing != 'none':
            raise ValueError('Unknown tag preprocessing: '+preprocessing)
        detected = self.detector.detect(gray, estimate_tag_pose=True,
                   camera_params=intrinsics, tag_size=self.c['tag_size_m'])
        result = {'time_ns': int(message['time_stamp']), 'intrinsics': intrinsics,
                  'tag_size_m': self.c['tag_size_m'], 'family': self.c['tag_family'],
                  'visible': False, 'detections': [], 'selected': None,
                  'preprocessing':preprocessing,
                  'preprocessing_parameters':{'clip_limit':2.0,'tile_grid':[8,8]} if preprocessing=='clahe' else {}}
        cp, cr = camera_pose(message)
        candidates = []
        for d in detected:
            t = np.asarray(d.pose_t).reshape(3)
            item = {'id': int(d.tag_id), 'corners': d.corners.tolist(), 'center': d.center.tolist(),
                    'pose_t_optical_m': t.tolist(), 'pose_R_optical': d.pose_R.tolist(),
                    'decision_margin': float(d.decision_margin), 'hamming': int(d.hamming),
                    'pose_error': float(d.pose_err),
                    'world_position_ned': (cp+cr @ t[[2, 0, 1]]).tolist()}
            result['detections'].append(item)
            if (item['id'] in self.c['tag_ids'] and item['decision_margin'] >= self.c['tag_min_margin']
                and item['hamming'] <= self.c['tag_max_hamming'] and t[2] > 0 and np.isfinite(t).all()):
                candidates.append(item)
        if candidates:
            result['selected'] = max(candidates, key=lambda d: d['decision_margin'])
            result['visible'] = True
        return result


def annotate(image, detection, phase):
    out = image.copy()
    for d in detection['detections']:
        corners = np.round(d['corners']).astype(np.int32)
        selected=detection['selected'] is not None and d['id']==detection['selected']['id']
        cv2.polylines(out, [corners], True, (40, 220, 40) if selected else (170,170,170), 2, cv2.LINE_AA)
        if not selected:continue
        center = tuple(np.round(d['center']).astype(int))
        cv2.drawMarker(out, center, (0, 230, 255), cv2.MARKER_CROSS, 16, 2)
    cv2.rectangle(out, (0, 0), (out.shape[1], 45), (25, 25, 25), -1)
    label = f"{phase} | sim {detection['time_ns']/1e9:.2f}s | AprilTag {'detected' if detection['visible'] else 'not detected'}"
    cv2.putText(out, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2, cv2.LINE_AA)
    if detection['selected']:
        d=detection['selected']
        cv2.putText(out,f"ID {d['id']} | margin {d['decision_margin']:.0f} | camera Z {d['pose_t_optical_m'][2]:.2f} m",
                    (15,75),cv2.FONT_HERSHEY_SIMPLEX,.7,(30,255,80),2,cv2.LINE_AA)
    return out
