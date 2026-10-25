"""Assemble synchronized recorded cameras with explicit pose/recognition annotations."""
from __future__ import annotations
import argparse
import bisect
import csv
import gzip
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

from record_water_landing import Encoder, sha, ROOT
from ag.math3d import camera_pose, pose_parts
from ag.storage import write_json

TITLES = {'mapping': 'Air-ground collaborative 3-D mapping',
          'navigation': 'Aerial-assisted multi-junction road navigation',
          'tracking': 'Multi-UAV tracking and UGV exit containment',
          'landing': 'UAV landing on a moving USV'}
COLORS = {'Drone1': (0, 190, 245), 'Drone2': (220, 190, 25), 'Car1': (235, 150, 40),
          'Target': (65, 65, 235), 'GateLeft': (100, 210, 30), 'GateRight': (230, 200, 30),
          'Vessel1': (175, 195, 35)}
LABELS = {'Drone1': 'UAV 1', 'Drone2': 'UAV 2', 'Car1': 'UGV', 'Target': 'Target',
          'GateLeft': 'UGV L', 'GateRight': 'UGV R', 'Vessel1': 'USV'}


def text(image, message, xy, scale=.55, color=(245, 245, 245)):
    size, baseline = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = xy
    cv2.rectangle(image, (max(0, x-3), max(0, y-size[1]-4)),
                  (min(image.shape[1]-1, x+size[0]+4), min(image.shape[0]-1, y+baseline+3)), (25, 25, 25), -1)
    cv2.putText(image, message, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


class Camera:
    def __init__(self, folder, entry):
        self.path = folder / entry['file']
        self.cap = cv2.VideoCapture(str(self.path))
        self.entry = entry
        self.rows = []
        with gzip.open(self.path.parent/'capture_metadata.jsonl.gz', 'rt', encoding='utf-8') as f:
            self.rows = [json.loads(line) for line in f]
        self.times = np.asarray([r['source_time_ns']/1e9 for r in self.rows])
        self.metadata_by_stamp = {r['source_time_ns']: r for r in self.rows}
        with (self.path.parent/'video_frame_index.csv').open(newline='', encoding='utf-8') as stream:
            self.source_stamps = [int(r['source_time_ns']) for r in csv.DictReader(stream)]
        if len(self.source_stamps) != entry['frames']:
            raise RuntimeError('Camera frame index length mismatch')
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.i, self.frame = -1, None

    def at(self, now):
        target = max(0, min(int(round((now-self.entry['start_time_ns']/1e9)*self.fps)), self.entry['frames']-1))
        if target < self.i:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, target)
            self.i = target-1
        while self.i < target:
            ok, frame = self.cap.read()
            if not ok:
                break
            self.frame, self.i = frame, self.i+1
        stamp = self.source_stamps[self.i]
        return self.frame.copy(), self.metadata_by_stamp[stamp]


def pose_labels(frame, record, task, fov, tracks):
    pos, rot = camera_pose(record['capture'])
    h, w = frame.shape[:2]
    f = w / (2*math.tan(math.radians(fov)/2))
    for name, track in tracks.items():
        if name not in LABELS:
            continue
        at = record['source_time_ns']/1e9
        i = int(np.argmin(np.abs(track[:, 0]-at)))
        if abs(track[i, 0]-at) > .6:
            continue
        p = track[i, 1:4].copy()
        p[2] -= .3 if not name.startswith('Drone') else 0
        local = rot.T @ (p-pos)
        if local[0] <= .2:
            continue
        u, v = w/2+f*local[1]/local[0], h/2+f*local[2]/local[0]
        if not (12 <= u < w-12 and 40 <= v < h-12):
            continue
        color = COLORS[name]
        cv2.circle(frame, (round(u), round(v)), 5, color, 1, cv2.LINE_AA)
        cv2.line(frame, (round(u+5), round(v-5)), (round(u+14), round(v-20)), color, 1, cv2.LINE_AA)
        text(frame, LABELS[name], (min(round(u+15), w-90), max(25, round(v-22))), .48, color)
    return frame


def sensor_boxes(frame, record):
    for name, box in record.get('segmentation_boxes', {}).items():
        if name not in {'Target', 'UAV', 'UGV', 'USV', 'UGV L', 'UGV R'}:
            continue
        x0, y0, x1, y1 = box
        if (x1-x0)*(y1-y0) > frame.shape[0]*frame.shape[1]*.35:
            continue
        cv2.rectangle(frame, (x0, y0), (x1, y1), (70, 240, 100), 1, cv2.LINE_AA)
        text(frame, name, (x0, max(20, y0-6)), .5, (70, 240, 100))
    return frame


def planned_route(frame, record, plan, fov):
    if not plan or record['source_time_ns']/1e9 < plan['committed_sim_time_sec']:
        return frame
    pos, rot = camera_pose(record['capture'])
    points = np.asarray(plan['waypoints_ned'])
    local = (points-pos) @ rot
    h, w = frame.shape[:2]
    f = w/(2*math.tan(math.radians(fov)/2))
    for a, b in zip(local[:-1], local[1:]):
        if min(a[0], b[0]) <= 1:
            continue
        aa = (round(w/2+f*a[1]/a[0]), round(h/2+f*a[2]/a[0]))
        bb = (round(w/2+f*b[1]/b[0]), round(h/2+f*b[2]/b[0]))
        visible, aa, bb = cv2.clipLine((0, 0, w, h), aa, bb)
        if visible:
            cv2.line(frame, aa, bb, (55, 210, 95), 1, cv2.LINE_AA)
    return frame


def fit(image, width, height):
    h, w = image.shape[:2]
    scale = min(width/w, height/h)
    out = np.full((height, width, 3), 15, dtype=np.uint8)
    resized = cv2.resize(image, (round(w*scale), round(h*scale)), interpolation=cv2.INTER_AREA)
    y, x = (height-resized.shape[0])//2, (width-resized.shape[1])//2
    out[y:y+resized.shape[0], x:x+resized.shape[1]] = resized
    return out


def load_telemetry(experiment, task):
    if task == 'landing':
        rows = list(csv.DictReader((experiment/'trajectory.csv').open(encoding='utf-8')))
        tracks = {}
        for name, prefix in [('Drone1', 'uav'), ('Vessel1', 'usv')]:
            tracks[name] = np.array([[float(r['sim_time_sec']), float(r[prefix+'_x']), float(r[prefix+'_y']), float(r[prefix+'_z'])] for r in rows])
        phase = [(float(r['sim_time_sec']), r['phase'], r) for r in rows]
    else:
        path = experiment/('trajectories.csv' if task == 'mapping' else 'trajectories_controls.csv')
        rows = list(csv.DictReader(path.open(encoding='utf-8')))
        tracks = {}
        time_key, name_key = ('time', 'name') if task == 'mapping' else ('sim_time_sec', 'robot')
        for r in rows:
            tracks.setdefault(r[name_key], []).append([float(r[time_key]), float(r['x']), float(r['y']), float(r['z'])])
        tracks = {name: np.array(points) for name, points in tracks.items()}
        phase = [(float(r[time_key]), r.get('phase', 'mapping orbit'), r) for r in rows]
    phase.sort(key=lambda x: x[0])
    return tracks, phase


def minimap(tracks, now):
    out = np.full((195, 440, 3), (34, 32, 28), dtype=np.uint8)
    points = np.concatenate([t[:, 1:3] for t in tracks.values()])
    lo, hi = points.min(axis=0)-3, points.max(axis=0)+3
    scale = min(380/(hi[1]-lo[1]), 145/(hi[0]-lo[0]))
    def project(p):
        return np.rint(np.c_[35+(p[:, 1]-lo[1])*scale, 170-(p[:, 0]-lo[0])*scale]).astype(np.int32)
    for name, track in tracks.items():
        active = track[track[:, 0] <= now]
        if len(active):
            xy = project(active[:, 1:3])
            cv2.polylines(out, [xy], False, COLORS.get(name, (200, 200, 200)), 2, cv2.LINE_AA)
            cv2.circle(out, tuple(xy[-1]), 4, COLORS.get(name, (200, 200, 200)), -1)
    text(out, 'Recorded trajectories | East / North', (12, 21), .5)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('task', choices=list(TITLES))
    parser.add_argument('folder', type=Path)
    parser.add_argument('--speed', type=float, default=1)
    args = parser.parse_args()
    root = args.folder
    original = root/'camera_originals'
    manifest = json.loads((original/'recording_manifest.json').read_text())
    cameras = {key: Camera(original, entry) for key, entry in manifest['cameras'].items()}
    experiment = root/'experiment'
    if (root/'experiment_link.json').exists():
        experiment = Path(json.loads((root/'experiment_link.json').read_text())['path'])
    tracks, phases = load_telemetry(experiment, args.task)
    car_phases = [r for r in phases if r[2].get('robot') == 'Car1']
    car_phase_times = [r[0] for r in car_phases]
    plan = (json.loads((experiment/'committed_plan.json').read_text())
            if args.task == 'navigation' and (experiment/'committed_plan.json').exists() else None)
    tracking_rows = []
    if args.task == 'tracking':
        tracking_rows = list(csv.DictReader((experiment/'tracking.csv').open(encoding='utf-8')))
        tracking_times = [float(r['sim_time_sec']) for r in tracking_rows]
    start = min(t[0, 0] for t in tracks.values())
    end = max(t[-1, 0] for t in tracks.values())
    initial_start = start
    if args.task == 'landing':
        # Begin with the first exposure at the fixed observer pose; raw pretrial frames remain retained.
        fixed = [m['source_time_ns']/1e9 for m in cameras['overview'].rows
                 if np.allclose([m['capture']['pos_'+a] for a in 'xyz'], [0., -12., -22.], atol=1e-4)]
        if not fixed:
            raise RuntimeError('No recorded exposure at the requested fixed observer position')
        start = max(start, fixed[0])
    if args.task == 'landing':
        main_key, side_keys = 'overview', ['down']
    elif args.task == 'mapping':
        main_key, side_keys = 'Observer/Overview/scene_camera', ['Drone1/scene_camera', 'Car1/scene_camera']
    elif args.task == 'navigation':
        main_key = 'Observer/VideoFollow/scene_camera'
        side_keys = ['Observer/Overview/scene_camera', 'Drone1/scene_camera', 'Car1/scene_camera']
    else:
        main_key = 'Observer/VideoFollow/scene_camera'
        side_keys = ['Observer/Overview/scene_camera', 'Drone1/scene_camera', 'GateRight/scene_camera']
    side_keys = [key for key in side_keys if key in cameras]
    name = ('presentation.mp4' if args.speed == 1 else f'presentation_{args.speed:g}x.mp4')
    encoder = Encoder(root/name, 1600, 900, 30)
    count = math.ceil((end-start)/args.speed*30)
    phase_times = [x[0] for x in phases]
    detector = None
    if args.task == 'landing':
        from ag.landing_vision import DeckDetector, annotate
        detector = DeckDetector(json.loads((experiment/'config.json').read_text()))
    previews = {round(count*p): f'preview_{i}.jpg' for i, p in enumerate([.1, .35, .65, .9], 1)}
    for i in range(count):
        now = start+i/30*args.speed
        canvas = np.full((900, 1600, 3), (26, 23, 20), dtype=np.uint8)
        frame, record = cameras[main_key].at(now)
        fov = 65 if args.task == 'landing' else 75 if 'VideoFollow' in main_key else 80
        frame = planned_route(frame, record, plan, fov)
        frame = pose_labels(frame, record, args.task, fov, tracks)
        canvas[:675, :1200] = fit(frame, 1200, 675)
        text(canvas, TITLES[args.task], (20, 33), .85)
        text(canvas, 'Observer RGB | pose-projected labels', (20, 62), .52)
        phase_i = max(0, bisect.bisect_right(phase_times, now)-1)
        phase, row = phases[phase_i][1:]
        for j, key in enumerate(side_keys):
            image, meta = cameras[key].at(now)
            label = key.replace('/scene_camera', '').replace('Observer/', '')
            if key == 'down' and detector:
                result = detector.detect(dict(meta['capture'], data=b''), image)
                image = annotate(image, result, phase)
                label = 'UAV RGB | AprilTag deck detection'
                if result['selected']:
                    corners = np.asarray(result['selected']['corners'])
                    center = corners.mean(axis=0)
                    half = max(80, np.ptp(corners, axis=0).max()*2.5)
                    x0, y0 = np.maximum(0, center-half).astype(int)
                    x1, y1 = np.minimum([image.shape[1], image.shape[0]], center+half).astype(int)
                    crop = image[y0:y1, x0:x1]
                    zoom = fit(crop, 400, 300)
                    text(zoom, 'Deck RGB crop | measured marker', (8, 22), .44)
                    canvas[325:625, 1200:1600] = zoom
            elif args.task == 'tracking' and not key.startswith('Observer'):
                image = sensor_boxes(image, meta)
                label += ' | segmentation target box'
            elif key.startswith('Observer'):
                image = planned_route(image, meta, plan, 75 if 'VideoFollow' in key else 80)
                image = pose_labels(image, meta, args.task, 75 if 'VideoFollow' in key else 80, tracks)
            elif args.task == 'navigation' and key.startswith('Drone'):
                image = planned_route(image, meta, plan, 90)
            tile = fit(image, 400, 225)
            text(tile, label, (8, 20), .44)
            canvas[j*225:(j+1)*225, 1200:1600] = tile
        canvas[692:887, 20:460] = minimap(tracks, now)
        text(canvas, f'Simulation time {now:.1f}s  |  elapsed {now-start:.1f}s  |  playback {args.speed:g}x', (485, 723), .67)
        text(canvas, 'Phase: '+phase.replace('_', ' '), (485, 759), .67)
        if args.task == 'landing':
            text(canvas, f"Horizontal error {float(row['horizontal_error_m']):.3f} m  |  USV speed {float(row['boat_speed_mps']):.2f} m/s", (485, 795), .63)
            text(canvas, f"Rotor thrust {float(row['rotor_total_thrust_n']):.3f} N  |  supported dwell {float(row['dwell_sec']):.1f} s", (485, 831), .63)
        else:
            offset = 0
            for robot in tracks:
                text(canvas, LABELS.get(robot, robot), (485+offset, 799), .6, COLORS.get(robot, (200, 200, 200)))
                offset += 145
            if args.task == 'navigation':
                active = tracks['Car1'][tracks['Car1'][:, 0] <= now]
                distance = np.linalg.norm(np.diff(active[:, 1:3], axis=0), axis=1).sum()
                car_row = car_phases[max(0, bisect.bisect_right(car_phase_times, now)-1)][2]
                deviation = car_row.get('cross_track_m')
                text(canvas, f'UGV travelled {distance:.1f} m | route deviation {float(deviation or 0):.2f} m', (485, 835), .6)
            if args.task == 'tracking' and tracking_rows:
                tr = tracking_rows[max(0, bisect.bisect_right(tracking_times, now)-1)]
                error = tr.get('error_m_evaluation_only')
                text(canvas, 'Target estimation error: '+(f'{float(error):.2f} m' if error else 'no current observation'), (485, 835), .6)
        text(canvas, 'Same-run camera topics and trajectories; camera frames held between actual exposures.', (485, 871), .47)
        encoder.write(canvas)
        if i in previews:
            cv2.imwrite(str(root/previews[i]), canvas)
        if i % 500 == 0:
            print(f'COMPOSE {i}/{count}', flush=True)
    end_card = None
    if args.task == 'mapping':
        results = experiment/'plaza_results'
        paths = [results/'pointcloud_structure_render.png', results/'colored_surface_render.png']
        quality = json.loads((results/'surface'/'quality.json').read_text())
        images = [cv2.imread(str(path)) for path in paths]
        if any(image is None for image in images):
            raise RuntimeError('Same-run mapping result images are missing')
        card = np.full((900, 1600, 3), (26, 23, 20), dtype=np.uint8)
        text(card, '3-D reconstruction from this recorded run', (30, 45), .9)
        for j, (image, label) in enumerate(zip(images, ['Measured LiDAR | structural display', 'RGB-D fusion | colored surface'])):
            card[95:735, j*800:(j+1)*800] = fit(image, 800, 640)
            text(card, label, (30+j*800, 785), .7)
        text(card, 'Same-run raw LiDAR, RGB-D, sensor poses and reconstruction files are retained.', (30, 837), .65)
        text(card, 'Point-cloud support filtering and shading are display processing; no synthetic geometry added.', (30, 875), .55)
        for _ in range(80):
            encoder.write(card)
        cv2.imwrite(str(root/'preview_results.jpg'), card)
        end_card = {'duration_sec': 8, 'frames': 80,
                    'sources': [{'path': str(path), 'sha256': sha(path)} for path in paths],
                    'surface_quality': quality}
    encoder.close()
    for camera in cameras.values():
        camera.cap.release()
    write_json(root/name.replace('.mp4', '_manifest.json'), {
        'video': name, 'sha256': sha(root/name), 'experiment_source': str(experiment),
        'simulation_interval_sec': [start, end], 'pre_fixed_camera_initialization_omitted_sec': start-initial_start, 'playback_speed': args.speed,
        'frames': count + (80 if end_card else 0), 'output_fps': 30, 'main_camera': main_key, 'inset_cameras': side_keys,
        'same_run_result_end_card': end_card,
        'annotations': 'Observer labels projected from nearest same-run trajectory sample (max0.6s) using recorded exposure camera pose; onboard target boxes from timestamp-paired segmentation; landing marker boxes rerun through real AprilTag detector',
        'no_interpolation': True, 'raw_recording_manifest': str(original/'recording_manifest.json')})
    print(root/name, flush=True)


if __name__ == '__main__':
    main()
