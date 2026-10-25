"""Check same-run pursuit evidence before delivering a recorded video."""
import argparse
import csv
import gzip
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from record_experiment import ROOT, sha, write_json


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=Path)
    folder = parser.parse_args().folder
    run = Path(json.loads((folder/'experiment_link.json').read_text())['path'])
    summary = json.loads((run/'summary.json').read_text())
    cfg = json.loads((run/'config.json').read_text())
    require(summary['status'] == 'completed' and summary['error'] is None,
            'Pursuit did not complete')
    require(not summary['collision_events'] and summary['target_blocked'],
            'Collision or unconfirmed target stopping')
    ids = json.loads((run/'segmentation_ids.json').read_text())
    for name, identifier in [('Target',210), ('GateLeft',211), ('GateRight',212)]:
        require([k for k,v in ids.items() if v == identifier] == [name],
                'Background shares a reserved vehicle segmentation ID')
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    brakes = next(e for e in events if e['kind'] == 'target_brakes_for_observed_blocker')
    contained = next(e for e in events if e['kind'] == 'contained')
    require(contained['both_gates_occupied'] and contained['both_uavs_see_target'],
            'Missing multi-platform completion conditions')
    require(contained['sim_time_sec']-brakes['sim_time_sec'] >= cfg['capture']['dwell_sec'],
            'Insufficient time after observed braking')
    rows = list(csv.DictReader((run/'trajectories_controls.csv').open(encoding='utf-8')))
    target = np.asarray([[float(r[k]) for k in ['sim_time_sec','x','y','z','speed_mps']]
                         for r in rows if r['robot'] == 'Target'])
    speed = np.linalg.norm(np.diff(target[:, 1:3], axis=0), axis=1)/np.diff(target[:, 0])
    require(np.isfinite(speed).all() and speed.max() < 6,
            'Target trajectory contains an implausible speed spike')
    require(target[-1, 4] < cfg['capture']['target_stop_speed_mps'], 'Target did not stop')
    tracking = list(csv.DictReader((run/'tracking.csv').open(encoding='utf-8')))
    errors = [float(r['error_m_evaluation_only']) for r in tracking if r['error_m_evaluation_only']]
    rmse = float(np.sqrt(np.mean(np.square(errors))))
    require(abs(rmse-summary['tracking_rmse_m']) < 1e-8, 'Tracking RMSE mismatch')
    provenance = json.loads((run/'provenance.json').read_text())
    for name, digest in provenance['source_sha256'].items():
        require(sha(run/'source_snapshot'/name) == digest, 'Changed source snapshot: '+name)
    sys.path.insert(0, cfg['sdk_path'])
    from projectairsim.utils import unpack_image
    from projectairsim.image_utils import segmentation_id_to_color
    verified = 0
    for capture in summary['captures']:
        with np.load(capture['payload']) as packet:
            payload = packet['payload'].tobytes()
            metadata = json.loads(packet['metadata_json'].item())
        import hashlib
        require(hashlib.sha256(payload).hexdigest() == capture['payload_sha256'], 'RGB payload hash mismatch')
        image = cv2.imread(capture['png'])
        require(np.array_equal(image, unpack_image({**metadata, 'data': payload})), 'RGB PNG differs from source')
        verified += 1
    with np.load(run/'blocker_seen_raw.npz') as packet:
        metadata = json.loads(packet['metadata_json'].item())
        require(abs(metadata['depth']['time_stamp']-metadata['segmentation']['time_stamp']) <= 60_000_000,
                'Blocker depth and segmentation are not synchronized')
        for leaf in ['depth', 'segmentation']:
            require(unpack_image({**metadata[leaf], 'data': packet[leaf+'_payload'].tobytes()}).size > 0,
                    'Missing blocker image')
    view_checks = {}
    color = segmentation_id_to_color(210)
    for view in ['Drone1_scene_camera','GateRight_scene_camera']:
        directory = folder/'camera_originals'/view
        with gzip.open(directory/'capture_metadata.jsonl.gz', 'rt') as stream:
            frames = [json.loads(line) for line in stream]
        boxes = [frame for frame in frames if 'Target' in frame['segmentation_boxes']]
        require(len(boxes) >= 10, 'Insufficient target boxes in '+view)
        replayed = 0
        for frame in boxes:
            path = directory/f"segmentation_{frame['source_time_ns']}.npz"
            if not path.exists():
                continue
            with np.load(path) as packet:
                metadata = json.loads(packet['metadata_json'].item())
                image = unpack_image({**metadata, 'data': packet['payload'].tobytes()})
            yy, xx = np.nonzero(np.all(image[:, :, :3] == [color.b,color.g,color.r], axis=2))
            require(len(xx) >= 5 and frame['segmentation_boxes']['Target'] ==
                    [int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())],
                    'Target box does not reproduce from raw segmentation: '+view)
            require(abs(metadata['time_stamp']-frame['source_time_ns']) <= 50_000_000,
                    'RGB and segmentation are not paired')
            replayed += 1
            if replayed == 3:
                break
        require(replayed == 3, 'Missing raw segmentation samples: '+view)
        view_checks[view] = {'target_box_frames': len(boxes), 'raw_segmentation_boxes_replayed': replayed}
    report = {'status': 'verified', 'experiment': str(run), 'summary_sha256': sha(run/'summary.json'),
              'rgb_png_payload_pairs_verified': verified, 'blocker_depth_segmentation_replay': True,
              'source_snapshots_verified': len(provenance['source_sha256']), 'tracking_rmse_m': rmse,
              'onboard_target_box_checks': view_checks,
              'max_sampled_target_speed_mps': float(speed.max()),
              'target_stopped': True, 'both_gates_occupied': True, 'both_uavs_see_target': True,
              'collision_events': 0,
              'scope': 'Internal consistency of same-run recordings; not independent third-party validation'}
    write_json(folder/'audit_report.json', report)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
