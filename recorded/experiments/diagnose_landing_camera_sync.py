"""Offline static-tag reprojection check; never supplies truth to flight control."""
import argparse
import json
from pathlib import Path
import numpy as np
from ag.math3d import camera_pose
from ag.landing_core import deck_position
from ag.storage import write_json


def diagnose(trial, start=15.):
    config = json.loads((trial/'config.json').read_text(encoding='utf-8'))
    if config['desired_speed_mps'] != 0:
        raise ValueError('This diagnostic requires a stationary vessel trial')
    state = json.loads((trial/'initial_state.json').read_text(encoding='utf-8'))[config['usv']]
    target = deck_position([state['translation'][a] for a in 'xyz'],
                           state['rotation'], config['tag_offset_hull_m'])
    poses, observations = [], []
    for path in sorted((trial/'frames/down').glob('*.json'), key=lambda p: int(p.stem)):
        detection = json.loads(path.read_text(encoding='utf-8'))
        with np.load(path.with_suffix('.npz'), allow_pickle=False) as data:
            metadata = json.loads(str(data['metadata_json']))
        position, rotation = camera_pose(metadata)
        t = int(path.stem)/1e9
        poses.append((t, position, rotation))
        if detection['selected'] and t >= start:
            selected = detection['selected']
            observations.append((t, np.array(selected['pose_t_optical_m'])[[2, 0, 1]],
                                 np.array(selected['center']), detection['intrinsics']))
    timestamps = np.array([p[0] for p in poses])
    rows = []
    for shift in [-.3, -.2, -.1, 0., .1, .2, .3]:
        world_errors, pixels, offsets = [], [], []
        for t, translation, center, intrinsics in observations:
            j = np.abs(timestamps-(t+shift)).argmin()
            pt, position, rotation = poses[j]
            if abs(pt-(t+shift)) > .065:
                continue
            local = rotation.T @ (target-position)
            if local[0] <= 0:
                continue
            fx, fy, cx, cy = intrinsics
            projected = np.array([cx, cy])+np.array([fx, fy])*local[1:]/local[0]
            world_errors.append(np.linalg.norm(position+rotation@translation-target))
            pixels.append(np.linalg.norm(projected-center))
            offsets.append(pt-t)
        rows.append({'pose_time_shift_sec': shift, 'samples': len(pixels),
                     'actual_shift_median_sec': float(np.median(offsets)) if offsets else None,
                     'reprojection_median_p90_pixels': np.quantile(pixels, [.5, .9]).tolist() if pixels else None,
                     'position_median_p90_m': np.quantile(world_errors, [.5, .9]).tolist() if world_errors else None})
    result = {'trial': str(trial.resolve()), 'stationary_tag_truth_ned': target.tolist(),
              'minimum_sim_time_sec': start, 'observations': len(observations), 'shifts': rows,
              'purpose': 'Diagnostic only. Truth and shifted poses are not used by controller or edited into raw evidence.'}
    write_json(trial/'derived/camera_sync_diagnosis.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--start', type=float, default=15.)
    args = parser.parse_args()
    print(json.dumps(diagnose(args.trial, args.start), indent=2))
