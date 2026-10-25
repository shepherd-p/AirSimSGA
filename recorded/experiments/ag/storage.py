import csv
import hashlib
import json
from datetime import datetime
from pathlib import Path
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def serial(value):
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v) for v in value]
    if isinstance(value, np.ndarray):
        return serial(value.tolist())
    if isinstance(value, np.generic):
        return serial(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(serial(value), indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def verify_originals():
    baseline = json.loads((ROOT/'verification/protected_files.json').read_text(encoding='utf-8'))
    changed = [p for p, digest in baseline.items()
               if not Path(p).is_file() or hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest]
    return {'checked_files': len(baseline), 'changed_files': changed, 'unchanged': not changed}


class Run:
    def __init__(self, config, output=None):
        self.path = (Path(output) if output else ROOT/'outputs')/config['task']/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.path.mkdir(parents=True)
        write_json(self.path/'config.json', config)
        sources = {}
        for source in list((ROOT/'ag').glob('*.py'))+list((ROOT/'ag').glob('*.html'))+list(ROOT.glob('*.py')):
            relative = source.relative_to(ROOT)
            target = self.path/'source_snapshot'/relative
            target.parent.mkdir(parents=True,exist_ok=True)
            content = source.read_bytes()
            target.write_bytes(content)
            sources[str(relative)] = hashlib.sha256(content).hexdigest()
        write_json(self.path/'source_sha256.json',sources)
        self.files = {}
        self.manifest = {'status': 'running', 'backend': 'projectairsim',
                         'data_source': 'live sensor topics and services', 'task': config['task'],
                         'ground_truth_usage': 'ego localization, mapping capture pose, evaluation; target truth excluded from tracking control',
                         'perception': ('simulator segmentation + perspective depth baseline' if config['task']=='tracking'
                                        else config['navigation'].get('map_source','uav_lidar') if config['task']=='navigation'
                                        else 'LiDAR and camera measurement streams'),
                         'start_utc': datetime.utcnow().isoformat()+'Z'}
        write_json(self.path/'manifest.json', self.manifest)

    def row(self, name, record):
        if name not in self.files:
            f = (self.path/f'{name}.csv').open('w', newline='', encoding='utf-8')
            writer = csv.DictWriter(f, fieldnames=list(record))
            writer.writeheader()
            self.files[name] = f, writer
        f, writer = self.files[name]
        writer.writerow(serial(record))
        f.flush()

    def json(self, name, value):
        write_json(self.path/name, value)

    def finish(self, status, **extra):
        self.manifest.update(status=status, end_utc=datetime.utcnow().isoformat()+'Z', **extra)
        self.json('manifest.json', self.manifest)
        for f, _ in self.files.values():
            f.close()


def save_pcd(path, points):
    p = np.asarray(points).reshape(-1,3)
    with Path(path).open('w', encoding='ascii') as f:
        f.write(f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nPOINTS {len(p)}\nDATA ascii\n')
        np.savetxt(f, p, fmt='%.5f')
