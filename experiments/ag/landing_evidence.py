"""Lossless consumed sensor frames, raw topics, commands, and immutable receipts."""
import csv
import gzip
import hashlib
import json
from pathlib import Path
import threading
from datetime import datetime, timezone
import cv2
import numpy as np
from .storage import ROOT, serial, write_json
from .landing_vision import annotate


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(2**20), b''):
            h.update(chunk)
    return h.hexdigest()


class Evidence:
    def __init__(self, path, config, trial):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        self.config = config
        self.streams = {}
        self.frames = set()
        self.lock = threading.Lock()
        self.topic_errors = []
        self.raw = gzip.open(self.path/'raw_topics.jsonl.gz', 'wt', encoding='utf-8')
        self.manifest = {'schema': 1, 'status': 'running', 'backend': 'projectairsim',
                         'start_utc': datetime.now(timezone.utc).isoformat(), 'trial': trial,
                         'provenance': config['provenance'], 'protocol': 'moving-deck-contact-v2-rotors' if config.get('require_rotor_verification') else 'moving-deck-contact-v1',
                         'coordinate_system': 'NED metres, quaternion wxyz, simulation timestamps in ns',
                         'image_selection': 'All images consumed by controller, overview at configured interval; not every camera exposure',
                         'synchronization': 'Each image carries its own exposure pose/time; state RPC start/end times recorded, no synthetic simultaneity'}
        write_json(self.path/'config.json', config)
        sources = list((ROOT/'ag').glob('*.py')) + list(ROOT.glob('*landing*.py'))
        sources += list(Path(config['sim_config_path']).glob('*'))
        sources += [Path(p) for p in config['provenance']['reference_programs']]
        if config.get('calibration_file'):sources.append(Path(config['calibration_file']))
        physics_root = None
        if config.get('physics_build_receipt'):
            receipt_path=Path(config['physics_build_receipt'])
            physics_root=receipt_path.parent
            receipt=json.loads(receipt_path.read_text(encoding='utf-8'))
            for filename, expected in receipt['binaries'].items():
                if digest(filename) != expected:
                    raise RuntimeError('Physics binary changed after build receipt: '+filename)
            for filename, expected in receipt['sources'].items():
                if digest(filename) != expected:
                    raise RuntimeError('Physics source changed after build receipt: '+filename)
            sources.append(receipt_path)
            sources.extend(Path(p) for p in receipt['sources'])
            runtime_path=physics_root/'runtime_receipt.json'
            if not runtime_path.exists():
                raise RuntimeError('Independent simulator runtime verification is missing')
            runtime=json.loads(runtime_path.read_text(encoding='utf-8'))
            import psutil
            process=psutil.Process(runtime['pid'])
            if abs(process.create_time()-runtime['create_time'])>.001:
                raise RuntimeError('Verified simulator PID was reused')
            if digest(runtime['loaded_plugin'])!=runtime['plugin_sha256']:
                raise RuntimeError('Verified runtime plugin changed')
            sources.append(runtime_path)
        hashes = {}
        for source in sources:
            if not source.is_file():
                continue
            relative = source.relative_to(ROOT) if source.is_relative_to(ROOT) else Path('references')/source.name
            if physics_root and source.is_relative_to(physics_root):
                relative=Path('physics_build')/source.relative_to(physics_root)
            target = self.path/'source_snapshot'/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
            hashes[relative.as_posix()] = digest(target)
        write_json(self.path/'source_sha256.json', hashes)
        write_json(self.path/'manifest.json', self.manifest)

    def topic(self, key, message):
        if key!='rotor' and not key.endswith(('/pose', '/collision')):
            return
        try:
            with self.lock:
                self.raw.write(json.dumps(serial({'topic': key, 'message': message}), allow_nan=False)+'\n')
        except Exception as exc:
            self.topic_errors.append(str(exc))

    def row(self, name, record):
        if name not in self.streams:
            f = (self.path/(name+'.csv')).open('w', newline='', encoding='utf-8')
            writer = csv.DictWriter(f, fieldnames=list(record))
            writer.writeheader()
            self.streams[name] = f, writer
        f, writer = self.streams[name]
        writer.writerow(serial(record))
        f.flush()

    def frame(self, camera, message, image, detection=None, phase=''):
        stamp = int(message['time_stamp'])
        key = camera, stamp
        if key in self.frames:
            return f'frames/{camera}/{stamp}.png'
        folder = self.path/'frames'/camera
        folder.mkdir(parents=True, exist_ok=True)
        metadata = {k:v for k,v in message.items() if k != 'data'}
        # Original encoded sensor bytes and metadata can reproduce the decoded PNG.
        np.savez(folder/f'{stamp}.npz', payload=np.frombuffer(bytes(message['data']), dtype='u1'),
                 metadata_json=np.array(json.dumps(serial(metadata), allow_nan=False)))
        if not cv2.imwrite(str(folder/f'{stamp}.png'), image):
            raise OSError('Failed to save raw RGB')
        if detection is not None:
            write_json(folder/f'{stamp}.json', detection)
            if not cv2.imwrite(str(folder/f'{stamp}_annotated.png'), annotate(image, detection, phase)):
                raise OSError('Failed to save annotated RGB')
        self.row('frames', {'camera':camera, 'time_ns':stamp, 'phase':phase,
                            'raw_image':f'frames/{camera}/{stamp}.png',
                            'payload':f'frames/{camera}/{stamp}.npz',
                            'detection':f'frames/{camera}/{stamp}.json' if detection is not None else '',
                            'visible':int(detection['visible']) if detection is not None else ''})
        self.frames.add(key)
        return f'frames/{camera}/{stamp}.png'

    def close(self, result):
        with self.lock:
            self.raw.close()
        for f, _ in self.streams.values():
            f.close()
        self.manifest.update(result)
        self.manifest['end_utc'] = datetime.now(timezone.utc).isoformat()
        self.manifest['topic_recording_errors'] = self.topic_errors
        if self.topic_errors:
            self.manifest.update(status='error', success=False, reason='raw_topic_recording_failed')
        write_json(self.path/'manifest.json', self.manifest)
        files = {p.relative_to(self.path).as_posix(): digest(p)
                 for p in sorted(self.path.rglob('*')) if p.is_file()}
        write_json(self.path/'evidence_sha256.json', files)
        write_json(self.path/'receipt.json', {'algorithm': 'SHA256', 'files': len(files),
             'manifest_sha256': digest(self.path/'evidence_sha256.json'),
             'note': 'Integrity receipt, not independent proof of origin or experimental validity'})
