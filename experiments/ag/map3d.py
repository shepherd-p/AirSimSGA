"""Dependency-free offline WebGL viewer for measured XYZ point clouds."""
import json
from pathlib import Path
import numpy as np


def read_pcd(path):
    with Path(path).open(encoding='ascii') as f:
        header = {}
        for line in f:
            parts = line.strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            header[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        if header.get('DATA') != ['ascii'] or header.get('FIELDS') != ['x', 'y', 'z']:
            raise ValueError(f'Expected ASCII XYZ PCD: {path}')
        count = int(header['POINTS'][0])
        points = np.loadtxt(f).reshape(-1, 3) if count else np.empty((0, 3))
    if len(points) != count or not np.isfinite(points).all():
        raise ValueError(f'Invalid point count or non-finite coordinates: {path}')
    return points


def build_viewer(folder):
    folder = Path(folder).resolve()
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    if manifest['task'] != 'mapping' or manifest['status'] == 'running':
        raise ValueError('Select a finished mapping run')
    layers = []
    for filename, label in [('map_fused.pcd', '融合地图'),
                            ('map_fused_unregistered.pcd', '未配准合并图'),
                            ('map_Drone1.pcd', '无人机地图'), ('map_Car1.pcd', '地面车地图')]:
        path = folder/filename
        if path.exists():
            points = read_pcd(path)
            if len(points):
                # Display North/East/Up; PCD remains in the original NED convention.
                display = points*np.array([1., 1., -1.])
                layers.append({'name': label, 'file': filename, 'count': len(points),
                               'points': display.ravel().tolist()})
    if not layers:
        raise ValueError('No non-empty map PCD files in this run')
    data = {'run': folder.name, 'status': manifest['status'], 'layers': layers}
    template = Path(__file__).with_name('map3d_template.html').read_text(encoding='utf-8')
    payload = json.dumps(data, ensure_ascii=False, separators=(',', ':')).replace('<', '\\u003c')
    output = folder/'map_3d.html'
    output.write_text(template.replace('/*__MAP_DATA__*/', payload), encoding='utf-8')
    # Export a standard XYZ PLY with upward-positive Z; no synthetic surfaces.
    points = np.array(layers[0]['points']).reshape(-1, 3)
    name = Path(layers[0]['file']).stem+'_neu.ply'
    with (folder/name).open('w', encoding='ascii') as f:
        f.write('ply\nformat ascii 1.0\ncomment x North y East z Up metres\n'
                f'element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(f, points, fmt='%.5f')
    return output
