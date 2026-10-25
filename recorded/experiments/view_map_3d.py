"""Open a measured 3D map without connecting to or changing the simulator."""
import argparse
import webbrowser
import json
from pathlib import Path
from ag.storage import ROOT
from ag.map3d import build_viewer


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir', type=Path, help='Default: latest finished mapping run with point clouds')
    p.add_argument('--no-open', action='store_true')
    args = p.parse_args()
    folder = args.input_dir
    if folder is None:
        candidates = []
        for candidate in sorted((ROOT/'outputs/mapping').glob('*'), reverse=True):
            manifest = candidate/'manifest.json'
            if manifest.exists() and list(candidate.glob('map_*.pcd')):
                if json.loads(manifest.read_text(encoding='utf-8'))['status'] != 'running':
                    candidates.append(candidate)
        if not candidates:
            p.error('No finished mapping output exists; run run_mapping.py first')
        folder = candidates[0]
    output = None
    for result in sorted((folder/'reconstruction_tsdf').glob('*/status.json'),reverse=True):
        info=json.loads(result.read_text(encoding='utf-8'))
        if info['status']=='completed' and info.get('viewer') and Path(info['viewer']).is_file():
            quality=json.loads((result.parent/'quality.json').read_text(encoding='utf-8'))
            if quality.get('quality_status')=='passed':
                output=Path(info['viewer'])
                break
    if output is None:
        output = build_viewer(folder)
    print(f'3D map: {output}', flush=True)
    if not args.no_open:
        if not webbrowser.open(output.as_uri()):
            print('Open the HTML file above in Edge or Chrome.')


if __name__ == '__main__':
    main()
