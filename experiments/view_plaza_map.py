"""Open the latest successful three-product plaza viewer without rerunning simulation."""
import argparse
from pathlib import Path
import webbrowser
import json
from ag.storage import ROOT


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input-dir',type=Path);a=p.parse_args()
    choices=([a.input_dir/'plaza_results/map_plaza_3d.html'] if a.input_dir else
             [f for f in sorted((ROOT/'outputs/mapping').glob('*/plaza_results/map_plaza_3d.html'),reverse=True)
              if json.loads((f.parent/'surface/quality.json').read_text(encoding='utf-8'))['quality_status']=='passed'
              and json.loads((f.parents[1]/'manifest.json').read_text(encoding='utf-8'))['status']=='completed'])
    if not choices or not choices[0].exists():p.error('No plaza result yet; run run_mapping_plaza.py first')
    print(choices[0]);webbrowser.open(choices[0].resolve().as_uri())
