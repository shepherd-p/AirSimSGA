"""Explicit runtime scene reload. Does not restore a running mission's transient state."""
import argparse
import json
import os
import sys
from ag.backend import Backend
from ag.storage import ROOT

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--scene', default='scene_citypark_sga.jsonc')
    p.add_argument('--config-dir',default=str(ROOT.parent/'CityPark1/sim_config'))
    args = p.parse_args()
    os.chdir(ROOT)
    config = json.loads((ROOT/'configs/mapping.json').read_text(encoding='utf-8'))
    b = Backend(config)
    try:
        b.connect()
        from projectairsim import World
        w = World(b.client,args.scene,sim_config_path=args.config_dir,delay_after_load_sec=1)
        print('Reloaded runtime scene:',w.parent_topic)
    finally:
        b.close()
