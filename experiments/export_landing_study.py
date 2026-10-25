import argparse
from pathlib import Path
from ag.landing_figures import export_suite

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input-dir',type=Path,required=True)
    print(export_suite(p.parse_args().input_dir))
