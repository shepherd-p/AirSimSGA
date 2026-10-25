"""Verify recorded study data and replay its sensor-derived calculations offline."""
import argparse
import json
from pathlib import Path
from ag.study_audit import audit


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir',required=True,type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();print(json.dumps(audit(args.input_dir,args.output),indent=2))
