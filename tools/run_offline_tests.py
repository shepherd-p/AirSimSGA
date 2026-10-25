"""Run preserved algorithm/geometry/evidence gates and release tests, without UE."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import sys
import unittest
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extra-site-packages',type=Path,help='Optional existing dependency environment, appended after the active environment')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    sys.path[:0]=[str(ROOT),str(ROOT/'experiments'),str(ROOT/'experiments/citysample_scene')]
    if args.extra_site_packages:
        if not args.extra_site_packages.is_dir():parser.error('Extra dependency directory missing')
        sys.path.append(str(args.extra_site_packages.resolve()))
    loader=unittest.TestLoader();suite=unittest.TestSuite()
    suite.addTests(unittest.TestLoader().discover(str(ROOT/'experiments/tests'),pattern='test_*.py'))
    suite.addTests(unittest.TestLoader().discover(str(ROOT/'tests'),pattern='test_*.py'))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    if args.report:
        packages={}
        for name in ['numpy','opencv-python','matplotlib','Pillow','psutil','open3d','pupil-apriltags']:
            try:packages[name]=importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:packages[name]='not installed'
        report={'created_utc':datetime.now(timezone.utc).isoformat(),'python':sys.version,'tests_run':result.testsRun,
                'failures':[str(test) for test,_ in result.failures],'errors':[str(test) for test,_ in result.errors],
                'skipped':[str(test) for test,_ in result.skipped],'passed':result.wasSuccessful(),
                'extra_dependency_environment_used':args.extra_site_packages is not None,'packages':packages,
                'scope':'offline algorithm/contact/evidence/geometry/portability regressions; no simulator connected'}
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return 0 if result.wasSuccessful() else 1
if __name__=='__main__':raise SystemExit(main())
