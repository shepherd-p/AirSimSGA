"""Run each test suite in its existing environment without installing packages."""
import argparse
import subprocess
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parent
SUITES={
    'control':['test_algorithms.py','test_cooperative_studies.py','test_evidence_archive.py','test_scan_capture.py','test_scene_figure.py','test_landing_core.py'],
    'reconstruction':['test_figure_render.py','test_plaza_reconstruction.py','test_reconstruction.py','test_structural_cloud.py']}

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--suite',choices=list(SUITES));args=parser.parse_args()
    if args.suite:
        suite=unittest.TestSuite()
        for pattern in SUITES[args.suite]:suite.addTests(unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern=pattern))
        result=unittest.TextTestRunner(verbosity=2).run(suite);sys.exit(0 if result.wasSuccessful() else 1)
    interpreters={'control':Path('C:/Users/Stephen/.conda/envs/projectairsim310/python.exe'),
                  'reconstruction':ROOT/'.venv_reconstruction/Scripts/python.exe'}
    for name,python in interpreters.items():
        if not python.is_file():raise FileNotFoundError(python)
        subprocess.run([str(python),str(Path(__file__).resolve()),'--suite',name],cwd=ROOT,check=True)
