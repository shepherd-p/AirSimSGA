import argparse
from pathlib import Path
from ag.study_figures import StudyFigures


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input-dir',type=Path,required=True);a=p.parse_args()
    figures=StudyFigures(a.input_dir);figures.export()
