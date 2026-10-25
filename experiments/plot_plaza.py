"""Export figures after render_plaza.py has generated full-data map renders."""
import argparse
from pathlib import Path
from ag.figure_plot import plot

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);plot(p.parse_args().folder)
