"""Render full local point/mesh products in the isolated Open3D environment."""
import os
os.environ.setdefault('OMP_NUM_THREADS','4')
from pathlib import Path
import argparse
from ag.figure_render import render

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path)
    render(p.parse_args().folder)
