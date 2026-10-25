"""Compare ProjectAirSim depth pixel conventions against recorded closure poses."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from ag.math3d import rotation


def evaluate(run: Path, number: int):
    from projectairsim.image_utils import segmentation_id_to_color
    from projectairsim.utils import unpack_image

    scenario = json.loads((run / "scenario.json").read_text(encoding="utf-8"))
    closure = scenario["evaluation_closures"][number - 1]
    packet_path = run / f"closure_{number}_raw.npz"
    if not packet_path.exists():
        packet_path = run / f"scout_probe_{number}_raw.npz"
    packet = np.load(packet_path)
    metadata = json.loads(packet["metadata_json"].item())
    depth_message = {**metadata["depth"], "data": packet["depth_payload"].tobytes()}
    segmentation_message = {
        **metadata["segmentation"],
        "data": packet["segmentation_payload"].tobytes(),
    }
    depth = unpack_image(depth_message).astype(float) / 1000
    segmentation = unpack_image(segmentation_message)
    color = segmentation_id_to_color(closure["segmentation_id"])
    mask = np.all(segmentation[:, :, :3] == [color.b, color.g, color.r], axis=2)
    v, u = np.where(mask)
    if len(u) == 0:
        raise ValueError(f"No pixels for closure {number} in {run}")
    distance = depth[mask]
    height, width = depth.shape
    focal = width / 2
    origin = np.array([depth_message[f"pos_{axis}"] for axis in "xyz"])
    orientation = rotation([depth_message[f"rot_{axis}"] for axis in "wxyz"])
    truth = np.array(closure["center_ned"])
    print(f"{run.name} closure={number} pixels={len(u)} median_uv={np.median(u):.0f},{np.median(v):.0f}")
    for horizontal in (-1, 1):
        for vertical in (-1, 1):
            rays = np.stack([
                np.ones_like(u),
                horizontal * (u - width / 2) / focal,
                vertical * (v - height / 2) / focal,
            ], axis=1)
            rays /= np.linalg.norm(rays, axis=1, keepdims=True)
            estimated = np.median((rays * distance[:, None]) @ orientation.T + origin,
                                  axis=0)
            error = np.linalg.norm(estimated[:2] - truth[:2])
            print(f"  image_axes={horizontal:+d},{vertical:+d} "
                  f"xy_error_m={error:.2f} estimated_ned={estimated.round(2).tolist()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="+", type=Path)
    args = parser.parse_args()
    from ag.portable import sdk_path
    sdk = sdk_path()
    sys.path.insert(0, str(sdk))
    for directory in args.run:
        for index in (1, 2):
            if ((directory / f"closure_{index}_raw.npz").exists() or
                    (directory / f"scout_probe_{index}_raw.npz").exists()):
                evaluate(directory, index)
