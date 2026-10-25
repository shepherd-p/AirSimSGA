"""Report vehicle lanes near a world point for ground-image route selection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from inspect_zonegraph import SOURCE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("--radius", type=float, default=16)
    args = parser.parse_args()
    point = np.array([args.x, args.y])
    data = json.loads(SOURCE.read_text(encoding="utf-8"))
    matches = []
    for dataset in data["datasets"]:
        for lane in dataset["lanes"]:
            if len(lane["points_cm"]) < 2 or not lane["tag_mask"] & 2:
                continue
            points = np.asarray(lane["points_cm"], dtype=float)[:, :2] / 100
            delta = np.diff(points, axis=0)
            fraction = np.clip(np.sum((point - points[:-1]) * delta, axis=1) /
                               np.maximum(np.sum(delta * delta, axis=1), 1e-9), 0, 1)
            nearest = points[:-1] + fraction[:, None] * delta
            index = int(np.argmin(np.linalg.norm(nearest - point, axis=1)))
            distance = float(np.linalg.norm(nearest[index] - point))
            if distance <= args.radius:
                matches.append({"distance_m": round(distance, 2),
                                "lane": lane["index"],
                                "near_xy_m": nearest[index].round(2).tolist(),
                                "start_xy_m": points[0].round(2).tolist(),
                                "end_xy_m": points[-1].round(2).tolist(),
                                "length_m": round(float(np.linalg.norm(delta, axis=1).sum()), 2),
                                "outgoing": lane["outgoing"]})
    print(json.dumps(sorted(matches, key=lambda row: row["distance_m"])[:40], indent=2))


if __name__ == "__main__":
    main()
