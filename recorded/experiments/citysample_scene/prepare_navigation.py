"""Freeze a local City Sample lane closure and parallel-lane bypass."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from inspect_zonegraph import SOURCE
from plan_routes import load_graph


HERE = Path(__file__).resolve().parent
START, PARALLEL = 79595, 79596


def lane_point(lane, distance):
    points = np.asarray(lane["points_cm"], dtype=float) / 100
    points[:, 2] *= -1
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
    if distance > lengths[-1]:
        raise RuntimeError("Requested point lies beyond the native lane")
    return np.array([np.interp(distance, lengths, points[:, axis]) for axis in range(3)])


def main():
    graph, lanes = load_graph(SOURCE)
    if START not in graph or PARALLEL not in graph:
        raise RuntimeError("Declared City Sample vehicle lanes are missing")
    start = lane_point(lanes[START], 0)
    next_point = lane_point(lanes[START], 5)
    center = lane_point(lanes[START], 85)
    angle = math.atan2(next_point[1] - start[1], next_point[0] - start[0])
    baseline = np.asarray([lane_point(lanes[START], distance)
                           for distance in (0, 35, 60, 85, 110, 145)])
    diverted = np.asarray([
        lane_point(lanes[START], 0),
        lane_point(lanes[START], 35),
        lane_point(lanes[START], 50),
        lane_point(lanes[PARALLEL], 70),
        lane_point(lanes[PARALLEL], 100),
        lane_point(lanes[START], 125),
        lane_point(lanes[START], 145),
    ])
    scenario = {
        "map": "Big_City_LVL", "road_source": "City Sample native ZoneGraph",
        "zonegraph_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        "start_lane": START, "parallel_lane": PARALLEL,
        "start_ned": start.tolist(), "start_yaw_deg": math.degrees(angle),
        "baseline_lanes": [START], "diverted_lanes": [START, PARALLEL, START],
        "lane_change_policy": "Continuous local bypass sampled from two same-direction native lanes; not a directed ZoneGraph edge",
        "baseline_route_ned": baseline.tolist(),
        "diverted_route_ned": diverted.tolist(),
        "baseline_length_m": float(np.linalg.norm(np.diff(baseline[:, :2], axis=0), axis=1).sum()),
        "diverted_length_m": float(np.linalg.norm(np.diff(diverted[:, :2], axis=0), axis=1).sum()),
        "closure": {"center_ned": [float(center[0]), float(center[1]), float(center[2] - 1.1)],
                    "yaw_deg": math.degrees(angle) + 90,
                    "size_m": [3.5, 1.5, 2.2], "segmentation_id": 75},
    }
    output = HERE / "generated/navigation_scenario.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "baseline_length_m": scenario["baseline_length_m"],
                      "diverted_length_m": scenario["diverted_length_m"],
                      "closure_center_ned": scenario["closure"]["center_ned"]}))


if __name__ == "__main__":
    main()
