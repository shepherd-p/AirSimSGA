"""Freeze a two-decision City Sample street-grid navigation scenario."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from district_routes import lane_points, load_lanes, route, route_points
HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent.parent / "CitySample/Saved/AGCitySampleZoneGraph.json"
START, GOAL = 86952, 74881
CLOSURES = (86958, 84792)
SEGMENTATION_IDS = (202, 204)


def heading(points):
    delta = points[1, :2] - points[0, :2]
    return math.degrees(math.atan2(delta[1], delta[0]))


def main():
    lanes = load_lanes(SOURCE)
    routes = []
    for count in range(3):
        indices, length = route(lanes, START, GOAL, CLOSURES[:count])
        routes.append({"lane_ids": indices, "length_m": length})
    if (CLOSURES[0] not in routes[0]["lane_ids"] or
            CLOSURES[1] not in routes[1]["lane_ids"] or
            CLOSURES[0] in routes[1]["lane_ids"] or
            any(index in routes[2]["lane_ids"] for index in CLOSURES)):
        raise RuntimeError("Declared closure decisions no longer match ZoneGraph")
    first = lane_points(lanes[START])
    yaw = heading(first)
    direction = first[1, :2] - first[0, :2]
    direction /= np.linalg.norm(direction)
    start = first[0].copy()
    start[:2] += direction * 10
    drone_start = start.copy()
    drone_start[:2] -= direction * 6
    closures = []
    for number, lane_id in enumerate(CLOSURES, 1):
        points = lane_points(lanes[lane_id])
        midpoint = (points[len(points) // 2 - 1] + points[len(points) // 2]) / 2
        direction = points[-1, :2] - points[0, :2]
        angle = math.degrees(math.atan2(direction[1], direction[0]))
        closures.append({
            "name": f"AGCitySampleRoadClosure{number}",
            "segmentation_id": SEGMENTATION_IDS[number - 1],
            "evaluation_lane_id": lane_id,
            "center_ned": [float(midpoint[0]), float(midpoint[1]), float(midpoint[2] - 1.1)],
            "yaw_deg": angle + 90,
            "size_m": [3.6, 1.5, 2.2],
        })
    actors = [
        {"type": "robot", "name": "Car1",
         "origin": {"xyz": f"{start[0]:.4f} {start[1]:.4f} -2.0",
                    "rpy-deg": f"0 0 {yaw:.4f}"},
         "robot-config": "../configs/robot_ugv_navigation_study.jsonc"},
        {"type": "robot", "name": "Drone1",
         "origin": {"xyz": f"{drone_start[0]:.4f} {drone_start[1]:.4f} -2.0",
                    "rpy-deg": f"0 0 {yaw:.4f}"},
         "robot-config": "../configs/robot_uav_navigation_study.jsonc"},
        {"type": "robot", "name": "Observer",
         "origin": {"xyz": "0 0 0", "rpy-deg": "0 0 0"},
         "robot-config": "../urban_scene/observer_probe.jsonc"},
    ]
    scene = {
        "id": "AGCitySampleDistrictNavigation",
        "clock": {"type": "steppable", "step-ns": 5000000,
                  "real-time-update-rate": 5000000, "pause-on-start": True},
        "home-geo-point": {"latitude": 0, "longitude": 0, "altitude": 0},
        "segmentation": {"initialize-ids": True, "ignore-existing": False,
                         "use-owner-name": True},
        "scene-type": "UnrealNative", "actors": actors,
    }
    scenario = {
        "map": "Big_City_LVL", "road_source": "City Sample native ZoneGraph",
        "zonegraph_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        "start_lane": START, "goal_lane": GOAL,
        "start_ned": start.tolist(),
        "goal_ned": lane_points(lanes[GOAL])[-1].tolist(),
        "evaluation_closures": closures,
        "reference_routes": routes,
        "observer_position_ned": [-800.0, -590.0, -350.0],
        "observer_target_ned": [-800.0, -590.0, 0.0],
    }
    output = HERE / "generated"
    output.mkdir(exist_ok=True)
    (output / "scene_navigation_district.jsonc").write_text(
        json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    (output / "district_navigation_scenario.json").write_text(
        json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"scene": str(output / "scene_navigation_district.jsonc"),
                      "route_lengths_m": [round(item["length_m"], 1) for item in routes],
                      "closure_lanes": CLOSURES}))


if __name__ == "__main__":
    main()
