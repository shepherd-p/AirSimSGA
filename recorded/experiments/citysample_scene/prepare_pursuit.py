"""Generate a reproducible two-exit City Sample pursuit from native lanes."""

from __future__ import annotations

import hashlib
import json
import math
from copy import deepcopy
from pathlib import Path

import numpy as np

from inspect_zonegraph import SOURCE
from plan_routes import load_graph


HERE = Path(__file__).resolve().parent
APPROACH, JUNCTION = 83819, 90037
EXITS = (83799, 83796)


def points_ned(lane):
    points = np.asarray(lane["points_cm"], dtype=float) / 100
    points[:, 2] *= -1
    return points


def at_distance(points, distance):
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
    return np.array([np.interp(distance, lengths, points[:, axis]) for axis in range(3)])


def heading(points, distance):
    before = at_distance(points, max(0, distance - 2))
    after = at_distance(points, distance + 2)
    return math.degrees(math.atan2(after[1] - before[1], after[0] - before[0]))


def origin(point, yaw):
    # The rover's physical mesh settles onto the road after scene load.
    return {"xyz": f"{point[0]:.4f} {point[1]:.4f} -2.0",
            "rpy-deg": f"0 0 {yaw:.4f}"}


def main():
    graph, lanes = load_graph(SOURCE)
    for exit_lane in EXITS:
        if not graph.has_edge(JUNCTION, exit_lane):
            raise RuntimeError(f"Exit {exit_lane} does not leave the declared junction")
    if not graph.has_edge(APPROACH, JUNCTION):
        raise RuntimeError("Declared approach does not enter the junction lane")
    junction = points_ned(lanes[JUNCTION])
    cumulative = np.r_[0, np.cumsum(np.linalg.norm(np.diff(junction[:, :2], axis=0), axis=1))]
    target_distance = cumulative[-1] - 80
    if target_distance < 25:
        raise RuntimeError("Approach lane is too short for the target staging point")
    target_start = at_distance(junction, target_distance)
    stage = np.vstack((target_start, junction[np.searchsorted(cumulative, target_distance):]))
    gate_points = {}
    gate_starts = {}
    gate_yaws = {}
    routes = {}
    for label, lane_id in zip(("Left", "Right"), EXITS):
        exit_points = points_ned(lanes[lane_id])
        gate_starts[label] = at_distance(exit_points, 10)
        gate_points[label] = at_distance(exit_points, 20)
        gate_yaws[label] = heading(exit_points, 10)
        routes[label] = np.vstack((stage, exit_points[1:])).tolist()
    drone_track_start = at_distance(junction, target_distance - 12)
    drone_overwatch_start = at_distance(junction, cumulative[-1] - 18)
    actors = [
        {"type": "robot", "name": "Target",
         "origin": origin(target_start, heading(junction, target_distance)),
         "robot-config": "../configs/robot_ugv_navigation_study.jsonc"},
        *[{"type": "robot", "name": "Gate" + label,
           "origin": origin(gate_starts[label], gate_yaws[label]),
           "robot-config": "../urban_scene/robots/pursuit_interceptor.jsonc"}
          for label in ("Left", "Right")],
        {"type": "robot", "name": "Drone1",
         "origin": origin(drone_track_start, heading(junction, target_distance - 12)),
         "robot-config": "../urban_scene/robots/pursuit_uav.jsonc"},
        {"type": "robot", "name": "Drone2",
         "origin": origin(drone_overwatch_start, heading(junction, cumulative[-1] - 18)),
         "robot-config": "../urban_scene/robots/pursuit_uav.jsonc"},
        {"type": "robot", "name": "Observer",
         "origin": origin(np.zeros(3), 0),
         "robot-config": "../urban_scene/observer_probe.jsonc"},
    ]
    scene = {
        "id": "AGCitySamplePursuit",
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
        "approach_lane": APPROACH, "junction_lane": JUNCTION,
        "exit_lanes": dict(zip(("Left", "Right"), EXITS)),
        "target_start_ned": target_start.tolist(),
        "fork_ned": junction[-1].tolist(),
        "gate_start_ned": {key: value.tolist() for key, value in gate_starts.items()},
        "gate_ned": {key: value.tolist() for key, value in gate_points.items()},
        "routes_ned": routes,
        "observer_position_ned": [float(junction[-1, 0] - 20),
                                  float(junction[-1, 1] - 65), -130],
        "observer_target_ned": ((junction[-1] + target_start) / 2).tolist(),
        "approach_distance_m": 80,
    }
    generated = HERE / "generated"
    generated.mkdir(exist_ok=True)
    tracker = json.loads((HERE.parent / "urban_scene/robots/pursuit_interceptor.jsonc")
                         .read_text(encoding="utf-8"))
    camera = next(sensor for sensor in tracker["sensors"]
                  if sensor["id"] == "FrontCamera")
    camera["origin"]["rpy-deg"] = "0 0 80"
    camera["capture-settings"] = [
        {**camera["capture-settings"][0], "image-type": image_type}
        for image_type in (0, 2, 3)
    ]
    (generated / "pursuit_tracker_ugv.jsonc").write_text(
        json.dumps(tracker, indent=2) + "\n", encoding="utf-8")
    evidence_scene = deepcopy(scene)
    evidence_scene["id"] = "AGCitySamplePursuitViews"
    next(actor for actor in evidence_scene["actors"]
         if actor["name"] == "GateRight")["robot-config"] = (
             "generated/pursuit_tracker_ugv.jsonc")
    (generated / "scene_pursuit_views.jsonc").write_text(
        json.dumps(evidence_scene, indent=2) + "\n", encoding="utf-8")
    (generated / "scene_pursuit.jsonc").write_text(
        json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    (generated / "pursuit_scenario.json").write_text(
        json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"scene": str(generated / "scene_pursuit.jsonc"),
                      "target_start_ned": scenario["target_start_ned"],
                      "gate_ned": scenario["gate_ned"]}))


if __name__ == "__main__":
    main()
