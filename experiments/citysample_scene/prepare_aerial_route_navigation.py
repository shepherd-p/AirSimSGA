"""Freeze a high-altitude, three-intersection City Sample route scenario."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from district_routes import lane_points, load_lanes, route


HERE = Path(__file__).resolve().parent
import sys
sys.path.insert(0, str(HERE.parent))
from ag.portable import zonegraph_path
SOURCE = zonegraph_path()
START_LANE = 79595
GOAL_LANE = 84251
JUNCTION_LANES = (88198, 68441, 72453)
START_OFFSET_M = 0.0
CONTACT_PROXY_Z_M = -0.45


def center_ground_contacts(robot, proxy_z):
    by_name = {link["name"]: link for link in robot["links"]}
    for actuator in robot["actuators"]:
        visual_name = actuator["child-link"]
        proxy_name = f"Contact_{visual_name}"
        proxy = copy.deepcopy(by_name[visual_name])
        proxy["name"] = proxy_name
        proxy["inertial"]["origin"]["xyz"] = f"0 0 {proxy_z}"
        proxy["visual"]["origin"]["xyz"] = f"0 0 {proxy_z}"
        robot["links"].append(proxy)
        robot["joints"].append({
            "id": f"Frame_{proxy_name}", "type": "fixed",
            "parent-link": "Frame", "child-link": proxy_name,
            "axis": "0 0 1",
        })
        actuator["child-link"] = proxy_name


def lane_point_at_distance(points, distance):
    distances = np.r_[0, np.cumsum(np.linalg.norm(
        np.diff(points[:, :2], axis=0), axis=1))]
    if not 0 <= distance < distances[-1]:
        raise ValueError("Start offset is outside the selected lane")
    return np.array([np.interp(distance, distances, points[:, axis])
                     for axis in range(3)])


def main():
    lanes = load_lanes(SOURCE)
    indices, length = route(lanes, START_LANE, GOAL_LANE)
    if not all(index in indices for index in JUNCTION_LANES):
        raise RuntimeError("Chosen junctions are not on the directed route")
    junctions = []
    for number, index in enumerate(JUNCTION_LANES, 1):
        points = lane_points(lanes[index])
        junctions.append({
            "index": number,
            "lane_id": index,
            "center_ned": points.mean(axis=0).tolist(),
        })
    scenario = {
        "map": "Big_City_LVL",
        "road_source": "City Sample native vehicle ZoneGraph",
        "zonegraph_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        "start_lane": START_LANE,
        "goal_lane": GOAL_LANE,
        "start_ned": lane_point_at_distance(
            lane_points(lanes[START_LANE]), START_OFFSET_M).tolist(),
        "start_offset_on_lane_m": START_OFFSET_M,
        "goal_ned": lane_points(lanes[GOAL_LANE])[-1].tolist(),
        "candidate_lane_ids": indices,
        "candidate_length_m": length,
        "expected_driven_length_m": length - START_OFFSET_M,
        "survey_junctions": junctions,
        "survey_altitude_ned_m": -120.0,
        "observer_position_ned": [-600.0, -230.0, -420.0],
        "observer_target_ned": [-600.0, -230.0, 0.0],
        "planning_method": "Directed shortest path on the native road graph; UAV junction views are archived before the route is committed to UGV control",
        "ugv_contact_proxy_origin_ned_m": [0, 0, CONTACT_PROXY_Z_M],
        "ugv_physics_scope": "Simple-drive wheel kinematics with centered collision-isolation links; wheel-ground contact feedback is not asserted",
    }
    start = lane_point_at_distance(lane_points(lanes[START_LANE]),
                                   START_OFFSET_M)
    tangent = lane_points(lanes[START_LANE])[1, :2] - start[:2]
    tangent /= np.linalg.norm(tangent)
    yaw = np.degrees(np.arctan2(tangent[1], tangent[0]))
    drone_xy = start[:2] + np.array([tangent[1], -tangent[0]]) * 8
    scene = {
        "id": "AGCitySampleAerialRoute",
        "clock": {"type": "steppable", "step-ns": 5000000,
                  "real-time-update-rate": 5000000, "pause-on-start": True},
        "home-geo-point": {"latitude": 0, "longitude": 0, "altitude": 0},
        "segmentation": {"initialize-ids": True, "ignore-existing": False,
                         "use-owner-name": True},
        "scene-type": "UnrealNative",
        "actors": [
            {"type": "robot", "name": "Car1",
             "origin": {"xyz": f"{start[0]:.4f} {start[1]:.4f} -2.0",
                        "rpy-deg": f"0 0 {yaw:.4f}"},
             "robot-config": "generated/robot_ugv_aerial_route.jsonc"},
            {"type": "robot", "name": "Drone1",
             "origin": {"xyz": f"{drone_xy[0]:.4f} {drone_xy[1]:.4f} -2.0",
                        "rpy-deg": f"0 0 {yaw:.4f}"},
             "robot-config": "../configs/robot_uav_navigation_study.jsonc"},
            {"type": "robot", "name": "Observer",
             "origin": {"xyz": "0 0 0", "rpy-deg": "0 0 0"},
             "robot-config": "../urban_scene/observer_probe.jsonc"},
        ],
    }
    output = HERE / "generated/aerial_route_scenario.json"
    output.parent.mkdir(exist_ok=True)
    robot = json.loads((HERE.parent / "configs/robot_ugv_navigation_study.jsonc").read_text())
    center_ground_contacts(robot, CONTACT_PROXY_Z_M)
    robot_path = output.with_name("robot_ugv_aerial_route.jsonc")
    robot_path.write_text(json.dumps(robot, indent=2) + "\n", encoding="utf-8")
    output.write_text(json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    scene_path = output.with_name("scene_navigation_aerial_route.jsonc")
    scene_path.write_text(json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"scenario": str(output), "scene": str(scene_path),
                    "ugv_config": str(robot_path),
                    "route_length_m": round(length - START_OFFSET_M, 1),
                      "junction_lanes": JUNCTION_LANES}))


if __name__ == "__main__":
    main()
