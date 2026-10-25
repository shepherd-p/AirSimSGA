"""Reproduce the City Sample rover pose jump without the UAV sensor load."""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from ag.planning import rover_command
from ag.storage import write_json
from ag.study_planning import path_lookahead
from citysample_runtime import CitySampleBackend
from district_routes import cross_track, load_lanes, route_points
from prepare_aerial_route_navigation import center_ground_contacts
from urban_scene.urban_runtime import new_run, receipt


async def run(center_proxies, proxy_z, stop_after_m):
    scenario = json.loads((HERE / "generated/aerial_route_scenario.json").read_text())
    scene = json.loads((HERE / "generated/scene_navigation_aerial_route.jsonc").read_text())
    scene["id"] = "AGCitySampleUGVPoseDiagnostic"
    scene["actors"] = [actor for actor in scene["actors"]
                       if actor["name"] == "Car1"]
    robot = json.loads((ROOT / "configs/robot_ugv_navigation_study.jsonc").read_text())
    for sensor in robot["sensors"]:
        sensor["enabled"] = False
    if center_proxies:
        center_ground_contacts(robot, proxy_z)
    robot_path = HERE / "generated/robot_ugv_pose_diagnostic.jsonc"
    robot_path.write_text(json.dumps(robot, indent=2) + "\n", encoding="utf-8")
    scene["actors"][0]["robot-config"] = "generated/robot_ugv_pose_diagnostic.jsonc"
    scene_path = HERE / "generated/scene_ugv_pose_diagnostic.jsonc"
    scene_path.write_text(json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    cfg = {
        "scene": "generated/scene_ugv_pose_diagnostic.jsonc",
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "control_hz": 3.0, "desired_speed_mps": 2.0,
        "max_engine": .15, "stop_after_m": stop_after_m,
        "center_ground_contact_proxies": center_proxies,
        "contact_proxy_link_origin_ned": [0, 0, proxy_z] if center_proxies else None,
    }
    lanes = load_lanes(ROOT.parent / "CitySample/Saved/AGCitySampleZoneGraph.json")
    path = route_points(lanes, scenario["candidate_lane_ids"])
    output = new_run("citysample_ugv_pose_diagnostic",
                     f"center_z_{proxy_z:g}" if center_proxies else "original_contacts")
    write_json(output / "config.json", cfg)
    write_json(output / "scenario.json", scenario)
    backend = CitySampleBackend(cfg)
    status, error = "failed", None
    distance, previous, collision_stamp = 0.0, None, None
    last_motion_time = None
    latest_collision = None
    fields = ["sim_time_sec", "x", "y", "z", "yaw_rad", "speed_mps",
              "cross_track_m", "engine", "steering", "brake",
              "collision_time_ns", "collision_object", "collision_normal_x",
              "collision_normal_y", "collision_normal_z",
              "collision_penetration_m"]
    stream = (output / "telemetry.csv").open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    events = (output / "events.jsonl").open("w", encoding="utf-8")

    def event(kind, **details):
        record = {"kind": kind, "sim_time_sec": backend.now(), **details}
        events.write(json.dumps(record) + "\n")
        events.flush()
        print(json.dumps(record), flush=True)

    try:
        backend.connect()
        write_json(output / "loaded_scene.json", backend.load())
        await backend.ready(60)
        await backend.settle()
        await backend.start()
        start = backend.now()
        last_motion_time = start
        deadline = time.monotonic() + max(240, stop_after_m * 3)
        while (backend.now() - start < max(120, stop_after_m * 2)
               and time.monotonic() < deadline):
            car = backend.pose("Car1")
            current = car["position"]
            if previous is not None:
                step_distance = float(np.linalg.norm(current[:2] - previous[:2]))
                distance += step_distance
                if step_distance > .02:
                    last_motion_time = backend.now()
            speed = float(np.linalg.norm(car["velocity"][:2]))
            deviation = cross_track(current, path)
            packet = backend.collision("Car1")
            if packet is not None and packet["time_stamp"] != collision_stamp:
                collision_stamp = packet["time_stamp"]
                latest_collision = packet
            target, _ = path_lookahead(path, current, lookahead=12.0)
            far_target, _ = path_lookahead(path, current, lookahead=20.0)
            far_delta = far_target[:2] - current[:2]
            heading = math.atan2(far_delta[1], far_delta[0])
            turn_ahead = math.atan2(math.sin(heading - car["yaw"]),
                                    math.cos(heading - car["yaw"]))
            desired_speed = .8 if abs(turn_ahead) > .17 else cfg["desired_speed_mps"]
            engine, steering, brake = rover_command(
                current, car["yaw"], target, speed,
                desired_speed=desired_speed,
                max_engine=cfg["max_engine"])
            normal = (packet or {}).get("normal", {})
            writer.writerow({
                "sim_time_sec": car["time"],
                "x": current[0], "y": current[1], "z": current[2],
                "yaw_rad": car["yaw"], "speed_mps": speed,
                "cross_track_m": deviation, "engine": engine,
                "steering": steering, "brake": brake,
                "collision_time_ns": (packet or {}).get("time_stamp", ""),
                "collision_object": (packet or {}).get("object_name", ""),
                "collision_normal_x": normal.get("x", ""),
                "collision_normal_y": normal.get("y", ""),
                "collision_normal_z": normal.get("z", ""),
                "collision_penetration_m": (packet or {}).get(
                    "penetration_depth", ""),
            })
            stream.flush()
            if speed > 7 or deviation > 5:
                status = "pose_jump"
                event("pose_jump", speed_mps=speed, cross_track_m=deviation,
                      previous_position_ned=previous.tolist()
                      if previous is not None else None,
                      current_position_ned=current.tolist(),
                      latest_collision_packet=latest_collision)
                break
            if backend.now() - last_motion_time > 8 and distance > 1:
                status = "stalled"
                event("stalled", driven_distance_m=distance,
                      current_position_ned=current.tolist(),
                      latest_collision_packet=latest_collision)
                break
            if distance >= cfg["stop_after_m"]:
                status = "completed"
                event("distance_reached", driven_distance_m=distance)
                break
            previous = current.copy()
            await backend.rover("Car1", engine, steering, brake)
            await asyncio.sleep(1 / cfg["control_hz"])
        if status == "failed":
            status = "timeout"
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(error, flush=True)
    finally:
        cleanup = await backend.cleanup()
        stream.close()
        events.close()
        receipt(output, status, error=error, driven_distance_m=distance,
                latest_collision_packet=latest_collision,
                cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status,
                      "distance_m": distance, "error": error}), flush=True)
    return status


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--center-ground-proxies", action="store_true")
    parser.add_argument("--proxy-z", type=float, default=-0.2)
    parser.add_argument("--stop-after-m", type=float, default=90.0)
    args = parser.parse_args()
    raise SystemExit(0 if asyncio.run(run(args.center_ground_proxies, args.proxy_z,
                                         args.stop_after_m)) ==
                     "completed" else 1)
