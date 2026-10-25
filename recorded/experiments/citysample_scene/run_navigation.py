"""Aerial lane-closure discovery and parallel-lane bypass in City Sample."""

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
from urban_scene.probe_urban_scene import camera_pose
from urban_scene.urban_runtime import (
    collision_snapshot, new_run, receipt, save_camera, save_observation_pair, sha256,
)


def cross_track(position, path):
    starts = path[:-1, :2]
    delta = np.diff(path[:, :2], axis=0)
    fractions = np.clip(np.sum((position[:2] - starts) * delta, axis=1) /
                        np.maximum(np.sum(delta * delta, axis=1), 1e-9), 0, 1)
    return float(np.linalg.norm(starts + fractions[:, None] * delta - position[:2], axis=1).min())


async def run(max_sim_seconds: float, vehicle_evidence: bool = False):
    scenario_path = HERE / "generated/navigation_scenario.json"
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    source = ROOT.parent / "CitySample/Saved/AGCitySampleZoneGraph.json"
    if sha256(source) != scenario["zonegraph_sha256"]:
        raise RuntimeError("City Sample ZoneGraph export changed after route planning")
    cfg = {
        "task": "citysample_navigation", "mode": "air", "scene": "scene_navigation_air.jsonc",
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "sensor_max_age_sec": 2.0, "sensor_pair_tolerance_sec": .06,
        "tracking": {"minimum_pixels": 25, "segmentation_color_order": "bgr"},
        "closure": scenario["closure"], "max_sim_seconds": max_sim_seconds,
        "control_hz": 3.0, "car_speed_mps": 3.0, "drone_speed_mps": 4.0,
        "goal_tolerance_m": 4.0, "closure_acceptance_m": 8.0,
        "ambient_traffic": "disabled by -ControlledTraffic startup option",
        "route_policy": "Car waits for UAV depth/segmentation observation, then uses an adjacent native same-direction lane to bypass the closure",
        "observer_position_ned": [-612.0, -352.0, -230.0],
        "observer_target_ned": [-612.0, -352.0, 0.0],
        "vehicle_evidence": vehicle_evidence,
    }
    output = new_run("citysample_navigation", "air")
    write_json(output / "config.json", cfg)
    write_json(output / "scenario.json", scenario)
    sources = [__file__, HERE / "citysample_runtime.py", HERE / cfg["scene"],
               HERE / "prepare_navigation.py", scenario_path,
               ROOT / "urban_scene/probe_urban_scene.py",
               ROOT / "ag/backend.py", ROOT / "ag/planning.py",
               ROOT / "ag/study_planning.py", ROOT / "ag/math3d.py"]
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for path in sources:
        path = Path(path)
        (snapshot / path.name).write_bytes(path.read_bytes())
    write_json(output / "provenance.json", {
        "source_sha256": {Path(path).name: sha256(path) for path in sources},
        "zonegraph_sha256": sha256(source),
        "map_asset": "/Game/Map/Big_City_LVL", "game_mode": "ProjectAirSimGameMode",
    })
    path = np.asarray(scenario["diverted_route_ned"], dtype=float)
    baseline = np.asarray(scenario["baseline_route_ned"], dtype=float)
    closure_xy = np.asarray(cfg["closure"]["center_ned"][:2])
    closure_waypoint = int(np.argmin(np.linalg.norm(baseline[:, :2] - closure_xy, axis=1)))
    scout_path = baseline[:closure_waypoint + 1]
    backend = CitySampleBackend(cfg)
    status, error, detection = "failed", None, None
    events = []
    collisions, collision_stamps = [], {}
    captures = []
    vehicle_captures = []
    captured_stages = set()
    reached_closure = False
    distance_at_closure = None
    candidate_streak, candidate_last_time = 0, -1.0
    first_car, last_car = None, None
    car_distance_m = 0.0
    start_wall = time.monotonic()
    stream = (output / "trajectories_controls.csv").open("w", newline="", encoding="utf-8")
    fields = ["sim_time_sec", "elapsed_sec", "robot", "x", "y", "z", "yaw_rad",
              "speed_mps", "goal_x", "goal_y", "goal_z", "engine", "steering",
              "brake", "cross_track_m", "phase"]
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()

    def event(kind, now, **details):
        events.append({"kind": kind, "sim_time_sec": now, **details})
        (output / "events.jsonl").open("a", encoding="utf-8").write(
            json.dumps(events[-1]) + "\n")

    try:
        backend.connect()
        write_json(output / "loaded_scene.json", backend.load())
        (output / "closure_barrier.glb").write_bytes(backend.closure_payload)
        write_json(output / "live_objects.json", {
            "closure_actor": backend.closure_actor,
            "closure_pose": backend.world.get_object_pose(backend.closure_actor),
            "actors": backend.world.list_actors()})
        from projectairsim.types import Pose
        observer_pose = camera_pose(cfg["observer_position_ned"],
                                    cfg["observer_target_ned"], Pose)
        accepted = backend.client.request({
            "method": backend.world.parent_topic +
                      "/robots/Observer/sensors/Overview/SetPose",
            "params": {"pose": observer_pose, "wait_for_pose_update": True},
            "version": 1.0})
        if not accepted:
            raise RuntimeError("Observer camera pose rejected")
        await backend.ready(60)
        await backend.settle()
        write_json(output / "startup_pose_sync.json", await backend.wait_for_pose_sync())
        car = backend.pose("Car1")
        if np.linalg.norm(car["position"][:2] - np.asarray(scenario["start_ned"][:2])) > 4:
            raise RuntimeError("Car failed to settle at the declared native lane start")
        captures.append(save_camera(backend, "Car1/scene_camera", output / "street_start.png"))
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_start.png"))
        await backend.start()
        write_json(output / "post_takeoff_pose_sync.json", await backend.wait_for_pose_sync())
        start_sim = backend.now()
        last_sim, last_progress = start_sim, time.monotonic()
        last_observation_time = -math.inf

        async def capture_vehicle(stage):
            import cv2
            from projectairsim.utils import unpack_image

            current_car = backend.pose("Car1")["position"]
            midpoint = (current_car[:2] + closure_xy) / 2
            camera_position = [float(midpoint[0]), float(midpoint[1]), -75.0]
            observer_pose = camera_pose(camera_position,
                                        [*camera_position[:2], 0.0], Pose)
            accepted = backend.client.request({
                "method": backend.world.parent_topic +
                          "/robots/Observer/sensors/Overview/SetPose",
                "params": {"pose": observer_pose, "wait_for_pose_update": True},
                "version": 1.0,
            })
            if not accepted:
                raise RuntimeError(f"Observer rejected {stage} camera pose")
            deadline = time.monotonic() + 8
            closure_bbox = None
            reprojection_error = None
            while time.monotonic() < deadline:
                message = backend.buffer.latest("Observer/Overview/scene_camera", 2)
                if message is not None and np.linalg.norm(
                    np.array([message[f"pos_{axis}"] for axis in "xyz"]) -
                    camera_position) < 0.2:
                    image = unpack_image(message)
                    orange = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                                         (8, 140, 150), (25, 255, 255))
                    _, _, stats, _ = cv2.connectedComponentsWithStats(orange)
                    candidates = [row for row in stats[1:]
                                  if row[4] > 300 and 15 <= row[2] <= 120
                                  and 15 <= row[3] <= 100]
                    if candidates:
                        x, y, width, height, _ = max(candidates,
                                                     key=lambda row: row[4])
                        depth = cfg["closure"]["center_ned"][2] - camera_position[2]
                        focal = image.shape[1] / (2 * math.tan(math.radians(40)))
                        predicted = np.array([
                            image.shape[1] / 2 + focal *
                            (closure_xy[1] - camera_position[1]) / depth,
                            image.shape[0] / 2 - focal *
                            (closure_xy[0] - camera_position[0]) / depth,
                        ])
                        reprojection_error = float(np.linalg.norm(
                            np.array([x + width / 2, y + height / 2]) - predicted))
                        if reprojection_error < 25:
                            closure_bbox = [int(x), int(y),
                                            int(x + width), int(y + height)]
                            break
                await asyncio.sleep(.1)
            else:
                raise RuntimeError(
                    f"No synchronized {stage} observer frame; "
                    f"last closure reprojection error={reprojection_error}")
            frame = save_camera(backend, "Observer/Overview/scene_camera",
                                output / "vehicle_evidence" / f"{stage}.png")
            if frame is None:
                raise RuntimeError(f"Cannot archive {stage} observer frame")
            captures.append(frame)
            car_pose = backend.pose("Car1")["position"]
            evidence = {"stage": stage, "frame": frame,
                        "car_pose_ned": car_pose.tolist(),
                        "closure_pose_ned": cfg["closure"]["center_ned"],
                        "car_closure_distance_m": float(np.linalg.norm(
                            car_pose[:2] - closure_xy)),
                        "observer_position_ned": camera_position,
                        "closure_bbox_px": closure_bbox,
                        "closure_reprojection_error_px": reprojection_error,
                        "sim_time_sec": backend.now()}
            vehicle_captures.append(evidence)
            event("vehicle_evidence_captured", evidence["sim_time_sec"],
                  stage=stage, distance_m=evidence["car_closure_distance_m"],
                  frame=frame)

        while backend.now() - start_sim < max_sim_seconds:
            now = backend.now()
            if now > last_sim:
                last_sim, last_progress = now, time.monotonic()
            if time.monotonic() - last_progress > 20 or time.monotonic() - start_wall > 1800:
                raise TimeoutError("City Sample simulation stopped progressing")
            car, drone = backend.pose("Car1"), backend.pose("Drone1")
            if first_car is None:
                first_car = car["position"].copy()
            if last_car is not None:
                car_distance_m += float(np.linalg.norm(car["position"][:2] - last_car[:2]))
            last_car = car["position"].copy()
            new_collisions = collision_snapshot(backend, backend.kinds, collision_stamps)
            if new_collisions:
                collisions.extend(new_collisions)
                for hit in new_collisions:
                    event("collision", now, robot=hit["robot"],
                          object_name=hit["event"].get("object_name"))
                status = "collision"
                break
            if detection is None:
                observation = backend.observation("Drone1", cfg["closure"]["segmentation_id"])
                if observation is not None and observation["time"] > last_observation_time:
                    last_observation_time = observation["time"]
                    error_m = float(np.linalg.norm(observation["position"][:2] - closure_xy))
                    if error_m <= cfg["closure_acceptance_m"]:
                        candidate_streak = candidate_streak + 1 if (
                            observation["time"] - candidate_last_time < 1.5) else 1
                        candidate_last_time = observation["time"]
                        if candidate_streak >= 3:
                            detection = {"time_sec": observation["time"],
                                         "estimated_ned": observation["position"].tolist(),
                                         "pixels": observation["pixels"],
                                         "pair_delta_ms": observation["pair_delta_ms"],
                                         "evaluation_error_m": error_m}
                            save_observation_pair(backend, "Drone1", output / "closure_detection_raw.npz")
                            captures.append(save_camera(backend, "Drone1/scene_camera",
                                                        output / "closure_detection.png"))
                            event("closure_observed", now, **detection)
                            event("car_released_on_detour", now,
                                  lane_ids=scenario["diverted_lanes"])
                    else:
                        candidate_streak = 0
                        event("observation_rejected", now, error_m=error_m,
                              pixels=observation["pixels"])
            if detection is None and now - start_sim > 150:
                status = "no_aerial_closure_detection"
                break
            if detection is not None and np.linalg.norm(car["position"][:2] - path[-1, :2]) < cfg["goal_tolerance_m"]:
                status = "completed"
                break
            if detection is not None and cross_track(car["position"], path) > 8:
                status = "off_route"
                break
            drone_target, _ = path_lookahead(scout_path, drone["position"], lookahead=12)
            horizontal = drone_target[:2] - drone["position"][:2]
            norm = np.linalg.norm(horizontal)
            velocity = np.array([*(horizontal / max(norm, 1e-6) * min(cfg["drone_speed_mps"], norm)),
                                 float(np.clip((-25 - drone["position"][2]) * .6, -3, 3))])
            if detection is not None:
                velocity[:2] = np.clip((closure_xy - drone["position"][:2]) * .4, -3, 3)
            await backend.drone_velocity("Drone1", velocity)
            if detection is not None:
                target, _ = path_lookahead(path, car["position"], lookahead=8)
                speed = float(np.linalg.norm(car["velocity"][:2]))
                engine, steering, brake = rover_command(
                    car["position"], car["yaw"], target, speed,
                    desired_speed=cfg["car_speed_mps"])
            else:
                target, engine, steering, brake = None, 0, 0, 1
            await backend.rover("Car1", engine, steering, brake)
            if vehicle_evidence and detection is not None:
                distance_to_closure = float(np.linalg.norm(
                    car["position"][:2] - closure_xy))
                if distance_to_closure <= 8:
                    reached_closure = True
                    distance_at_closure = car_distance_m
                stage = None
                if "approach" not in captured_stages and distance_to_closure <= 32:
                    stage = "approach"
                elif reached_closure and "beside_closure" not in captured_stages:
                    stage = "beside_closure"
                elif ("beside_closure" in captured_stages and
                      "departure" not in captured_stages and
                      car_distance_m >= distance_at_closure + 15 and
                      distance_to_closure >= 15):
                    stage = "departure"
                if stage:
                    await capture_vehicle(stage)
                    captured_stages.add(stage)
            for name, pose, goal in (("Car1", car, target), ("Drone1", drone, drone_target)):
                writer.writerow({"sim_time_sec": now, "elapsed_sec": now - start_sim,
                                 "robot": name, "x": pose["position"][0],
                                 "y": pose["position"][1], "z": pose["position"][2],
                                 "yaw_rad": pose["yaw"],
                                 "speed_mps": np.linalg.norm(pose["velocity"][:2]),
                                 "goal_x": goal[0] if goal is not None else "",
                                 "goal_y": goal[1] if goal is not None else "",
                                 "goal_z": goal[2] if goal is not None else "",
                                 "engine": engine if name == "Car1" else "",
                                 "steering": steering if name == "Car1" else "",
                                 "brake": brake if name == "Car1" else "",
                                 "cross_track_m": cross_track(car["position"], path) if name == "Car1" else "",
                                 "phase": "detour" if detection else "aerial_scout"})
            stream.flush()
            await asyncio.sleep(1 / cfg["control_hz"])
        if status == "failed":
            status = "timeout"
        captures.append(save_camera(backend, "Car1/scene_camera", output / "street_end.png"))
        captures.append(save_camera(backend, "Drone1/scene_camera", output / "aerial_end.png"))
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_capture.png"))
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(error, flush=True)
    finally:
        try:
            cleanup = await backend.cleanup()
        except Exception as exc:
            cleanup = [f"cleanup: {type(exc).__name__}: {exc}"]
        stream.close()
        receipt(output, status, error=error, detection=detection,
                first_car_pose_ned=first_car.tolist() if first_car is not None else None,
                last_car_pose_ned=last_car.tolist() if last_car is not None else None,
                car_distance_m=car_distance_m, collision_events=collisions,
                vehicle_captures=vehicle_captures,
                captures=[capture for capture in captures if capture],
                wall_duration_sec=time.monotonic() - start_wall, cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status,
                      "car_distance_m": car_distance_m, "error": error}), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-sim-seconds", type=float, default=220)
    parser.add_argument("--vehicle-evidence", action="store_true",
                        help="Archive external views of the physical UGV bypass")
    args = parser.parse_args()
    raise SystemExit(0 if asyncio.run(run(args.max_sim_seconds,
                                         args.vehicle_evidence)) == "completed" else 1)


if __name__ == "__main__":
    main()
