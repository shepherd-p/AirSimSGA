"""UAV-informed UGV rerouting across two City Sample street intersections."""

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
from district_routes import cross_track, load_lanes, nearest_route_lane, route, route_points
from urban_scene.probe_urban_scene import camera_pose
from urban_scene.urban_runtime import (
    collision_snapshot, new_run, receipt, save_camera, save_observation_pair, sha256,
)


async def run(max_sim_seconds: float):
    scenario_path = HERE / "generated/district_navigation_scenario.json"
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    source = ROOT.parent / "CitySample/Saved/AGCitySampleZoneGraph.json"
    if sha256(source) != scenario["zonegraph_sha256"]:
        raise RuntimeError("City Sample ZoneGraph changed after district route planning")
    lanes = load_lanes(source)
    start, goal = scenario["start_lane"], scenario["goal_lane"]
    initial_lanes, initial_length = route(lanes, start, goal)
    if initial_lanes != scenario["reference_routes"][0]["lane_ids"]:
        raise RuntimeError("Initial directed route no longer matches saved scenario")
    cfg = {
        "task": "citysample_district_navigation", "mode": "air_ground",
        "scene": "generated/scene_navigation_district.jsonc",
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "sensor_max_age_sec": 2.0, "sensor_pair_tolerance_sec": .06,
        "tracking": {"minimum_pixels": 25, "segmentation_color_order": "bgr"},
        "depth_pixel_signs": [-1, -1],
        "allocate_segmentation_ids": True,
        "closures": scenario["evaluation_closures"],
        "control_hz": 3.0, "car_speed_mps": 1.8, "drone_speed_mps": 5.0,
        "goal_tolerance_m": 4.0, "max_cross_track_m": 8.0,
        "max_sim_seconds": max_sim_seconds,
        "ambient_traffic": "disabled by -ControlledTraffic startup option",
        "route_policy": "UGV blocks only lanes matched to fresh UAV depth/segmentation observations, then reruns directed shortest-path planning",
        "truth_usage": "Closure positions spawn evaluation objects and are not passed to the ground route planner; own pose is used for vehicle feedback",
    }
    output = new_run("citysample_district_navigation", "air_ground")
    write_json(output / "config.json", cfg)
    write_json(output / "scenario.json", scenario)
    sources = [__file__, HERE / "district_routes.py", HERE / "citysample_runtime.py",
               HERE / "prepare_district_navigation.py", HERE / cfg["scene"],
               HERE / "check_depth_projection.py",
               scenario_path, ROOT / "urban_scene/probe_urban_scene.py",
               ROOT / "urban_scene/urban_runtime.py", ROOT / "ag/backend.py",
               ROOT / "ag/planning.py", ROOT / "ag/study_planning.py",
               ROOT / "ag/math3d.py"]
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for path in sources:
        (snapshot / Path(path).name).write_bytes(Path(path).read_bytes())
    write_json(output / "provenance.json", {
        "source_sha256": {Path(path).name: sha256(path) for path in sources},
        "zonegraph_sha256": sha256(source),
        "map_asset": "/Game/Map/Big_City_LVL", "game_mode": "ProjectAirSimGameMode",
    })
    backend = CitySampleBackend(cfg)
    route_lanes, route_length_m = initial_lanes, initial_length
    path = route_points(lanes, route_lanes)
    blocked_lanes, decisions, captures = [], [], []
    scout_probes = set()
    second_scout_released = False
    candidate_streak, candidate_time, last_seen = {}, {}, {}
    collision_stamps, collisions = {}, []
    status, error = "failed", None
    car_distance_m, previous_car, max_cross_track = 0.0, None, 0.0
    start_wall = time.monotonic()
    fields = ["sim_time_sec", "elapsed_sec", "robot", "x", "y", "z", "yaw_rad",
              "speed_mps", "goal_x", "goal_y", "goal_z", "engine", "steering",
              "brake", "cross_track_m", "turn_ahead_rad", "route_version", "phase"]
    stream = (output / "trajectories_controls.csv").open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    events = (output / "events.jsonl").open("w", encoding="utf-8")

    def event(kind, now, **details):
        record = {"kind": kind, "sim_time_sec": now, **details}
        events.write(json.dumps(record) + "\n")
        events.flush()
        print(json.dumps(record), flush=True)

    try:
        backend.connect()
        write_json(output / "loaded_scene.json", backend.load())
        scenario["evaluation_closures"] = cfg["closures"]
        write_json(output / "config.json", cfg)
        write_json(output / "scenario.json", scenario)
        candidate_streak = {closure["segmentation_id"]: 0
                            for closure in cfg["closures"]}
        candidate_time = {segmentation_id: -math.inf for segmentation_id in candidate_streak}
        last_seen = {segmentation_id: -math.inf for segmentation_id in candidate_streak}
        if set(backend.kinds) != {"Car1", "Drone1"}:
            raise RuntimeError("City Sample did not load both navigation actors")
        (output / "closure_barrier.glb").write_bytes(backend.closure_payload)
        write_json(output / "live_objects.json", {
            "closures": {name: backend.world.get_object_pose(actor)
                         for name, actor in backend.closure_actors.items()},
            "actors": backend.world.list_actors(),
        })
        segmentation_map = backend.world.get_segmentation_id_map()
        write_json(output / "segmentation_ids.json", segmentation_map)
        for closure in cfg["closures"]:
            owners = [name for name, segmentation_id in segmentation_map.items()
                      if segmentation_id == closure["segmentation_id"]]
            if owners != [closure["name"]]:
                raise RuntimeError(
                    f"Segmentation ID {closure['segmentation_id']} is not unique: {owners}")
        from projectairsim.types import Pose
        observer_pose = camera_pose(scenario["observer_position_ned"],
                                    scenario["observer_target_ned"], Pose)
        accepted = backend.client.request({
            "method": backend.world.parent_topic +
                      "/robots/Observer/sensors/Overview/SetPose",
            "params": {"pose": observer_pose, "wait_for_pose_update": True},
            "version": 1.0,
        })
        if not accepted:
            raise RuntimeError("Observer camera pose rejected")
        await backend.ready(60)
        await backend.settle()
        write_json(output / "startup_pose_sync.json", await backend.wait_for_pose_sync())
        car = backend.pose("Car1")
        if np.linalg.norm(car["position"][:2] - scenario["start_ned"][:2]) > 4:
            raise RuntimeError("UGV did not settle at the selected road start")
        captures.append(save_camera(backend, "Car1/scene_camera",
                                    output / "street_start.png"))
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_start.png"))
        await backend.start()
        write_json(output / "post_takeoff_pose_sync.json",
                   await backend.wait_for_pose_sync())
        start_sim = backend.now()
        last_sim, last_progress = start_sim, time.monotonic()
        event("initial_route", start_sim, lane_ids=route_lanes,
              length_m=route_length_m)
        while backend.now() - start_sim < max_sim_seconds:
            now = backend.now()
            if now > last_sim:
                last_sim, last_progress = now, time.monotonic()
            if time.monotonic() - last_progress > 20 or time.monotonic() - start_wall > 2400:
                raise TimeoutError("District navigation simulation stopped progressing")
            car, drone = backend.pose("Car1"), backend.pose("Drone1")
            if previous_car is not None:
                car_distance_m += float(np.linalg.norm(
                    car["position"][:2] - previous_car[:2]))
            previous_car = car["position"].copy()
            new_collisions = collision_snapshot(backend, backend.kinds, collision_stamps)
            if new_collisions:
                collisions.extend(new_collisions)
                event("collision", now, impacts=[{
                    "robot": hit["robot"], "object": hit["event"].get("object_name")}
                    for hit in new_collisions])
                status = "collision"
                break
            car_speed = float(np.linalg.norm(car["velocity"][:2]))
            if len(decisions) and car_speed > 7:
                status = "unmodeled_impulse"
                event("unmodeled_impulse", now, speed_mps=car_speed,
                      position_ned=car["position"].tolist())
                break
            for closure in cfg["closures"]:
                segmentation_id = closure["segmentation_id"]
                if any(decision["segmentation_id"] == segmentation_id
                       for decision in decisions):
                    continue
                observation = backend.observation("Drone1", segmentation_id)
                distance_to_closure = float(np.linalg.norm(
                    drone["position"][:2] - closure["center_ned"][:2]))
                if closure["name"] not in scout_probes and distance_to_closure < 8:
                    scout_probes.add(closure["name"])
                    index = cfg["closures"].index(closure) + 1
                    prefix = output / f"scout_probe_{index}"
                    raw = save_observation_pair(
                        backend, "Drone1", prefix.with_name(prefix.name + "_raw.npz"))
                    frame = save_camera(backend, "Drone1/scene_camera",
                                        prefix.with_suffix(".png"))
                    if frame:
                        captures.append(frame)
                    event("scout_probe", now, closure_index=index,
                          distance_m=distance_to_closure,
                          diagnostics=backend.observation_diagnostics.get("Drone1"),
                          raw_observation=raw, camera=frame)
                if observation is None or observation["time"] <= last_seen[segmentation_id]:
                    continue
                last_seen[segmentation_id] = observation["time"]
                lane_id, lane_error = nearest_route_lane(
                    lanes, route_lanes, observation["position"])
                if lane_error > 4:
                    candidate_streak[segmentation_id] = 0
                    continue
                candidate_streak[segmentation_id] = (
                    candidate_streak[segmentation_id] + 1
                    if observation["time"] - candidate_time[segmentation_id] < 2 else 1)
                candidate_time[segmentation_id] = observation["time"]
                if candidate_streak[segmentation_id] < 2:
                    continue
                owners = [name for name, assigned in
                          backend.world.get_segmentation_id_map().items()
                          if assigned == segmentation_id]
                if owners != [closure["name"]]:
                    raise RuntimeError(
                        f"Segmentation ID {segmentation_id} changed ownership: {owners}")
                previous_lanes, previous_length = route_lanes, route_length_m
                proposed_blocked = [*blocked_lanes, lane_id]
                try:
                    proposed_lanes, proposed_length = route(
                        lanes, start, goal, proposed_blocked)
                except ValueError:
                    candidate_streak[segmentation_id] = 0
                    event("observation_rejected_no_detour", now,
                          segmentation_id=segmentation_id,
                          estimated_ned=observation["position"].tolist(),
                          matched_lane=lane_id, lane_match_error_m=lane_error)
                    continue
                route_lanes, route_length_m = proposed_lanes, proposed_length
                path = route_points(lanes, route_lanes)
                blocked_lanes = proposed_blocked
                number = len(decisions) + 1
                prefix = output / f"closure_{number}"
                packet = save_observation_pair(
                    backend, "Drone1", prefix.with_name(prefix.name + "_raw.npz"))
                frame = save_camera(backend, "Drone1/scene_camera",
                                    prefix.with_suffix(".png"))
                if packet is None or frame is None:
                    raise RuntimeError("Accepted closure observation could not be archived")
                captures.append(frame)
                captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                            output / f"junction_{number}.png"))
                decision = {
                    "index": number, "sim_time_sec": now,
                    "segmentation_id": segmentation_id,
                    "estimated_ned": observation["position"].tolist(),
                    "matched_lane": lane_id, "lane_match_error_m": lane_error,
                    "evaluation_lane": closure["evaluation_lane_id"],
                    "evaluation_error_m": float(np.linalg.norm(
                        observation["position"][:2] - closure["center_ned"][:2])),
                    "pixels": observation["pixels"],
                    "pair_delta_ms": observation["pair_delta_ms"],
                    "old_lane_ids": previous_lanes, "old_length_m": previous_length,
                    "new_lane_ids": route_lanes, "new_length_m": route_length_m,
                    "raw_observation": packet, "camera": frame,
                }
                decisions.append(decision)
                event("aerial_closure_observed_and_route_replanned", now,
                      decision_index=number, segmentation_id=segmentation_id,
                      matched_lane=lane_id, old_length_m=previous_length,
                      new_length_m=route_length_m)
            if (len(decisions) == 2 and
                    np.linalg.norm(car["position"][:2] -
                                   scenario["goal_ned"][:2]) < cfg["goal_tolerance_m"]):
                status = "completed"
                event("ugv_goal_reached", now, car_distance_m=car_distance_m)
                break
            if not decisions and now - start_sim > 120:
                status = "first_observation_missed"
                event("first_observation_missed", now,
                      diagnostics=backend.observation_diagnostics.get("Drone1"),
                      scout_probes=sorted(scout_probes))
                break
            car_error = cross_track(car["position"], path)
            max_cross_track = max(max_cross_track, car_error)
            if len(decisions) and car_error > cfg["max_cross_track_m"]:
                status = "off_route"
                event("off_route", now, cross_track_m=car_error)
                break
            if len(decisions) == 1 and car_distance_m >= 35 and not second_scout_released:
                second_scout_released = True
                event("second_scout_released_after_ugv_progress", now,
                      car_distance_m=car_distance_m)
            if len(decisions) == 1 and not second_scout_released:
                scout_target = drone["position"].copy()
            else:
                scout_target, _ = path_lookahead(path, drone["position"], lookahead=11)
            scout_target[2] = -26.0
            velocity = .7 * (scout_target - drone["position"])
            speed = float(np.linalg.norm(velocity))
            if speed > cfg["drone_speed_mps"]:
                velocity *= cfg["drone_speed_mps"] / speed
            if decisions:
                car_target, _ = path_lookahead(path, car["position"], lookahead=9)
                far_target, _ = path_lookahead(path, car["position"], lookahead=22)
                far_delta = far_target[:2] - car["position"][:2]
                far_heading = math.atan2(far_delta[1], far_delta[0])
                turn_ahead = math.atan2(math.sin(far_heading - car["yaw"]),
                                        math.cos(far_heading - car["yaw"]))
                turning = abs(turn_ahead) > .22
                current_speed = float(np.linalg.norm(car["velocity"][:2]))
                engine, steering, brake = rover_command(
                    car["position"], car["yaw"], car_target, current_speed,
                    desired_speed=1.1 if turning else cfg["car_speed_mps"])
                if turning:
                    steering = float(np.clip(steering * 2.8, -1.05, 1.05))
            else:
                car_target, engine, steering, brake = None, 0.0, 0.0, 1.0
                turn_ahead = 0.0
            await asyncio.gather(backend.drone_velocity("Drone1", velocity),
                                 backend.rover("Car1", engine, steering, brake))
            for name, pose, target in (("Car1", car, car_target),
                                       ("Drone1", drone, scout_target)):
                writer.writerow({
                    "sim_time_sec": now, "elapsed_sec": now - start_sim,
                    "robot": name, "x": pose["position"][0],
                    "y": pose["position"][1], "z": pose["position"][2],
                    "yaw_rad": pose["yaw"],
                    "speed_mps": np.linalg.norm(pose["velocity"][:2]),
                    "goal_x": target[0] if target is not None else "",
                    "goal_y": target[1] if target is not None else "",
                    "goal_z": target[2] if target is not None else "",
                    "engine": engine if name == "Car1" else "",
                    "steering": steering if name == "Car1" else "",
                    "brake": brake if name == "Car1" else "",
                    "cross_track_m": car_error if name == "Car1" else "",
                    "turn_ahead_rad": turn_ahead if name == "Car1" else "",
                    "route_version": len(decisions),
                    "phase": "drive" if decisions else "aerial_scout",
                })
            stream.flush()
            await asyncio.sleep(1 / cfg["control_hz"])
        if status == "failed":
            status = "timeout"
        captures.append(save_camera(backend, "Car1/scene_camera",
                                    output / "street_end.png"))
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
        events.close()
        receipt(output, status, error=error, decisions=decisions,
                blocked_lanes=blocked_lanes, car_distance_m=car_distance_m,
                max_cross_track_m=max_cross_track, collision_events=collisions,
                captures=[capture for capture in captures if capture],
                wall_duration_sec=time.monotonic() - start_wall,
                cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status,
                      "decisions": len(decisions), "car_distance_m": car_distance_m,
                      "error": error}), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-sim-seconds", type=float, default=800)
    args = parser.parse_args()
    raise SystemExit(0 if asyncio.run(run(args.max_sim_seconds)) == "completed" else 1)


if __name__ == "__main__":
    main()
