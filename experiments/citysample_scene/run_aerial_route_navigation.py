"""Survey three City Sample intersections from a UAV, then route a UGV."""

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
from ag.portable import sdk_path, zonegraph_path, apply_runtime
from ag.study_planning import path_lookahead
from citysample_runtime import CitySampleBackend
from district_routes import cross_track, load_lanes, route, route_points
from urban_scene.probe_urban_scene import camera_pose
from urban_scene.urban_runtime import (
    collision_snapshot, new_run, receipt, save_camera,
    save_observation_pair, sha256,
)


def clamp_velocity(delta, maximum):
    velocity = np.asarray(delta, dtype=float) * .65
    norm = float(np.linalg.norm(velocity))
    return velocity * min(1.0, maximum / max(norm, 1e-9))


def check_aerial_depth(packet_path, junction):
    from projectairsim.utils import unpack_image

    with np.load(packet_path) as packet:
        metadata = json.loads(packet["metadata_json"].item())["depth"]
        if metadata["encoding"] != "16UC1":
            raise RuntimeError("Unexpected aerial depth encoding")
        message = {**metadata, "data": packet["depth_payload"].tobytes()}
        depth_m = unpack_image(message).astype(float) / 1000
    height, width = depth_m.shape
    center = depth_m[height // 2 - 10:height // 2 + 10,
                     width // 2 - 10:width // 2 + 10]
    valid = center[(center > 0) & (center < 65.5)]
    expected_m = junction["center_ned"][2] - metadata["pos_z"]
    result = {
        "valid_center_fraction": float(valid.size / center.size),
        "median_center_depth_m": float(np.median(valid)) if valid.size else None,
        "expected_road_depth_m": float(expected_m),
    }
    if result["valid_center_fraction"] < .8:
        raise RuntimeError(f"UAV depth unavailable at junction: {result}")
    difference = expected_m - result["median_center_depth_m"]
    result["surface_height_above_road_m"] = float(difference)
    result["visibility"] = ("overhead_occlusion" if difference > 4 else
                            "road_surface_visible" if abs(difference) <= 4 else
                            "height_mismatch")
    return result


async def run(max_sim_seconds):
    scenario_path = HERE / "generated/aerial_route_scenario.json"
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    source = zonegraph_path()
    if sha256(source) != scenario["zonegraph_sha256"]:
        raise RuntimeError("City Sample ZoneGraph changed after route preparation")
    lanes = load_lanes(source)
    candidate_lanes, candidate_length = route(
        lanes, scenario["start_lane"], scenario["goal_lane"])
    if candidate_lanes != scenario["candidate_lane_ids"]:
        raise RuntimeError("Candidate road route changed after preparation")
    cfg = {
        "task": "citysample_aerial_route_navigation", "mode": "air_ground",
        "scene": "generated/scene_navigation_aerial_route.jsonc",
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(sdk_path()),
        "sensor_max_age_sec": 2.0, "sensor_pair_tolerance_sec": .06,
        "control_hz": 3.0, "drone_speed_mps": 8.0,
        "car_straight_speed_mps": 2.0, "car_turn_speed_mps": .8,
        "goal_tolerance_m": 4.0, "max_cross_track_m": 10.0,
        "survey_altitude_ned_m": scenario["survey_altitude_ned_m"],
        "depth_survey_altitude_ned_m": -50.0,
        "max_sim_seconds": max_sim_seconds,
        "ambient_traffic": "disabled by -ControlledTraffic startup option",
        "route_policy": "UGV waits while the UAV surveys three junctions; the planner records aerial visibility and uses the ground-level directed ZoneGraph route beneath any overhead occlusion",
        "perception_scope": "High-altitude RGB and near-altitude depth classify junction visibility; the route is computed on City Sample's native road graph",
        "ugv_contact_proxy_origin_ned_m": scenario["ugv_contact_proxy_origin_ned_m"],
        "ugv_physics_scope": scenario["ugv_physics_scope"],
    }
    output = new_run("citysample_aerial_route_navigation", "air_ground")
    write_json(output / "config.json", cfg)
    write_json(output / "scenario.json", scenario)
    sources = [__file__, HERE / "prepare_aerial_route_navigation.py",
               HERE / "district_routes.py", HERE / "citysample_runtime.py",
               HERE / cfg["scene"], scenario_path,
               ROOT / "configs/robot_ugv_navigation_study.jsonc",
               HERE / "generated/robot_ugv_aerial_route.jsonc",
               ROOT / "configs/robot_uav_navigation_study.jsonc",
               ROOT / "urban_scene/probe_urban_scene.py",
               ROOT / "urban_scene/urban_runtime.py", ROOT / "ag/backend.py",
               ROOT / "ag/planning.py", ROOT / "ag/study_planning.py"]
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for path in sources:
        (snapshot / Path(path).name).write_bytes(Path(path).read_bytes())
    write_json(output / "provenance.json", {
        "source_sha256": {Path(path).name: sha256(path) for path in sources},
        "zonegraph_sha256": sha256(source),
        "map_asset": "/Game/Map/Big_City_LVL",
        "game_mode": "ProjectAirSimGameMode",
    })
    cfg = apply_runtime(cfg)
    backend = CitySampleBackend(cfg)
    status, error = "failed", None
    survey, crossings, captures, collisions = [], [], [], []
    survey_phase, high_frame = "overhead", None
    collision_stamps = {}
    committed_lanes, path = None, None
    first_car, last_car = None, None
    car_distance_m, max_cross_track_m = 0.0, 0.0
    max_observed_step_speed_mps = 0.0
    previous_car_time, last_car_motion_sim = None, None
    start_wall = time.monotonic()
    fields = ["sim_time_sec", "elapsed_sec", "robot", "x", "y", "z",
              "yaw_rad", "speed_mps", "goal_x", "goal_y", "goal_z",
              "engine", "steering", "brake", "cross_track_m", "phase"]
    stream = (output / "trajectories_controls.csv").open(
        "w", newline="", encoding="utf-8")
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
        if set(backend.kinds) != {"Car1", "Drone1"}:
            raise RuntimeError("Both UAV and UGV must be loaded")
        from projectairsim.types import Pose

        async def place_observer(position, target):
            pose = camera_pose(position, target, Pose)
            accepted = backend.client.request({
                "method": backend.world.parent_topic +
                          "/robots/Observer/sensors/Overview/SetPose",
                "params": {"pose": pose, "wait_for_pose_update": True},
                "version": 1.0,
            })
            if not accepted:
                raise RuntimeError("Observer camera pose rejected")
            request_time = backend.now()
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                message = backend.buffer.latest("Observer/Overview/scene_camera", 2)
                if (message is not None and
                        message["time_stamp"] / 1e9 >= request_time + 1.0 and
                        np.linalg.norm(np.array([
                            message[f"pos_{axis}"] for axis in "xyz"]) -
                            np.asarray(position)) < .2):
                    return
                await asyncio.sleep(.1)
            raise RuntimeError("Observer frame did not settle at requested pose")

        await place_observer(scenario["observer_position_ned"],
                             scenario["observer_target_ned"])
        await backend.ready(60)
        await backend.settle()
        car = backend.pose("Car1")
        if np.linalg.norm(car["position"][:2] - scenario["start_ned"][:2]) > 4:
            raise RuntimeError("UGV did not settle at the selected road start")
        captures.append(save_camera(backend, "Car1/scene_camera",
                                    output / "street_start.png"))
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_start.png"))
        await backend.start()
        write_json(output / "pose_sync.json", await backend.wait_for_pose_sync())
        start_sim = backend.now()
        last_sim, last_progress = start_sim, time.monotonic()
        event("candidate_road_route", start_sim,
              lane_ids=candidate_lanes, length_m=candidate_length,
              expected_driven_length_m=scenario["expected_driven_length_m"])
        while backend.now() - start_sim < max_sim_seconds:
            now = backend.now()
            if now > last_sim:
                last_sim, last_progress = now, time.monotonic()
            if time.monotonic() - last_progress > 20 or time.monotonic() - start_wall > 2400:
                raise TimeoutError("City Sample simulation stopped progressing")
            car, drone = backend.pose("Car1"), backend.pose("Drone1")
            if first_car is None:
                first_car = car["position"].copy()
            previous_car = last_car.copy() if last_car is not None else None
            if last_car is not None:
                step_distance = float(np.linalg.norm(
                    car["position"][:2] - last_car[:2]))
                car_distance_m += step_distance
                if step_distance > .05:
                    last_car_motion_sim = now
                elapsed = now - previous_car_time
                if elapsed > 0:
                    max_observed_step_speed_mps = max(
                        max_observed_step_speed_mps, step_distance / elapsed)
            last_car = car["position"].copy()
            previous_car_time = now
            impacts = collision_snapshot(backend, backend.kinds, collision_stamps)
            if impacts:
                collisions.extend(impacts)
                event("collision", now, impacts=[{
                    "robot": item["robot"],
                    "object": item["event"].get("object_name")}
                    for item in impacts])
                status = "collision"
                break
            car_speed = float(np.linalg.norm(car["velocity"][:2]))
            if (committed_lanes is not None and
                    (car_speed > 7 or max_observed_step_speed_mps > 7)):
                status = "unexplained_pose_jump"
                event("unexplained_pose_jump", now, speed_mps=car_speed,
                      max_observed_step_speed_mps=max_observed_step_speed_mps,
                      previous_position_ned=previous_car.tolist()
                      if previous_car is not None else None,
                      current_position_ned=car["position"].tolist(),
                      latest_collision_packet=backend.collision("Car1"))
                break

            if len(survey) < len(scenario["survey_junctions"]):
                junction = (survey[-1]["junction"] if survey_phase == "ascent"
                            else scenario["survey_junctions"][len(survey)])
                drone_target = np.array(junction["center_ned"], dtype=float)
                drone_target[2] = (cfg["depth_survey_altitude_ned_m"]
                                   if survey_phase == "depth" else
                                   cfg["survey_altitude_ned_m"])
                velocity = clamp_velocity(drone_target - drone["position"],
                                          cfg["drone_speed_mps"])
                if (np.linalg.norm(drone["position"][:2] - drone_target[:2]) < 8
                        and abs(drone["position"][2] - drone_target[2]) < 5):
                    index = len(survey) + 1
                    if survey_phase == "overhead":
                        high_frame = save_camera(
                            backend, "Drone1/scene_camera",
                            output / f"aerial_junction_{index}.png")
                        if high_frame is None:
                            raise RuntimeError(f"Missing high UAV RGB frame at junction {index}")
                        captures.append(high_frame)
                        survey_phase = "depth"
                    elif survey_phase == "depth":
                        near_frame = save_camera(
                            backend, "Drone1/scene_camera",
                            output / f"aerial_junction_{index}_near.png")
                        raw = save_observation_pair(
                            backend, "Drone1",
                            output / f"aerial_junction_{index}_depth_segmentation.npz")
                        if near_frame is None or raw is None:
                            raise RuntimeError(f"Missing near UAV sensors at junction {index}")
                        assessment = check_aerial_depth(raw["file"], junction)
                        captures.append(near_frame)
                        survey.append({"junction": junction,
                                       "drone_pose_ned": drone["position"].tolist(),
                                       "frame": high_frame, "near_frame": near_frame,
                                       "depth_segmentation": raw,
                                       "road_surface_assessment": assessment,
                                       "sim_time_sec": now})
                        event("aerial_junction_observed", now,
                              junction_index=index, lane_id=junction["lane_id"],
                              frame=high_frame, near_frame=near_frame,
                              depth_segmentation=raw,
                              road_surface_assessment=assessment)
                        survey_phase, high_frame = "ascent", None
                    else:
                        survey_phase = "overhead"
                engine, steering, brake = 0.0, 0.0, 1.0
                car_target = None
                phase = "aerial_survey"
                car_error = 0.0
            else:
                if committed_lanes is None:
                    if any(item["road_surface_assessment"]["visibility"] ==
                           "height_mismatch" for item in survey):
                        raise RuntimeError(
                            "Aerial depth disagrees with a road layer; route not committed")
                    committed_lanes, length_m = route(
                        lanes, scenario["start_lane"], scenario["goal_lane"])
                    if committed_lanes != candidate_lanes:
                        raise RuntimeError("Road route changed during aerial survey")
                    path = route_points(lanes, committed_lanes)
                    write_json(output / "committed_plan.json", {
                        "method": "directed shortest path on City Sample ZoneGraph",
                        "aerial_images_recorded_before_commit": [
                            item["frame"] for item in survey],
                        "aerial_depth_segmentation_recorded_before_commit": [
                            item["depth_segmentation"] for item in survey],
                        "aerial_junction_visibility": [
                            item["road_surface_assessment"] for item in survey],
                        "lane_ids": committed_lanes, "length_m": length_m,
                        "start_offset_on_lane_m": scenario["start_offset_on_lane_m"],
                        "expected_driven_length_m": scenario["expected_driven_length_m"],
                        "waypoints_ned": path.tolist(),
                        "committed_sim_time_sec": now,
                    })
                    event("aerial_route_committed", now,
                          lane_ids=committed_lanes, length_m=length_m,
                          expected_driven_length_m=scenario["expected_driven_length_m"])
                    last_car_motion_sim = now
                    first_junction = scenario["survey_junctions"][0]
                    center = first_junction["center_ned"]
                    await place_observer([center[0], center[1], -75.0],
                                         [center[0], center[1], 0.0])
                drone_target = np.array(
                    scenario["survey_junctions"][-1]["center_ned"], dtype=float)
                drone_target[2] = cfg["survey_altitude_ned_m"]
                velocity = clamp_velocity(drone_target - drone["position"],
                                          cfg["drone_speed_mps"])
                if (len(crossings) == 3 and
                        np.linalg.norm(car["position"][:2] -
                                       scenario["goal_ned"][:2]) <
                        cfg["goal_tolerance_m"]):
                    status = "completed"
                    event("ugv_goal_reached", now,
                          car_distance_m=car_distance_m)
                    break
                car_error = cross_track(car["position"], path)
                max_cross_track_m = max(max_cross_track_m, car_error)
                if car_error > cfg["max_cross_track_m"]:
                    status = "off_route"
                    event("off_route", now, cross_track_m=car_error)
                    break
                car_target, _ = path_lookahead(path, car["position"],
                                               lookahead=12.0)
                far_target, _ = path_lookahead(path, car["position"],
                                               lookahead=20.0)
                far_delta = far_target[:2] - car["position"][:2]
                heading = math.atan2(far_delta[1], far_delta[0])
                turn_ahead = math.atan2(math.sin(heading - car["yaw"]),
                                        math.cos(heading - car["yaw"]))
                desired_speed = (cfg["car_turn_speed_mps"]
                                 if abs(turn_ahead) > .17 else
                                 cfg["car_straight_speed_mps"])
                engine, steering, brake = rover_command(
                    car["position"], car["yaw"], car_target, car_speed,
                    desired_speed=desired_speed, max_engine=.15)
                if engine > .04 and now - last_car_motion_sim > 15:
                    status = "ugv_stalled"
                    event("ugv_stalled", now,
                          car_distance_m=car_distance_m,
                          current_position_ned=car["position"].tolist())
                    break
                phase = "ugv_route_execution"
                if len(crossings) < 3:
                    junction = scenario["survey_junctions"][len(crossings)]
                    if np.linalg.norm(car["position"][:2] -
                                      junction["center_ned"][:2]) < 12:
                        index = len(crossings) + 1
                        frame = save_camera(
                            backend, "Observer/Overview/scene_camera",
                            output / f"ugv_junction_{index}.png")
                        if frame is None:
                            raise RuntimeError(
                                f"Missing external UGV frame at junction {index}")
                        captures.append(frame)
                        crossing = {"junction": junction,
                                    "car_pose_ned": car["position"].tolist(),
                                    "frame": frame, "sim_time_sec": now,
                                    "car_distance_m": car_distance_m}
                        crossings.append(crossing)
                        event("ugv_junction_crossed", now,
                              junction_index=index,
                              car_distance_m=car_distance_m, frame=frame)
                        if index < 3:
                            next_center = scenario["survey_junctions"][index]["center_ned"]
                            await place_observer(
                                [next_center[0], next_center[1], -75.0],
                                [next_center[0], next_center[1], 0.0])

            await asyncio.gather(
                backend.drone_velocity("Drone1", velocity),
                backend.rover("Car1", engine, steering, brake),
            )
            for name, pose, target in (("Car1", car, car_target),
                                       ("Drone1", drone, drone_target)):
                writer.writerow({
                    "sim_time_sec": now, "elapsed_sec": now - start_sim,
                    "robot": name,
                    "x": pose["position"][0], "y": pose["position"][1],
                    "z": pose["position"][2], "yaw_rad": pose["yaw"],
                    "speed_mps": np.linalg.norm(pose["velocity"][:2]),
                    "goal_x": target[0] if target is not None else "",
                    "goal_y": target[1] if target is not None else "",
                    "goal_z": target[2] if target is not None else "",
                    "engine": engine if name == "Car1" else "",
                    "steering": steering if name == "Car1" else "",
                    "brake": brake if name == "Car1" else "",
                    "cross_track_m": car_error if name == "Car1" else "",
                    "phase": phase,
                })
            stream.flush()
            await asyncio.sleep(1 / cfg["control_hz"])
        if status == "failed":
            status = "timeout"
        captures.append(save_camera(backend, "Car1/scene_camera",
                                    output / "street_end.png"))
        await place_observer(scenario["observer_position_ned"],
                             scenario["observer_target_ned"])
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
        receipt(output, status, error=error, survey=survey,
                junction_crossings=crossings, committed_lanes=committed_lanes,
                first_car_pose_ned=(first_car.tolist()
                                    if first_car is not None else None),
                last_car_pose_ned=(last_car.tolist()
                                   if last_car is not None else None),
                car_distance_m=car_distance_m,
                max_cross_track_m=max_cross_track_m,
                max_observed_step_speed_mps=max_observed_step_speed_mps,
                collision_events=collisions,
                captures=[item for item in captures if item],
                wall_duration_sec=time.monotonic() - start_wall,
                cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status,
                      "surveyed_junctions": len(survey),
                      "ugv_crossings": len(crossings),
                      "car_distance_m": car_distance_m,
                      "error": error}), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-sim-seconds", type=float, default=700)
    args = parser.parse_args()
    raise SystemExit(0 if asyncio.run(run(args.max_sim_seconds)) == "completed" else 1)


if __name__ == "__main__":
    main()
