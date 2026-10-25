"""Matched urban road-closure runs: ground discovery or aerial advance notice."""

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
from urban_scene.road_graph import RoadGraph, rounded_xy
from urban_scene.urban_runtime import (
    MESH, UrbanBackend, collision_snapshot, new_run, receipt,
    save_camera, save_observation_pair, sha256,
)


def cross_track(xy, path):
    starts = path[:-1, :2]
    segments = np.diff(path[:, :2], axis=0)
    fraction = np.clip(np.sum((xy - starts) * segments, axis=1) /
                       np.maximum(np.sum(segments * segments, axis=1), 1e-9), 0, 1)
    return float(np.linalg.norm(starts + fraction[:, None] * segments - xy, axis=1).min())


def road_reference(graph, nodes):
    xy = rounded_xy(graph.xy(nodes), radius=5.0)
    if "281277367" in nodes and "53071662" in nodes:
        # Keep the eastbound robot in a measured clear lane beside the sign pole.
        eastbound = (xy[:, 0] > -61.0) & (xy[:, 1] > 80.0) & (xy[:, 1] < 92.0)
        weight = np.clip((xy[:, 0] + 61.0) / 11.0, 0.0, 1.0)
        xy[eastbound, 1] += 1.85 * weight[eastbound]
    return np.column_stack((xy, np.zeros(len(xy))))


def config_for(mode):
    return {
        "task": "urban_navigation", "mode": mode,
        "scene": "scene_navigation_air.jsonc" if mode == "air" else "scene_navigation_ground.jsonc",
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "sensor_max_age_sec": 1.5, "sensor_pair_tolerance_sec": .05,
        "tracking": {"minimum_pixels": 30, "segmentation_color_order": "bgr"},
        "closure": {"center_ned": [-12.30, -42.0, -1.55],
                    "size_m": [11.0, 2.5, 3.1], "segmentation_id": 75},
        "control_hz": 4.0, "car_speed_mps": 1.4,
        "drone_speed_mps": 3.0, "max_sim_seconds": 500.0,
        "goal_tolerance_m": 2.0,
        "perception_acceptance_m": 8.0,
        "eastbound_lane_offset_m": 1.85,
        "collision_policy": "Stop on non-ground contact",
        "road_model": "Undirected OSM public-road centerline graph; traffic one-way rules are not modeled",
    }


async def run(mode, max_sim_seconds=None):
    graph = RoadGraph()
    candidate = graph.report["candidate_route"]["osm_node_ids"]
    start_node, goal_node = candidate[23], candidate[30]
    closure_edge = (candidate[28], candidate[29])
    route, open_length = graph.shortest(start_node, goal_node)
    diverted_route, diverted_length = graph.shortest(start_node, goal_node, [closure_edge])
    cfg = config_for(mode)
    if max_sim_seconds is not None:
        cfg["max_sim_seconds"] = max_sim_seconds
    output = new_run("urban_navigation", mode)
    scene_path = HERE / cfg["scene"]
    cfg["scenario"] = {
        "start_osm_node": start_node, "goal_osm_node": goal_node,
        "closure_osm_edge": closure_edge,
        "open_route_length_m": open_length,
        "diverted_route_length_m": diverted_length,
        "open_osm_node_ids": route, "diverted_osm_node_ids": diverted_route,
    }
    write_json(output / "config.json", cfg)
    write_json(output / "provenance.json", {
        "scene_config_sha256": sha256(scene_path),
        "osm_sha256": sha256(HERE / "source/south_lake_union_clipped.osm"),
        "road_report_sha256": sha256(HERE / "generated/road_network_report.json"),
        "runtime_glb_sha256": sha256(MESH),
        "control_sha256": sha256(__file__),
        "graph_sha256": sha256(HERE / "road_graph.py"),
        "backend_sha256": sha256(HERE / "urban_runtime.py"),
    })
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    for source in (__file__, scene_path, HERE / "road_graph.py",
                   HERE / "urban_runtime.py", ROOT / "ag/backend.py",
                   ROOT / "ag/planning.py", ROOT / "ag/study_planning.py",
                   ROOT / "ag/math3d.py", ROOT / "ag/contact.py"):
        source = Path(source)
        (source_dir / source.name).write_bytes(source.read_bytes())
    backend = UrbanBackend(cfg)
    status, error = "failed", None
    captures, detection = [], None
    rejected_candidates = 0
    candidate_streak = 0
    candidate_last_time = -math.inf
    candidate_last_position = None
    first_pose, last_pose = None, None
    path = road_reference(graph, route)
    active_route = route
    last_collisions = {}
    obstacles = []
    distance_m = 0.0
    car_released = mode == "ground"
    recovery_state = None
    recovery_used = False
    recovery_started_at = None
    air_waypoints = np.asarray([
        [-59.2, -62.0, -35.0], [-12.0, -62.0, -35.0],
        [-12.3, -42.0, -28.0],
    ])
    air_index = 0
    route_replanned_at = None
    last_observation_time = -math.inf
    start_wall = time.monotonic()
    run_stream = (output / "samples.csv").open("w", encoding="utf-8", newline="")
    fields = ["sim_time_sec", "elapsed_sec", "robot", "x", "y", "z", "yaw_rad",
              "speed_mps", "goal_x", "goal_y", "goal_z", "engine", "steering",
              "brake", "vx", "vy", "vz", "cross_track_m", "route_state"]
    writer = csv.DictWriter(run_stream, fieldnames=fields)
    writer.writeheader()
    events_stream = (output / "events.jsonl").open("w", encoding="utf-8")
    def event(kind, sim_time, **details):
        events_stream.write(json.dumps({"kind": kind, "sim_time_sec": sim_time, **details}) + "\n")
        events_stream.flush()

    try:
        backend.connect()
        scene = backend.load()
        (output / "closure_barrier.glb").write_bytes(backend.closure_payload)
        write_json(output / "loaded_scene.json", scene)
        write_json(output / "live_objects.json", {
            "mesh": backend.mesh_actor, "closure": backend.closure_actor,
            "actors": backend.world.list_actors(),
        })
        await backend.ready(45)
        await backend.settle()
        write_json(output / "startup_pose_sync.json", await backend.wait_for_pose_sync())
        initial = backend.pose("Car1")["position"]
        if np.linalg.norm(initial[:2] - graph.points[start_node]) > 4.0:
            raise RuntimeError("Car1 did not settle at the declared OSM start node")
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_start.png"))
        await backend.start()
        write_json(output / "post_takeoff_pose_sync.json", await backend.wait_for_pose_sync())
        start_sim = backend.now()
        last_tick = time.monotonic()
        last_progress = last_tick
        last_sim = start_sim
        while backend.now() - start_sim < cfg["max_sim_seconds"]:
            tick = time.monotonic()
            now = backend.now()
            if now > last_sim:
                last_progress, last_sim = tick, now
            if tick - last_progress > 12 or tick - start_wall > 1200:
                raise TimeoutError("Simulator clock or wall-time watchdog expired")
            car = backend.pose("Car1")
            if first_pose is None:
                first_pose = car["position"].copy()
            if last_pose is not None:
                distance_m += float(np.linalg.norm(car["position"][:2] - last_pose[:2]))
            last_pose = car["position"].copy()
            if abs(car["position"][2]) > 2.0:
                raise RuntimeError(f"Car1 left road-height band: NED z={car['position'][2]:.2f}")
            collisions = collision_snapshot(backend, backend.kinds, last_collisions)
            for collision in collisions:
                obstacles.append(collision)
                event("obstacle_collision", now, robot=collision["robot"],
                      actor=collision["event"].get("object_name"))
            if collisions:
                status = "obstacle_collision"
                break

            observer = "Drone1" if mode == "air" else "Car1"
            observation = backend.observation(observer, cfg["closure"]["segmentation_id"])
            if (observation is not None and detection is None and
                    observation["time"] > last_observation_time):
                last_observation_time = observation["time"]
                estimated = observation["position"]
                edge, centerline_error = graph.nearest_edge(estimated, math.inf)
                landmark_error = float(np.linalg.norm(estimated[:2] -
                                                      np.asarray(cfg["closure"]["center_ned"])[:2]))
                if (frozenset(edge) != frozenset(closure_edge) or
                        centerline_error >= cfg["perception_acceptance_m"]):
                    candidate_streak = 0
                    rejected_candidates += 1
                    if rejected_candidates == 1:
                        save_observation_pair(backend, observer,
                                              output / "first_rejected_observation_raw.npz")
                        captures.append(save_camera(backend, observer + "/scene_camera",
                                                    output / "first_rejected_observation.png"))
                    if rejected_candidates <= 8 or rejected_candidates % 20 == 0:
                        event("candidate_rejected", now, observer=observer,
                              estimated_ned=estimated.tolist(), pixels=observation["pixels"],
                              nearest_edge=edge, centerline_error_m=centerline_error,
                              landmark_error_m=landmark_error)
                else:
                    if (candidate_last_position is not None and
                            observation["time"] - candidate_last_time <= 1.5 and
                            np.linalg.norm(estimated[:2] - candidate_last_position[:2]) < 4.0):
                        candidate_streak += 1
                    else:
                        candidate_streak = 1
                    candidate_last_time = observation["time"]
                    candidate_last_position = estimated.copy()
                    if candidate_streak < 3:
                        event("candidate_pending", now, observer=observer,
                              estimated_ned=estimated.tolist(), streak=candidate_streak)
                        continue
                    detection = {
                        "robot": observer, "time_sec": observation["time"],
                        "estimated_ned": estimated.tolist(), "pixels": observation["pixels"],
                        "pair_delta_ms": observation["pair_delta_ms"],
                        "map_matched_edge": edge, "centerline_error_m": centerline_error,
                        "landmark_error_m_evaluation_only": landmark_error,
                    }
                    save_observation_pair(backend, observer, output / "closure_detection_raw.npz")
                    captures.append(save_camera(backend, observer + "/scene_camera",
                                                output / "closure_detection.png"))
                    event("closure_observed", now, **detection)
                    current_node = graph.nearest_node(car["position"][:2], maximum_distance=18.0)
                    if mode == "ground" and car["position"][0] > -63.0:
                        active_route = diverted_route
                        recovery_state = "brake_before_reverse"
                        recovery_used = True
                        recovery_started_at = now
                        event("reverse_recovery_started", now,
                              car_pose_ned=car["position"].tolist(),
                              reason="Passed the last available northbound branch")
                    else:
                        active_route, _ = graph.shortest(current_node, goal_node, [edge])
                    path = road_reference(graph, active_route)
                    route_replanned_at = now
                    car_released = True
                    event("road_replanned", now, current_osm_node=current_node,
                          node_ids=active_route)

            if mode == "air" and detection is None and now - start_sim > 90:
                status = "no_aerial_closure_detection"
                break
            if np.linalg.norm(car["position"][:2] - path[-1, :2]) < cfg["goal_tolerance_m"]:
                status = "completed" if detection is not None else "goal_without_closure_detection"
                break
            deviation = cross_track(car["position"][:2], path)
            if car_released and recovery_state is None and deviation > 7.0:
                status = "off_route"
                break
            if recovery_state is not None:
                if (now - recovery_started_at > 75 or car["position"][0] > -44 or
                        abs(car["position"][1] + 62.0) > 4):
                    status = "reverse_recovery_failed"
                    break
                speed = float(np.linalg.norm(car["velocity"][:2]))
                if recovery_state == "brake_before_reverse" and speed < .15:
                    recovery_state = "reverse"
                    event("reverse_motion_started", now)
                if recovery_state == "reverse" and car["position"][0] < -69.0:
                    recovery_state = "brake_after_reverse"
                    event("reverse_target_reached", now,
                          car_pose_ned=car["position"].tolist())
                if recovery_state == "brake_after_reverse" and speed < .15:
                    recovery_state = None
                    event("reverse_recovery_completed", now,
                          car_pose_ned=car["position"].tolist())
                if recovery_state == "reverse":
                    target, engine, steering, brake = None, -.055, 0.0, 0.0
                else:
                    target, engine, steering, brake = None, 0.0, 0.0, 1.0
            elif car_released:
                target, _ = path_lookahead(path, car["position"], lookahead=6.0)
                speed = float(np.linalg.norm(car["velocity"][:2]))
                engine, steering, brake = rover_command(
                    car["position"], car["yaw"], target, speed,
                    desired_speed=cfg["car_speed_mps"])
            else:
                target, engine, steering, brake = None, 0.0, 0.0, 1.0
            writer.writerow({"sim_time_sec": now, "elapsed_sec": now - start_sim,
                             "robot": "Car1", "x": car["position"][0],
                             "y": car["position"][1], "z": car["position"][2],
                             "yaw_rad": car["yaw"], "speed_mps": np.linalg.norm(car["velocity"][:2]),
                             "goal_x": target[0] if target is not None else "",
                             "goal_y": target[1] if target is not None else "",
                             "goal_z": target[2] if target is not None else "",
                             "engine": engine, "steering": steering, "brake": brake,
                             "vx": "", "vy": "", "vz": "", "cross_track_m": deviation,
                             "route_state": recovery_state or ("diverted" if detection else "open")})
            controls = [backend.rover("Car1", engine, steering, brake)]
            if mode == "air":
                drone = backend.pose("Drone1")
                if air_index < len(air_waypoints) - 1 and np.linalg.norm(
                        drone["position"] - air_waypoints[air_index]) < 2.5:
                    air_index += 1
                air_goal = air_waypoints[air_index]
                velocity = .7 * (air_goal - drone["position"])
                norm = np.linalg.norm(velocity)
                if norm > cfg["drone_speed_mps"]:
                    velocity *= cfg["drone_speed_mps"] / norm
                writer.writerow({"sim_time_sec": now, "elapsed_sec": now - start_sim,
                                 "robot": "Drone1", "x": drone["position"][0],
                                 "y": drone["position"][1], "z": drone["position"][2],
                                 "yaw_rad": drone["yaw"], "speed_mps": np.linalg.norm(drone["velocity"]),
                                 "goal_x": air_goal[0], "goal_y": air_goal[1], "goal_z": air_goal[2],
                                 "engine": "", "steering": "", "brake": "",
                                 "vx": velocity[0], "vy": velocity[1], "vz": velocity[2],
                                 "cross_track_m": "", "route_state": "scouting"})
                controls.append(backend.drone_velocity("Drone1", velocity))
            await asyncio.gather(*controls)
            if tick - last_tick >= 2:
                run_stream.flush()
                last_tick = tick
            await asyncio.sleep(max(0.0, 1 / cfg["control_hz"] - (time.monotonic() - tick)))
        else:
            status = "timeout"
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_end.png"))
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(error, flush=True)
    finally:
        try:
            cleanup = await backend.cleanup()
        except Exception as exc:
            cleanup = [f"{type(exc).__name__}: {exc}"]
        run_stream.close()
        events_stream.close()
        receipt(output, status, error=error, cleanup_errors=cleanup,
                initial_pose_ned=first_pose.tolist() if first_pose is not None else None,
                last_pose_ned=last_pose.tolist() if last_pose is not None else None,
                car_distance_m=distance_m, detection=detection,
                rejected_observation_candidates=rejected_candidates,
                confirmed_candidate_streak=candidate_streak,
                detection_to_replan_sec=(route_replanned_at - detection["time_sec"]
                                         if detection and route_replanned_at else None),
                obstacle_collisions=obstacles, active_route_osm_nodes=active_route,
                active_goal_ned=path[-1].tolist(),
                recovery_used=recovery_used, recovery_final_state=recovery_state,
                captures=[item for item in captures if item],
                wall_duration_sec=time.monotonic() - start_wall)
    print(json.dumps({"output": str(output), "status": status, "error": error}, ensure_ascii=False), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["ground", "air"])
    parser.add_argument("--max-sim-seconds", type=float,
                        help="Diagnostic time cap; capped runs remain incomplete unless the goal is reached")
    args = parser.parse_args()
    status = asyncio.run(run(args.mode, args.max_sim_seconds))
    raise SystemExit(0 if status == "completed" else 1)


if __name__ == "__main__":
    main()
