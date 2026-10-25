"""Two-UAV, two-interceptor road containment in the independent Bellevue scene."""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
from pathlib import Path
import random
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from ag.planning import Tracker, rover_command
from ag.storage import write_json
from ag.study_planning import path_lookahead
from urban_scene.road_graph import RoadGraph, rounded_xy
from urban_scene.urban_runtime import (
    UrbanBackend, collision_snapshot, new_run, receipt, save_camera,
    save_observation_pair, sha256,
)


BELLEVUE_OSM = HERE / "source/bellevue_downtown_clipped.osm"
BELLEVUE_ROADS = HERE / "generated/bellevue_road_network_report.json"
BELLEVUE_MESH = HERE / "generated/bellevue_downtown_runtime_512.glb"
TARGET_ID = 210
NORTH_ID = 211
SOUTH_ID = 212
NORTH_GATE = np.array([11.1, -43.0, 0.0])
SOUTH_GATE = np.array([10.7, -82.0, 0.0])


def blocker_candidate(observation, target, junction, gate, settings):
    """Check a front-camera detection against the public road and exit geometry."""
    if observation is None:
        return False
    position = np.asarray(observation["position"][:2], dtype=float)
    ego = np.asarray(target["position"][:2], dtype=float)
    if not np.all(np.isfinite(position)):
        return False
    delta = position - ego
    heading = np.array([math.cos(target["yaw"]), math.sin(target["yaw"])])
    forward = float(np.dot(delta, heading))
    lateral = abs(float(heading[0] * delta[1] - heading[1] * delta[0]))
    return (np.linalg.norm(ego - junction[:2]) < settings["junction_activation_m"]
            and np.linalg.norm(position - gate[:2]) < settings["gate_observation_tolerance_m"]
            and 0 < forward < settings["blocker_stop_range_m"]
            and lateral < settings["max_lateral_offset_m"])


def route_points(graph, nodes):
    points = rounded_xy(graph.xy(nodes), radius=5.0)
    return np.column_stack((points, np.zeros(len(points))))


def config(seed, max_sim_seconds):
    return {
        "task": "urban_pursuit", "seed": seed,
        "scene": "scene_bellevue_pursuit.jsonc",
        "mesh_path": str(BELLEVUE_MESH),
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "sensor_max_age_sec": 1.5, "sensor_pair_tolerance_sec": .06,
        "tracking": {"minimum_pixels": 25, "segmentation_color_order": "bgr"},
        "control_hz": 3.0, "target_speed_mps": 1.1,
        "interceptor_speed_mps": 1.3, "drone_speed_mps": 2.8,
        "max_sim_seconds": max_sim_seconds,
        "capture": {"blocker_stop_range_m": 15.0,
                    "junction_activation_m": 35.0,
                    "gate_observation_tolerance_m": 8.0,
                    "max_lateral_offset_m": 4.0,
                    "stable_observation_tolerance_m": 2.5,
                    "required_distinct_observations": 3,
                    "gate_tolerance_m": 3.0,
                    "target_to_blocker_max_m": 16.0,
                    "target_stop_speed_mps": .3,
                    "dwell_sec": 2.0,
                    "required_aerial_observers": 2},
        "route_policy": "Seeded target exit is private to target controller; ground gates do not receive it",
        "truth_usage": "Robot ego poses for feedback and evaluation; target pose excluded from aerial and interceptor goal calculation",
        "capture_definition": "Two exit gates occupied, target stopped by front-camera obstacle observation, and both UAVs currently see target for two seconds",
    }


async def run(seed, max_sim_seconds):
    cfg = config(seed, max_sim_seconds)
    graph = RoadGraph(BELLEVUE_OSM, BELLEVUE_ROADS)
    start_node = "4625654530"
    junction_node = "4625654529"
    exit_choice = random.Random(seed).choice(("north", "south"))
    exit_node = "10964263780" if exit_choice == "north" else "7861659472"
    approach, _ = graph.shortest(start_node, junction_node)
    departure, _ = graph.shortest(junction_node, exit_node)
    target_route = approach + departure[1:]
    target_path = route_points(graph, target_route)
    junction = graph.points[junction_node]
    output = new_run("urban_pursuit", f"seed{seed}")
    cfg["scenario"] = {
        "start_osm_node": start_node, "junction_osm_node": junction_node,
        "target_exit_osm_node_private_to_policy": exit_node,
        "north_gate_ned": NORTH_GATE.tolist(),
        "south_gate_ned": SOUTH_GATE.tolist(),
    }
    write_json(output / "config.json", cfg)
    sources = [__file__, HERE / "road_graph.py", HERE / "urban_runtime.py",
               HERE / "prepare_pursuit_configs.py",
               ROOT / "ag/backend.py", ROOT / "ag/planning.py",
               ROOT / "ag/study_planning.py", ROOT / "ag/math3d.py",
               ROOT / "ag/contact.py", ROOT / "ag/storage.py",
               HERE / "scene_bellevue_pursuit.jsonc",
               HERE / "robots/pursuit_uav.jsonc",
               HERE / "robots/pursuit_target.jsonc",
               HERE / "robots/pursuit_interceptor.jsonc"]
    source_dir = output / "source_snapshot"
    source_dir.mkdir()
    for source in sources:
        source = Path(source)
        (source_dir / source.name).write_bytes(source.read_bytes())
    write_json(output / "provenance.json", {
        "source_sha256": {Path(path).name: sha256(path) for path in sources},
        "osm_sha256": sha256(BELLEVUE_OSM),
        "road_report_sha256": sha256(BELLEVUE_ROADS),
        "mesh_sha256": sha256(BELLEVUE_MESH),
        "osm_license": "OpenStreetMap contributors, ODbL 1.0",
    })
    backend = UrbanBackend(cfg)
    tracker = Tracker(max_age=1.5, prediction_horizon=3.0)
    last_seen = {"Drone1": -1.0, "Drone2": -1.0}
    seen_count = {"Drone1": 0, "Drone2": 0}
    visibility_samples = 0
    both_visible_samples = 0
    target_started = False
    gate_commanded = False
    target_blocked = False
    dwell_start = None
    status, error = "failed", None
    collision_seen = {}
    impacts = []
    captures = []
    track_errors = []
    previous = {}
    distances = {name: 0.0 for name in
                 ("Target", "CarNorth", "CarSouth", "Drone1", "Drone2")}
    first_visual_time = None
    first_blocker_time = None
    blocker_confirmations = 0
    blocker_last_time = -1.0
    blocker_last_position = None
    start_wall = time.monotonic()
    row_fields = ["sim_time_sec", "elapsed_sec", "name", "x", "y", "z", "yaw_rad",
                  "speed_mps", "goal_x", "goal_y", "goal_z", "engine", "steering",
                  "brake", "vx", "vy", "vz", "phase"]
    stream = (output / "trajectories_controls.csv").open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(stream, fieldnames=row_fields)
    writer.writeheader()
    track_stream = (output / "tracking.csv").open("w", newline="", encoding="utf-8")
    track_fields = ["sim_time_sec", "truth_x", "truth_y", "estimate_x", "estimate_y",
                    "error_m_evaluation_only", "uav1_visible", "uav2_visible",
                    "target_front_north_visible", "target_front_south_visible"]
    track_writer = csv.DictWriter(track_stream, fieldnames=track_fields)
    track_writer.writeheader()
    events = (output / "events.jsonl").open("w", encoding="utf-8")
    def event(kind, sim_time, **details):
        events.write(json.dumps({"kind": kind, "sim_time_sec": sim_time, **details}) + "\n")
        events.flush()

    try:
        backend.connect()
        scene = backend.load()
        write_json(output / "loaded_scene.json", scene)
        if set(backend.kinds) != set(distances):
            raise RuntimeError("The five declared robots were not loaded")
        for name, sid in (("Target", TARGET_ID), ("CarNorth", NORTH_ID),
                          ("CarSouth", SOUTH_ID)):
            if not backend.world.set_segmentation_id_by_name(name, sid, False, True):
                raise RuntimeError(f"Could not label {name} for independent observation")
        write_json(output / "segmentation_ids.json",
                   backend.world.get_segmentation_id_map())
        await backend.ready(45)
        await backend.settle()
        write_json(output / "startup_pose_sync.json", await backend.wait_for_pose_sync())
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_start.png"))
        await backend.start()
        write_json(output / "post_takeoff_pose_sync.json", await backend.wait_for_pose_sync())
        start_sim = backend.now()
        last_sim, last_advance = start_sim, time.monotonic()
        while backend.now() - start_sim < cfg["max_sim_seconds"]:
            tick = time.monotonic()
            now = backend.now()
            if now > last_sim:
                last_sim, last_advance = now, tick
            if tick - last_advance > 12 or tick - start_wall > 1200:
                raise TimeoutError("Simulator or wall-time watchdog expired")
            poses = {name: backend.pose(name) for name in distances}
            for name, pose in poses.items():
                if name in previous:
                    distances[name] += float(np.linalg.norm(pose["position"][:2] - previous[name][:2]))
                previous[name] = pose["position"].copy()
            new_impacts = collision_snapshot(backend, distances, collision_seen)
            if new_impacts:
                impacts.extend(new_impacts)
                status = "obstacle_collision"
                event("obstacle_collision", now,
                      impacts=[{"robot": x["robot"], "actor": x["event"].get("object_name")}
                               for x in new_impacts])
                break
            observations = {name: backend.observation(name, TARGET_ID)
                            for name in ("Drone1", "Drone2")}
            fresh = []
            for name, observation in observations.items():
                if observation is not None and observation["time"] > last_seen[name]:
                    fresh.append(observation)
                    last_seen[name] = observation["time"]
                    seen_count[name] += 1
                    if seen_count[name] == 1:
                        save_observation_pair(backend, name,
                                              output / f"{name}_first_target_raw.npz")
                        captures.append(save_camera(backend, name + "/scene_camera",
                                                    output / f"{name}_first_target.png"))
            estimate, measured = tracker.update(now, fresh)
            visible = {name: observations[name] is not None for name in observations}
            visibility_samples += 1
            both_visible_samples += int(all(visible.values()))
            if estimate is not None and first_visual_time is None:
                first_visual_time = now
                event("aerial_target_lock", now, observer_count=sum(visible.values()))
            if not target_started and seen_count["Drone1"] >= 2:
                target_started = True
                gate_commanded = True
                event("target_and_interceptors_released", now,
                      trigger="First UAV has two independent target observations")
            if not target_started and now - start_sim > 45:
                status = "no_initial_aerial_target_lock"
                break
            target = poses["Target"]
            approaching_junction = (target_started and
                                    np.linalg.norm(target["position"][:2] - junction[:2]) <
                                    cfg["capture"]["junction_activation_m"])
            north_front = backend.observation("Target", NORTH_ID) if approaching_junction else None
            south_front = backend.observation("Target", SOUTH_ID) if approaching_junction else None
            blocker = north_front if exit_choice == "north" else south_front
            blocker_name = "CarNorth" if exit_choice == "north" else "CarSouth"
            gate = NORTH_GATE if exit_choice == "north" else SOUTH_GATE
            valid_blocker = blocker_candidate(blocker, target, junction, gate, cfg["capture"])
            if not target_blocked:
                if valid_blocker and blocker["time"] > blocker_last_time:
                    position = blocker["position"][:2]
                    if (blocker_last_position is None or
                            np.linalg.norm(position - blocker_last_position) <
                            cfg["capture"]["stable_observation_tolerance_m"]):
                        blocker_confirmations += 1
                    else:
                        blocker_confirmations = 1
                    blocker_last_time = blocker["time"]
                    blocker_last_position = position.copy()
                elif not valid_blocker:
                    blocker_confirmations = 0
                    blocker_last_position = None
                if blocker_confirmations >= cfg["capture"]["required_distinct_observations"]:
                    target_blocked = True
                    first_blocker_time = now
                    save_observation_pair(backend, "Target", output / "blocker_seen_raw.npz")
                    captures.append(save_camera(backend, "Target/scene_camera",
                                                output / "blocker_seen.png"))
                    event("target_brakes_for_observed_blocker", now,
                          blocker=blocker_name,
                          observed_position_ned=blocker["position"].tolist(),
                          distinct_confirmations=blocker_confirmations)
            track_error = (float(np.linalg.norm(estimate[:2] - target["position"][:2]))
                           if estimate is not None else None)
            if track_error is not None:
                track_errors.append(track_error)
            track_writer.writerow({
                "sim_time_sec": now, "truth_x": target["position"][0],
                "truth_y": target["position"][1],
                "estimate_x": estimate[0] if estimate is not None else "",
                "estimate_y": estimate[1] if estimate is not None else "",
                "error_m_evaluation_only": track_error if track_error is not None else "",
                "uav1_visible": visible["Drone1"], "uav2_visible": visible["Drone2"],
                "target_front_north_visible": north_front is not None,
                "target_front_south_visible": south_front is not None,
            })

            goals = {"Target": None, "CarNorth": None, "CarSouth": None,
                     "Drone1": None, "Drone2": None}
            if target_started and not target_blocked:
                goals["Target"] = path_lookahead(target_path, target["position"], 5.0)[0]
            if gate_commanded:
                goals["CarNorth"], goals["CarSouth"] = NORTH_GATE, SOUTH_GATE
            if estimate is not None:
                goals["Drone1"] = np.array([estimate[0], estimate[1], -27.0])
            else:
                goals["Drone1"] = np.array([-87.0, -62.5, -27.0])
            goals["Drone2"] = np.array([10.7, -62.3, -40.0])
            controls = []
            for name, pose in poses.items():
                goal = goals[name]
                if backend.kinds[name] == "uav":
                    velocity = .7 * (goal - pose["position"])
                    speed = np.linalg.norm(velocity)
                    if speed > cfg["drone_speed_mps"]:
                        velocity *= cfg["drone_speed_mps"] / speed
                    engine, steering, brake = "", "", ""
                    controls.append(backend.drone_velocity(name, velocity))
                else:
                    velocity = ["", "", ""]
                    speed = float(np.linalg.norm(pose["velocity"][:2]))
                    gate_reached = (name == "CarNorth" and np.linalg.norm(pose["position"][:2] - NORTH_GATE[:2]) < 2.0 or
                                    name == "CarSouth" and np.linalg.norm(pose["position"][:2] - SOUTH_GATE[:2]) < 2.0)
                    if goal is None or gate_reached:
                        engine, steering, brake = 0.0, 0.0, 1.0
                    else:
                        desired = (cfg["target_speed_mps"] if name == "Target"
                                   else cfg["interceptor_speed_mps"])
                        engine, steering, brake = rover_command(
                            pose["position"], pose["yaw"], goal, speed,
                            desired_speed=desired)
                    controls.append(backend.rover(name, engine, steering, brake))
                writer.writerow({
                    "sim_time_sec": now, "elapsed_sec": now - start_sim,
                    "name": name, "x": pose["position"][0], "y": pose["position"][1],
                    "z": pose["position"][2], "yaw_rad": pose["yaw"],
                    "speed_mps": np.linalg.norm(pose["velocity"]),
                    "goal_x": goal[0] if goal is not None else "",
                    "goal_y": goal[1] if goal is not None else "",
                    "goal_z": goal[2] if goal is not None else "",
                    "engine": engine, "steering": steering, "brake": brake,
                    "vx": velocity[0], "vy": velocity[1], "vz": velocity[2],
                    "phase": "contained" if target_blocked else
                    ("tracking" if target_started else "acquire"),
                })
            await asyncio.gather(*controls)
            at_gates = (np.linalg.norm(poses["CarNorth"]["position"][:2] - NORTH_GATE[:2]) < cfg["capture"]["gate_tolerance_m"] and
                        np.linalg.norm(poses["CarSouth"]["position"][:2] - SOUTH_GATE[:2]) < cfg["capture"]["gate_tolerance_m"])
            near_blocker = (np.linalg.norm(target["position"][:2] - poses[blocker_name]["position"][:2]) <
                            cfg["capture"]["target_to_blocker_max_m"])
            contained = (target_blocked and at_gates and near_blocker and
                         np.linalg.norm(target["velocity"][:2]) < cfg["capture"]["target_stop_speed_mps"] and
                         sum(visible.values()) >= cfg["capture"]["required_aerial_observers"])
            if contained:
                if dwell_start is None:
                    dwell_start = now
                if now - dwell_start >= cfg["capture"]["dwell_sec"]:
                    status = "completed"
                    event("contained", now, target_exit=exit_choice,
                          target_ned=target["position"].tolist(),
                          target_to_blocker_m=float(np.linalg.norm(
                              target["position"][:2] - poses[blocker_name]["position"][:2])))
                    captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                                output / "scene_capture.png"))
                    for name in ("Drone1", "Drone2"):
                        save_observation_pair(backend, name,
                                              output / f"{name}_capture_raw.npz")
                        captures.append(save_camera(backend, name + "/scene_camera",
                                                    output / f"{name}_capture.png"))
                    break
            else:
                dwell_start = None
            if int(now - start_sim) % 10 == 0:
                stream.flush()
                track_stream.flush()
            await asyncio.sleep(max(0.0, 1 / cfg["control_hz"] - (time.monotonic() - tick)))
        else:
            status = "timeout"
        if status != "completed":
            captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                        output / "scene_end.png"))
    except (KeyboardInterrupt, asyncio.CancelledError) as exc:
        status = "interrupted_diagnostic"
        error = type(exc).__name__
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(error, flush=True)
    finally:
        try:
            cleanup = await backend.cleanup()
        except Exception as exc:
            cleanup = [f"cleanup: {type(exc).__name__}: {exc}"]
        stream.close()
        track_stream.close()
        events.close()
        receipt(output, status, error=error, seed=seed, target_exit=exit_choice,
                target_route_osm_nodes=target_route, target_route_ned=target_path.tolist(),
                collision_events=impacts, distances_m=distances,
                aerial_seen_frames=seen_count, tracking_samples=visibility_samples,
                both_aerial_visible_fraction=(both_visible_samples / visibility_samples
                                              if visibility_samples else None),
                tracking_rmse_m=(float(np.sqrt(np.mean(np.square(track_errors))))
                                 if track_errors else None),
                first_visual_time_sec=first_visual_time,
                first_blocker_time_sec=first_blocker_time,
                target_blocked=target_blocked,
                captures=[item for item in captures if item],
                wall_duration_sec=time.monotonic() - start_wall,
                cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status, "error": error}), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-sim-seconds", type=float, default=220.0)
    args = parser.parse_args()
    status = asyncio.run(run(args.seed, args.max_sim_seconds))
    raise SystemExit(0 if status == "completed" else 1)


if __name__ == "__main__":
    main()
