"""Two-UAV target tracking and two-exit containment on City Sample roads."""

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
from citysample_runtime import CitySampleBackend
from urban_scene.probe_urban_scene import camera_pose
from urban_scene.urban_runtime import (
    collision_snapshot, new_run, receipt, save_camera, save_observation_pair, sha256,
)


IDS = {"Target": 210, "GateLeft": 211, "GateRight": 212}
ROBOTS = ("Target", "GateLeft", "GateRight", "Drone1", "Drone2")


def blocker_candidate(observation, target, gate, settings):
    if observation is None:
        return False
    position = np.asarray(observation["position"][:2], dtype=float)
    ego = target["position"][:2]
    forward = np.array([math.cos(target["yaw"]), math.sin(target["yaw"])])
    delta = position - ego
    return (np.all(np.isfinite(position)) and
            np.linalg.norm(position - gate[:2]) < settings["gate_observation_tolerance_m"] and
            0 < np.dot(delta, forward) < settings["blocker_stop_range_m"] and
            abs(forward[0] * delta[1] - forward[1] * delta[0]) <
            settings["max_lateral_offset_m"])


async def run(seed: int, max_sim_seconds: float, evidence_views: bool = False):
    scenario_path = HERE / "generated/pursuit_scenario.json"
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    source = ROOT.parent / "CitySample/Saved/AGCitySampleZoneGraph.json"
    if sha256(source) != scenario["zonegraph_sha256"]:
        raise RuntimeError("City Sample ZoneGraph changed after pursuit planning")
    choice = random.Random(seed).choice(("Left", "Right"))
    cfg = {
        "task": "citysample_pursuit", "seed": seed,
        "scene": ("generated/scene_pursuit_views.jsonc" if evidence_views else
                  "generated/scene_pursuit.jsonc"), "ip": "127.0.0.1",
        "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "sensor_max_age_sec": 2.0, "sensor_pair_tolerance_sec": .06,
        "tracking": {"minimum_pixels": 25, "segmentation_color_order": "bgr"},
        "control_hz": 3.0,
        "target_speed_mps": .85 if evidence_views else 1.4,
        "gate_speed_mps": .8 if evidence_views else 1.1,
        "drone_speed_mps": 4.0,
        "max_sim_seconds": max_sim_seconds,
        "ambient_traffic": "disabled by -ControlledTraffic startup option",
        "capture": {"blocker_stop_range_m": 18.0,
                    "gate_observation_tolerance_m": 5.0,
                    "max_lateral_offset_m": 4.0,
                    "required_distinct_observations": 2,
                    "gate_tolerance_m": 2.5,
                    "target_to_blocker_max_m": 20.0,
                    "target_stop_speed_mps": .3,
                    "dwell_sec": 2.0,
                    "required_aerial_observers": 2},
        "route_policy": "Seeded exit is private to target controller; both gate vehicles receive both exits",
        "truth_usage": "Own pose for feedback; target truth only for offline evaluation and safety interlock",
        "evidence_views": evidence_views,
    }
    output = new_run("citysample_pursuit", f"seed{seed}")
    write_json(output / "config.json", cfg)
    write_json(output / "scenario.json", scenario)
    sources = [__file__, HERE / "citysample_runtime.py", HERE / "prepare_pursuit.py",
               HERE / cfg["scene"], scenario_path,
               ROOT / "ag/backend.py", ROOT / "ag/planning.py",
               ROOT / "ag/study_planning.py", ROOT / "ag/math3d.py"]
    if evidence_views:
        sources.append(HERE / "generated/pursuit_tracker_ugv.jsonc")
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for path in sources:
        path = Path(path)
        (snapshot / path.name).write_bytes(path.read_bytes())
    write_json(output / "provenance.json", {
        "source_sha256": {Path(path).name: sha256(path) for path in sources},
        "zonegraph_sha256": sha256(source),
        "map_asset": "/Game/Map/Big_City_LVL",
        "game_mode": "ProjectAirSimGameMode",
    })
    routes = {key: np.asarray(value, dtype=float)
              for key, value in scenario["routes_ned"].items()}
    gates = {key: np.asarray(value, dtype=float)
             for key, value in scenario["gate_ned"].items()}
    fork = np.asarray(scenario["fork_ned"], dtype=float)
    initial_target = np.asarray(scenario["target_start_ned"], dtype=float)
    backend = CitySampleBackend(cfg)
    tracker = Tracker(max_age=2.0, prediction_horizon=3.0)
    status, error = "failed", None
    impacts, collision_stamps, captures = [], {}, []
    last_seen = {"Drone1": -1.0, "Drone2": -1.0}
    seen_count = {"Drone1": 0, "Drone2": 0}
    last_ugv_seen_time, ugv_seen_count = -1.0, 0
    paired_views, last_pair_time = [], -math.inf
    last_blocker_time, blocker_confirmations = -1.0, 0
    target_started, target_blocked = False, False
    first_visual_time, first_blocker_time, dwell_start = None, None, None
    samples, both_visible_samples, tracking_errors = 0, 0, []
    distances = {name: 0.0 for name in ROBOTS}
    previous = {}
    start_wall = time.monotonic()
    stream = (output / "trajectories_controls.csv").open("w", newline="", encoding="utf-8")
    fields = ["sim_time_sec", "elapsed_sec", "robot", "x", "y", "z", "yaw_rad",
              "speed_mps", "goal_x", "goal_y", "goal_z", "engine", "steering",
              "brake", "vx", "vy", "vz", "phase"]
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    tracks = (output / "tracking.csv").open("w", newline="", encoding="utf-8")
    track_writer = csv.DictWriter(tracks, fieldnames=[
        "sim_time_sec", "truth_x", "truth_y", "estimate_x", "estimate_y",
        "error_m_evaluation_only", "uav1_visible", "uav2_visible",
        "target_front_blocker_visible"])
    track_writer.writeheader()
    events = (output / "events.jsonl").open("w", encoding="utf-8")

    def event(kind, now, **details):
        events.write(json.dumps({"kind": kind, "sim_time_sec": now, **details}) + "\n")
        events.flush()

    try:
        backend.connect()
        write_json(output / "loaded_scene.json", backend.load())
        if set(backend.kinds) != set(ROBOTS):
            raise RuntimeError("City Sample pursuit did not load all five robots")
        for name, sid in IDS.items():
            if not backend.world.set_segmentation_id_by_name(name, sid, False, True):
                raise RuntimeError(f"Could not assign segmentation ID to {name}")
        write_json(output / "segmentation_ids.json",
                   backend.world.get_segmentation_id_map())
        from projectairsim.types import Pose
        observer_pose = camera_pose(scenario["observer_position_ned"],
                                    scenario["observer_target_ned"], Pose)
        backend.client.request({
            "method": backend.world.parent_topic +
                      "/robots/Observer/sensors/Overview/SetPose",
            "params": {"pose": observer_pose, "wait_for_pose_update": True},
            "version": 1.0})
        await backend.ready(60)
        await backend.settle()
        write_json(output / "startup_pose_sync.json", await backend.wait_for_pose_sync())
        if np.linalg.norm(backend.pose("Target")["position"][:2] -
                          initial_target[:2]) > 4:
            raise RuntimeError("Target did not settle at the declared road start")
        captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                    output / "scene_start.png"))
        captures.append(save_camera(backend, "Target/scene_camera",
                                    output / "street_start.png"))
        await backend.start()
        write_json(output / "post_takeoff_pose_sync.json", await backend.wait_for_pose_sync())
        start_sim = backend.now()
        last_sim, last_progress = start_sim, time.monotonic()
        while backend.now() - start_sim < max_sim_seconds:
            now = backend.now()
            if now > last_sim:
                last_sim, last_progress = now, time.monotonic()
            if time.monotonic() - last_progress > 20 or time.monotonic() - start_wall > 1200:
                raise TimeoutError("City Sample simulation stopped progressing")
            poses = {name: backend.pose(name) for name in ROBOTS}
            for name, pose in poses.items():
                if name in previous:
                    distances[name] += float(np.linalg.norm(
                        pose["position"][:2] - previous[name][:2]))
                previous[name] = pose["position"].copy()
            new_impacts = collision_snapshot(backend, backend.kinds, collision_stamps)
            if new_impacts:
                impacts.extend(new_impacts)
                status = "collision"
                event("collision", now, impacts=[{
                    "robot": hit["robot"], "object": hit["event"].get("object_name")}
                    for hit in new_impacts])
                break
            observations = {name: backend.observation(name, IDS["Target"])
                            for name in ("Drone1", "Drone2")}
            fresh = []
            for name, observation in observations.items():
                if observation is not None and observation["time"] > last_seen[name]:
                    last_seen[name] = observation["time"]
                    seen_count[name] += 1
                    fresh.append(observation)
                    if seen_count[name] == 1:
                        save_observation_pair(backend, name,
                                              output / f"{name}_first_target_raw.npz")
                        captures.append(save_camera(backend, name + "/scene_camera",
                                                    output / f"{name}_first_target.png"))
            gate_right_settled = (np.linalg.norm(
                poses["GateRight"]["position"][:2] - gates["Right"][:2]) <
                cfg["capture"]["gate_tolerance_m"] and
                np.linalg.norm(poses["GateRight"]["velocity"][:2]) < .3)
            target_near_gate = (np.linalg.norm(
                poses["Target"]["position"][:2] -
                poses["GateRight"]["position"][:2]) < 30)
            if evidence_views and gate_right_settled and target_near_gate:
                ugv_observation = backend.observation("GateRight", IDS["Target"])
                if (ugv_observation is not None and
                        ugv_observation["time"] > last_ugv_seen_time):
                    last_ugv_seen_time = ugv_observation["time"]
                    ugv_seen_count += 1
                    uav_observation = observations["Drone1"]
                    if (uav_observation is not None and len(paired_views) < 3 and
                            abs(uav_observation["time"] -
                                ugv_observation["time"]) <= .6 and
                            now - last_pair_time >= 1.0):
                        index = len(paired_views) + 1
                        saved = {}
                        for observer in ("Drone1", "GateRight"):
                            prefix = output / f"view_{index:02d}_{observer}"
                            packet = save_observation_pair(
                                backend, observer,
                                prefix.with_name(prefix.name + "_raw.npz"))
                            frame = save_camera(
                                backend, observer + "/scene_camera",
                                prefix.with_suffix(".png"))
                            if packet is None or frame is None:
                                raise RuntimeError(f"Could not save {observer} target view")
                            captures.append(frame)
                            saved[observer] = {"observation": packet, "scene": frame}
                        paired_views.append({"index": index, "sim_time_sec": now,
                                             "uav_target_pixels": uav_observation["pixels"],
                                             "ugv_target_pixels": ugv_observation["pixels"],
                                             "observation_delta_ms": abs(
                                                 uav_observation["time"] -
                                                 ugv_observation["time"]) * 1000,
                                             "files": saved})
                        last_pair_time = now
                        event("paired_target_views_saved", now, index=index,
                              ugv_pixels=ugv_observation["pixels"])
            estimate, _ = tracker.update(now, fresh)
            visible = {name: observations[name] is not None for name in observations}
            samples += 1
            both_visible_samples += int(all(visible.values()))
            target = poses["Target"]
            error_m = (float(np.linalg.norm(estimate[:2] - target["position"][:2]))
                       if estimate is not None else None)
            if error_m is not None:
                tracking_errors.append(error_m)
            if first_visual_time is None and estimate is not None:
                first_visual_time = now
                event("aerial_target_lock", now, observer_count=sum(visible.values()))
            if not target_started and seen_count["Drone1"] >= 2:
                target_started = True
                event("target_and_gates_released", now,
                      trigger="two independent UAV target observations")
            if not target_started and now - start_sim > 55:
                status = "no_initial_aerial_target_lock"
                break
            blocker_name = "Gate" + choice
            blocker_observation = (backend.observation("Target", IDS[blocker_name])
                                   if target_started else None)
            valid_blocker = blocker_candidate(
                blocker_observation, target, gates[choice], cfg["capture"])
            if not target_blocked:
                if valid_blocker and blocker_observation["time"] > last_blocker_time:
                    last_blocker_time = blocker_observation["time"]
                    blocker_confirmations += 1
                elif not valid_blocker:
                    blocker_confirmations = 0
                if blocker_confirmations >= cfg["capture"]["required_distinct_observations"]:
                    target_blocked = True
                    first_blocker_time = now
                    save_observation_pair(backend, "Target", output / "blocker_seen_raw.npz")
                    captures.append(save_camera(backend, "Target/scene_camera",
                                                output / "blocker_seen.png"))
                    event("target_brakes_for_observed_blocker", now,
                          blocker=blocker_name,
                          observed_position_ned=blocker_observation["position"].tolist(),
                          pixels=blocker_observation["pixels"])
            if (target_started and not target_blocked and
                    np.linalg.norm(target["position"][:2] -
                                   poses[blocker_name]["position"][:2]) < 7):
                status = "safety_stop_without_visual_confirmation"
                event("safety_stop", now, reason="blocker too close without visual confirmation")
                break
            track_writer.writerow({
                "sim_time_sec": now,
                "truth_x": target["position"][0], "truth_y": target["position"][1],
                "estimate_x": estimate[0] if estimate is not None else "",
                "estimate_y": estimate[1] if estimate is not None else "",
                "error_m_evaluation_only": error_m if error_m is not None else "",
                "uav1_visible": visible["Drone1"],
                "uav2_visible": visible["Drone2"],
                "target_front_blocker_visible": valid_blocker})
            goals = {
                "Target": (path_lookahead(routes[choice], target["position"], 8)[0]
                           if target_started and not target_blocked else None),
                "GateLeft": gates["Left"] if target_started else None,
                "GateRight": gates["Right"] if target_started else None,
                "Drone1": np.array([*(estimate[:2] if estimate is not None
                                      else initial_target[:2]), -26.0]),
                "Drone2": np.array([fork[0], fork[1], -34.0]),
            }
            controls = []
            for name, pose in poses.items():
                goal = goals[name]
                if backend.kinds[name] == "uav":
                    velocity = .7 * (goal - pose["position"])
                    norm = np.linalg.norm(velocity)
                    if norm > cfg["drone_speed_mps"]:
                        velocity *= cfg["drone_speed_mps"] / norm
                    engine = steering = brake = ""
                    controls.append(backend.drone_velocity(name, velocity))
                else:
                    velocity = ("", "", "")
                    speed = float(np.linalg.norm(pose["velocity"][:2]))
                    gate_reached = (name.startswith("Gate") and
                                    np.linalg.norm(pose["position"][:2] - goal[:2]) <
                                    cfg["capture"]["gate_tolerance_m"] if goal is not None
                                    else False)
                    if goal is None or gate_reached:
                        engine, steering, brake = 0.0, 0.0, 1.0
                    else:
                        desired = (cfg["target_speed_mps"] if name == "Target"
                                   else cfg["gate_speed_mps"])
                        engine, steering, brake = rover_command(
                            pose["position"], pose["yaw"], goal, speed,
                            desired_speed=desired)
                    controls.append(backend.rover(name, engine, steering, brake))
                writer.writerow({
                    "sim_time_sec": now, "elapsed_sec": now - start_sim,
                    "robot": name, "x": pose["position"][0],
                    "y": pose["position"][1], "z": pose["position"][2],
                    "yaw_rad": pose["yaw"],
                    "speed_mps": np.linalg.norm(pose["velocity"][:2]),
                    "goal_x": goal[0] if goal is not None else "",
                    "goal_y": goal[1] if goal is not None else "",
                    "goal_z": goal[2] if goal is not None else "",
                    "engine": engine, "steering": steering, "brake": brake,
                    "vx": velocity[0], "vy": velocity[1], "vz": velocity[2],
                    "phase": "blocked" if target_blocked else
                    ("pursuit" if target_started else "acquire"),
                })
            await asyncio.gather(*controls)
            at_gates = all(np.linalg.norm(
                poses["Gate" + label]["position"][:2] - gates[label][:2]) <
                cfg["capture"]["gate_tolerance_m"] for label in ("Left", "Right"))
            close_to_blocker = (np.linalg.norm(
                target["position"][:2] - poses[blocker_name]["position"][:2]) <
                cfg["capture"]["target_to_blocker_max_m"])
            contained = (target_blocked and at_gates and close_to_blocker and
                         np.linalg.norm(target["velocity"][:2]) <
                         cfg["capture"]["target_stop_speed_mps"] and
                         sum(visible.values()) >=
                         cfg["capture"]["required_aerial_observers"])
            if contained:
                if dwell_start is None:
                    dwell_start = now
                if now - dwell_start >= cfg["capture"]["dwell_sec"]:
                    if evidence_views and len(paired_views) < 3:
                        status = "insufficient_paired_target_views"
                        event("evidence_incomplete", now,
                              paired_views=len(paired_views),
                              ugv_seen_frames=ugv_seen_count)
                        break
                    status = "completed"
                    event("contained", now, target_exit=choice,
                          target_ned=target["position"].tolist(),
                          both_gates_occupied=True, both_uavs_see_target=True)
                    captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                                output / "scene_capture.png"))
                    captures.append(save_camera(backend, "Target/scene_camera",
                                                output / "target_capture.png"))
                    for name in ("Drone1", "Drone2"):
                        save_observation_pair(backend, name,
                                              output / f"{name}_capture_raw.npz")
                        captures.append(save_camera(backend, name + "/scene_camera",
                                                    output / f"{name}_capture.png"))
                    break
            else:
                dwell_start = None
            stream.flush()
            tracks.flush()
            await asyncio.sleep(1 / cfg["control_hz"])
        if status == "failed":
            status = "timeout"
        if status != "completed":
            captures.append(save_camera(backend, "Observer/Overview/scene_camera",
                                        output / "scene_end.png"))
            captures.append(save_camera(backend, "Target/scene_camera",
                                        output / "target_end.png"))
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(error, flush=True)
    finally:
        try:
            cleanup = await backend.cleanup()
        except Exception as exc:
            cleanup = [f"cleanup: {type(exc).__name__}: {exc}"]
        stream.close()
        tracks.close()
        events.close()
        receipt(output, status, error=error, seed=seed, target_exit=choice,
                collision_events=impacts, distances_m=distances,
                aerial_seen_frames=seen_count, tracking_samples=samples,
                ugv_target_seen_frames=ugv_seen_count,
                paired_target_views=paired_views,
                both_aerial_visible_fraction=(both_visible_samples / samples
                                              if samples else None),
                tracking_rmse_m=(float(np.sqrt(np.mean(np.square(tracking_errors))))
                                 if tracking_errors else None),
                first_visual_time_sec=first_visual_time,
                first_blocker_time_sec=first_blocker_time,
                target_blocked=target_blocked,
                captures=[item for item in captures if item],
                wall_duration_sec=time.monotonic() - start_wall,
                cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status,
                      "target_exit": choice, "error": error}), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-sim-seconds", type=float, default=180)
    parser.add_argument("--evidence-views", action="store_true")
    args = parser.parse_args()
    raise SystemExit(0 if asyncio.run(run(args.seed, args.max_sim_seconds,
                                       args.evidence_views)) ==
                     "completed" else 1)


if __name__ == "__main__":
    main()
