"""Matched 1+1, 2+2 and 3+3 moving-fleet load measurements in Bellevue."""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np
import psutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from ag.planning import rover_command
from ag.storage import write_json
from urban_scene.urban_runtime import (
    UrbanBackend, collision_snapshot, new_run, receipt, save_camera, sha256,
)


ORIGINS = {
    "Car1": ("-87.03 -62.49 -2.0", "0 0 0"),
    "Car2": ("11.6 -18.0 -2.0", "0 0 -90"),
    "Car3": ("10.7 -124.9 -2.0", "0 0 90"),
    "Drone1": ("-87.0 -68.0 -2.0", "0 0 0"),
    "Drone2": ("10.7 -62.3 -2.0", "0 0 0"),
    "Drone3": ("10.7 -116.0 -2.0", "0 0 0"),
}
CAR_GOALS = {
    "Car1": np.array([-37.0, -62.4, 0.0]),
    "Car2": np.array([11.1, -43.0, 0.0]),
    "Car3": np.array([10.7, -82.0, 0.0]),
}
BELLEVUE_MESH = HERE / "generated/bellevue_downtown_runtime_512.glb"


def scene_for(size):
    actors = []
    for kind in ("Car", "Drone"):
        for number in range(1, size + 1):
            name = f"{kind}{number}"
            origin, rotation = ORIGINS[name]
            actors.append({
                "type": "robot", "name": name,
                "origin": {"xyz": origin, "rpy-deg": rotation},
                "robot-config": ("robots/pursuit_uav.jsonc" if kind == "Drone"
                                 else "robots/pursuit_target.jsonc"),
            })
    return {
        "id": f"AGBellevueScale{size}",
        "clock": {"type": "steppable", "step-ns": 5000000,
                  "real-time-update-rate": 5000000, "pause-on-start": True},
        "home-geo-point": {"latitude": 47.610, "longitude": -122.203,
                           "altitude": 0.0},
        "segmentation": {"initialize-ids": True, "ignore-existing": False,
                         "use-owner-name": True},
        "scene-type": "UnrealNative", "actors": actors,
    }


def simulator_process(pid):
    if pid is not None:
        process = psutil.Process(pid)
        if process.name().lower() != "unrealeditor.exe":
            raise ValueError(f"PID {pid} is not UnrealEditor.exe")
        return process
    matches = []
    for process in psutil.process_iter(["name", "cmdline"]):
        try:
            command = " ".join(process.info["cmdline"] or []).lower()
            if (process.info["name"] or "").lower() == "unrealeditor.exe" and "citypark1.uproject" in command:
                matches.append(process)
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    if len(matches) != 1:
        raise RuntimeError(f"Expected one CityPark1 UnrealEditor; found {len(matches)}. Pass --sim-pid.")
    return matches[0]


def cpu_seconds(process):
    usage = process.cpu_times()
    return usage.user + usage.system


async def run(size, duration, pid):
    if size not in (1, 2, 3) or duration <= 0:
        raise ValueError("Fleet size must be 1, 2 or 3 and duration must be positive")
    simulator = simulator_process(pid)
    client_process = psutil.Process()
    output = new_run("urban_scale", f"{size}uav_{size}ugv")
    scene_path = HERE / "generated" / f"bellevue_scale_{size}.jsonc"
    write_json(scene_path, scene_for(size))
    cfg = {
        "task": "urban_scale", "fleet_size_each_role": size,
        "scene": f"generated/bellevue_scale_{size}.jsonc",
        "mesh_path": str(BELLEVUE_MESH),
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "sensor_max_age_sec": 1.5, "sensor_pair_tolerance_sec": .06,
        "tracking": {"minimum_pixels": 25, "segmentation_color_order": "bgr"},
        "duration_sim_sec": duration, "control_hz": 3.0,
        "rover_speed_mps": 1.1, "drone_speed_mps": 2.8,
        "measurement_definition": "All robots move; all UAVs use the same three-stream 640x360 0.3 s camera, all UGVs use the same three-stream 640x360 0.3 s camera. No lidar or overview observer.",
        "fps_definition": "Received scene-camera messages per simulation second; this is sensor stream FPS, not Unreal render-thread FPS.",
        "simulator_pid": simulator.pid,
        "host": {"platform": platform.platform(), "logical_cpu_count": psutil.cpu_count()},
    }
    write_json(output / "config.json", cfg)
    sources = [__file__, HERE / "urban_runtime.py", HERE / "robots/pursuit_uav.jsonc",
               HERE / "robots/pursuit_target.jsonc", ROOT / "ag/backend.py",
               ROOT / "ag/planning.py", ROOT / "ag/contact.py",
               ROOT / "ag/math3d.py", ROOT / "ag/storage.py"]
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for source in [scene_path, *sources]:
        (snapshot / Path(source).name).write_bytes(Path(source).read_bytes())
    write_json(output / "provenance.json", {
        "source_sha256": {Path(path).name: sha256(path) for path in [scene_path, *sources]},
        "mesh_sha256": sha256(BELLEVUE_MESH),
        "mesh_source": "OpenStreetMap contributors, ODbL 1.0; OSM2World 0.4.0",
    })
    names = [f"Car{i}" for i in range(1, size + 1)] + [f"Drone{i}" for i in range(1, size + 1)]
    backend = UrbanBackend(cfg)
    distances = {name: 0.0 for name in names}
    previous = {}
    collision_seen = {}
    impacts = []
    captures = []
    status, error = "failed", None
    start_wall = None
    start_sim = None
    start_counts = {}
    initial_cpu = {}
    peak_rss = {"simulator": 0, "client": 0}
    trajectory_fields = ["sim_time_sec", "elapsed_sec", "name", "x", "y", "z",
                         "yaw_rad", "speed_mps", "goal_x", "goal_y", "goal_z",
                         "engine", "steering", "brake", "vx", "vy", "vz"]
    trajectory = (output / "trajectories_controls.csv").open("w", newline="", encoding="utf-8")
    trajectory_writer = csv.DictWriter(trajectory, fieldnames=trajectory_fields)
    trajectory_writer.writeheader()
    resource = (output / "resources.csv").open("w", newline="", encoding="utf-8")
    resource_fields = ["sim_time_sec", "elapsed_sec", "wall_elapsed_sec",
                       "simulator_cpu_seconds", "simulator_rss_bytes",
                       "client_cpu_seconds", "client_rss_bytes"]
    resource_writer = csv.DictWriter(resource, fieldnames=resource_fields)
    resource_writer.writeheader()
    try:
        backend.connect()
        loaded = backend.load()
        write_json(output / "loaded_scene.json", loaded)
        if set(backend.kinds) != set(names):
            raise RuntimeError("Live robot roster does not match the selected fleet")
        await backend.ready(60)
        await backend.settle()
        write_json(output / "startup_pose_sync.json", await backend.wait_for_pose_sync())
        await backend.start()
        write_json(output / "post_takeoff_pose_sync.json", await backend.wait_for_pose_sync())
        captures.append(save_camera(backend, "Drone1/scene_camera", output / "start_uav_view.png"))
        start_sim = backend.now()
        start_wall = time.monotonic()
        start_counts = dict(backend.buffer.counts)
        initial_cpu = {"simulator": cpu_seconds(simulator),
                       "client": cpu_seconds(client_process)}
        last_sim, last_advance = start_sim, start_wall
        while backend.now() - start_sim < duration:
            tick = time.monotonic()
            now = backend.now()
            if now > last_sim:
                last_sim, last_advance = now, tick
            if tick - last_advance > 12 or tick - start_wall > max(300, duration * 5):
                raise TimeoutError("Simulator or wall-time watchdog expired")
            poses = {name: backend.pose(name) for name in names}
            for name, pose in poses.items():
                if name in previous:
                    distances[name] += float(np.linalg.norm(pose["position"][:2] - previous[name][:2]))
                previous[name] = pose["position"].copy()
            new_impacts = collision_snapshot(backend, names, collision_seen)
            if new_impacts:
                impacts.extend(new_impacts)
                status = "obstacle_collision"
                break
            controls = []
            for name, pose in poses.items():
                if name.startswith("Car"):
                    goal = CAR_GOALS[name]
                    speed = float(np.linalg.norm(pose["velocity"][:2]))
                    if np.linalg.norm(goal[:2] - pose["position"][:2]) < 2.0:
                        engine, steering, brake = 0.0, 0.0, 1.0
                    else:
                        engine, steering, brake = rover_command(
                            pose["position"], pose["yaw"], goal, speed,
                            desired_speed=cfg["rover_speed_mps"])
                    velocity = ("", "", "")
                    controls.append(backend.rover(name, engine, steering, brake))
                else:
                    car = poses[f"Car{name[-1]}"]
                    goal = np.array([car["position"][0], car["position"][1], -27.0])
                    velocity = .7 * (goal - pose["position"])
                    speed = np.linalg.norm(velocity)
                    if speed > cfg["drone_speed_mps"]:
                        velocity *= cfg["drone_speed_mps"] / speed
                    engine = steering = brake = ""
                    controls.append(backend.drone_velocity(name, velocity))
                trajectory_writer.writerow({
                    "sim_time_sec": now, "elapsed_sec": now - start_sim,
                    "name": name, "x": pose["position"][0], "y": pose["position"][1],
                    "z": pose["position"][2], "yaw_rad": pose["yaw"],
                    "speed_mps": np.linalg.norm(pose["velocity"]),
                    "goal_x": goal[0], "goal_y": goal[1], "goal_z": goal[2],
                    "engine": engine, "steering": steering, "brake": brake,
                    "vx": velocity[0], "vy": velocity[1], "vz": velocity[2],
                })
            await asyncio.gather(*controls)
            sim_rss, client_rss = simulator.memory_info().rss, client_process.memory_info().rss
            peak_rss["simulator"] = max(peak_rss["simulator"], sim_rss)
            peak_rss["client"] = max(peak_rss["client"], client_rss)
            resource_writer.writerow({
                "sim_time_sec": now, "elapsed_sec": now - start_sim,
                "wall_elapsed_sec": time.monotonic() - start_wall,
                "simulator_cpu_seconds": cpu_seconds(simulator) - initial_cpu["simulator"],
                "simulator_rss_bytes": sim_rss,
                "client_cpu_seconds": cpu_seconds(client_process) - initial_cpu["client"],
                "client_rss_bytes": client_rss,
            })
            await asyncio.sleep(max(0.0, 1 / cfg["control_hz"] - (time.monotonic() - tick)))
        else:
            status = "completed"
        captures.append(save_camera(backend, "Drone1/scene_camera", output / "end_uav_view.png"))
        if status == "completed" and any(distance < 3 for distance in distances.values()):
            status = "insufficient_motion"
    except (KeyboardInterrupt, asyncio.CancelledError) as exc:
        status, error = "interrupted_diagnostic", type(exc).__name__
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(error, flush=True)
    finally:
        end_wall = time.monotonic()
        end_sim = backend.now() if backend.world is not None else start_sim
        counts = dict(backend.buffer.counts)
        try:
            cleanup = await backend.cleanup()
        except Exception as exc:
            cleanup = [f"cleanup: {type(exc).__name__}: {exc}"]
        trajectory.close()
        resource.close()
        sim_duration = max(0.0, end_sim - start_sim) if start_sim is not None and end_sim is not None else 0.0
        wall_duration = max(0.0, end_wall - start_wall) if start_wall is not None else 0.0
        rates = {}
        for name in names:
            rates[name] = {
                leaf: ((counts.get(f"{name}/{leaf}", 0) - start_counts.get(f"{name}/{leaf}", 0)) / sim_duration
                       if sim_duration else None)
                for leaf in ("scene_camera", "depth_camera", "segmentation_camera", "pose")
            }
        cpu = {}
        if initial_cpu and wall_duration:
            for label, process in (("simulator", simulator), ("client", client_process)):
                try:
                    cpu[label] = 100 * (cpu_seconds(process) - initial_cpu[label]) / wall_duration
                except psutil.Error:
                    cpu[label] = None
        receipt(output, status, error=error, fleet_size_each_role=size,
                sim_duration_sec=sim_duration, wall_duration_sec=wall_duration,
                real_time_factor=(sim_duration / wall_duration if wall_duration else None),
                process_cpu_percent_one_core_100=cpu, peak_rss_bytes=peak_rss,
                stream_hz_per_robot=rates, distances_m=distances,
                collision_events=impacts, captures=[capture for capture in captures if capture],
                cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status, "error": error}), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fleet-size", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--sim-pid", type=int)
    args = parser.parse_args()
    status = asyncio.run(run(args.fleet_size, args.duration, args.sim_pid))
    raise SystemExit(0 if status == "completed" else 1)


if __name__ == "__main__":
    main()
