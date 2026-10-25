"""Drive a short native City Sample lane and preserve pose, controls, and collisions."""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from ag.planning import rover_command
from ag.study_planning import path_lookahead
from ag.storage import write_json
from citysample_runtime import CitySampleBackend
from urban_scene.urban_runtime import collision_snapshot, new_run, receipt, save_camera, sha256


async def run(lane_index: int, max_sim_seconds: float):
    source = ROOT.parent / "CitySample/Saved/AGCitySampleZoneGraph.json"
    lane = next((lane for lane in json.loads(source.read_text(encoding="utf-8"))["datasets"][0]["lanes"]
                 if lane["index"] == lane_index), None)
    if lane is None:
        raise ValueError(f"Missing lane {lane_index}")
    path = np.asarray(lane["points_cm"], dtype=float) / 100
    path[:, 2] *= -1
    cfg = {"scene": "scene_drive_probe.jsonc", "ip": "127.0.0.1",
           "port_topics": 8989, "port_services": 8990,
           "sdk_path": str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"),
           "sensor_max_age_sec": 2.0, "control_hz": 3.0, "lane_index": lane_index,
           "max_sim_seconds": max_sim_seconds, "speed_mps": 1.5}
    output = new_run("citysample_drive_probe", f"lane{lane_index}")
    write_json(output / "config.json", cfg)
    write_json(output / "provenance.json", {"zonegraph_sha256": sha256(source),
              "scene_sha256": sha256(HERE / cfg["scene"]), "control_sha256": sha256(__file__)})
    backend = CitySampleBackend(cfg)
    rows = (output / "samples.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(rows)
    writer.writerow(("sim_time_sec", "x", "y", "z", "yaw_rad", "speed_mps",
                     "engine", "steering", "brake", "goal_x", "goal_y"))
    status, error, collisions = "failed", None, []
    last_stamps = {}
    first, last = None, None
    settle_samples = []
    distance_m = 0.0
    start_wall = time.monotonic()
    try:
        backend.connect()
        write_json(output / "loaded_scene.json", backend.load())
        await backend.ready(40)
        write_json(output / "surface_probe.json", {
            "lane_start_ned": path[0].tolist(),
            "surface_z_up_m": backend.world.get_surface_elevation_at_point(
                float(path[0, 0]), float(path[0, 1]))})
        for _ in range(15):
            pose = backend.pose("Car1")
            settle_samples.append({"sim_time_sec": pose["time"],
                                   "position_ned": pose["position"].tolist(),
                                   "velocity_mps": pose["velocity"].tolist()})
            await asyncio.sleep(.5)
        await backend.settle()
        write_json(output / "startup_pose_sync.json", await backend.wait_for_pose_sync())
        first = backend.pose("Car1")
        write_json(output / "first_pose.json", {"position_ned": first["position"].tolist(),
                   "yaw_rad": first["yaw"], "lane_start_ned": path[0].tolist()})
        if np.linalg.norm(first["position"][:2] - path[0, :2]) > 4:
            raise RuntimeError("Car did not settle at the native lane start")
        await backend.start()
        start_sim = backend.now()
        last_progress = time.monotonic()
        last_sim = start_sim
        while backend.now() - start_sim < max_sim_seconds:
            now = backend.now()
            if now > last_sim:
                last_progress, last_sim = time.monotonic(), now
            if time.monotonic() - last_progress > 15 or time.monotonic() - start_wall > 300:
                raise TimeoutError("Simulator clock stalled")
            car = backend.pose("Car1")
            if last is not None:
                distance_m += float(np.linalg.norm(car["position"][:2] - last[:2]))
            last = car["position"].copy()
            collisions.extend(collision_snapshot(backend, ("Car1",), last_stamps))
            if collisions:
                status = "collision"
                break
            segment = path[-1, :2] - path[0, :2]
            fraction = np.clip(np.dot(car["position"][:2] - path[0, :2], segment) /
                               np.dot(segment, segment), 0, 1)
            cross_track_m = float(np.linalg.norm(
                car["position"][:2] - (path[0, :2] + fraction * segment)))
            if cross_track_m > 5:
                status = "off_route"
                break
            if distance_m >= 40:
                status = "completed"
                break
            target, _ = path_lookahead(path, car["position"], lookahead=8.0)
            speed = float(np.linalg.norm(car["velocity"][:2]))
            engine, steering, brake = rover_command(
                car["position"], car["yaw"], target, speed, desired_speed=cfg["speed_mps"])
            await backend.rover("Car1", engine, steering, brake)
            writer.writerow((now, *car["position"], car["yaw"], speed, engine, steering,
                             brake, target[0], target[1]))
            await asyncio.sleep(1 / cfg["control_hz"])
        if status == "failed":
            status = "timeout"
        save_camera(backend, "Car1/scene_camera", output / "car_scene_end.png")
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(error, flush=True)
    finally:
        if backend.world is not None:
            try:
                pose = backend.pose("Car1")
                settle_samples.append({"sim_time_sec": pose["time"],
                                       "position_ned": pose["position"].tolist(),
                                       "velocity_mps": pose["velocity"].tolist()})
                write_json(output / "settle_samples.json", settle_samples)
            except Exception:
                pass
        try:
            cleanup = await backend.cleanup()
        except Exception as exc:
            cleanup = [f"cleanup: {type(exc).__name__}: {exc}"]
        rows.close()
        receipt(output, status, error=error, distance_m=distance_m, collision_events=collisions,
                last_position_ned=last.tolist() if last is not None else None,
                wall_duration_sec=time.monotonic() - start_wall, cleanup_errors=cleanup)
    print(json.dumps({"output": str(output), "status": status, "distance_m": distance_m,
                      "error": error}), flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lane", type=int, default=88625)
    parser.add_argument("--max-sim-seconds", type=float, default=90)
    args = parser.parse_args()
    raise SystemExit(0 if asyncio.run(run(args.lane, args.max_sim_seconds)) == "completed" else 1)


if __name__ == "__main__":
    main()
