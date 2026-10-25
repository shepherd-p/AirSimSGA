"""Drive through the previously colliding road-marking segment."""

import asyncio
import argparse
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ag.planning import rover_command
from ag.storage import write_json
from urban_scene.urban_runtime import UrbanBackend, collision_snapshot, new_run, sha256, MESH


async def main(args):
    output = new_run("urban_validation", args.name)
    config = {
        "scene": args.scene, "task": "urban_validation",
        "ip": "127.0.0.1", "port_topics": 8989, "port_services": 8990,
        "sdk_path": str(HERE.parent.parent / "ProjectAirSim/client/python/projectairsim/src"),
        "sensor_max_age_sec": 1.5,
        "mesh_path": str(args.mesh),
    }
    write_json(output / "config.json", config)
    backend = UrbanBackend(config)
    rows, impacts = [], []
    status, error = "failed", None
    last = {}
    start_wall = time.monotonic()
    try:
        backend.connect()
        backend.load()
        await backend.ready(30)
        await backend.settle()
        await backend.start()
        start = backend.now()
        goal = np.array([args.goal_x, args.goal_y, 0.0])
        while backend.now() - start < args.max_sim_seconds:
            pose = backend.pose("Car1")
            now = pose["time"]
            impacts += collision_snapshot(backend, ["Car1"], last)
            if impacts:
                status = "obstacle_collision"
                break
            if abs(pose["position"][2]) > 2.0:
                status = "off_road_height"
                break
            if np.linalg.norm(pose["position"][:2] - goal[:2]) < 1.0:
                status = "completed"
                break
            speed = float(np.linalg.norm(pose["velocity"][:2]))
            engine, steering, brake = rover_command(pose["position"], pose["yaw"], goal,
                                                      speed, desired_speed=1.0)
            rows.append({"time": now, "x": pose["position"][0], "y": pose["position"][1],
                         "z": pose["position"][2], "yaw": pose["yaw"],
                         "speed_mps": speed, "engine": engine, "steering": steering,
                         "brake": brake})
            await backend.rover("Car1", engine, steering, brake)
            await asyncio.sleep(.2)
        else:
            status = "timeout"
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        cleanup = await backend.cleanup()
        if rows:
            with (output / "trajectory.csv").open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        write_json(output / "summary.json", {
            "status": status, "error": error, "mesh_sha256": sha256(args.mesh),
            "impacts": impacts, "samples": len(rows), "last": rows[-1] if rows else None,
            "cleanup_errors": cleanup, "wall_sec": time.monotonic() - start_wall,
        })
    print(json.dumps({"output": str(output), "status": status, "error": error}), flush=True)
    return status


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="marking_probe")
    parser.add_argument("--scene", default="scene_marking_probe.jsonc")
    parser.add_argument("--mesh", type=Path, default=MESH)
    parser.add_argument("--goal-x", type=float, default=-12.0)
    parser.add_argument("--goal-y", type=float, default=89.3)
    parser.add_argument("--max-sim-seconds", type=float, default=45.0)
    raise SystemExit(0 if asyncio.run(main(parser.parse_args())) == "completed" else 1)
