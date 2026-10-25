"""Short real-control road and collision smoke test in the new urban level."""

import argparse
import asyncio
import csv
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SDK = Path("F:/EpicGamesEnv/ProjectAirSim/client/python/projectairsim/src")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SDK))

from ag.contact import is_ground_contact
from ag.math3d import pose_parts
from ag.planning import rover_command
from ag.study_planning import path_lookahead


async def command(rover, engine=0.0, steering=0.0, brake=1.0):
    task = await rover.set_rover_controls(float(engine), float(steering), float(brake))
    return await asyncio.wait_for(task, 5)


def distance_to_path(point, path):
    starts = path[:-1]
    segments = np.diff(path, axis=0)
    fraction = np.clip(np.sum((point - starts) * segments, axis=1) /
                       np.maximum(np.sum(segments * segments, axis=1), 1e-9), 0, 1)
    return float(np.min(np.linalg.norm(starts + fraction[:, None] * segments - point, axis=1)))


async def run(args):
    from projectairsim import ProjectAirSimClient, Rover, World
    from projectairsim.types import Pose
    from projectairsim.utils import unpack_image
    import cv2

    for port in (8989, 8990):
        with socket.create_connection(("127.0.0.1", port), timeout=3):
            pass
    report = json.loads((HERE / "generated/road_network_report.json").read_text(encoding="utf-8"))
    path = report["candidate_route"]["glb_x_z_m"]
    # The importer maps GLB X/Z to this level's local NED X/Y.
    goal_indices = (25, 27, 28) if args.corner else (25, 27)
    goals = np.asarray([path[index] for index in goal_indices], dtype=float)
    if args.corner:
        junction = np.asarray(path[27], dtype=float)
        radius = 5.0
        center = junction + [-radius, radius]
        turn = [center + radius * np.array([math.cos(theta), math.sin(theta)])
                for theta in np.linspace(-math.pi / 2, 0, 9)]
        reference = np.vstack([path[24], path[25], turn, path[28]])
    else:
        reference = np.asarray([path[24], *goals], dtype=float)
    payload = args.glb.read_bytes()
    args.output.mkdir(parents=True, exist_ok=True)
    client = ProjectAirSimClient(address="127.0.0.1", port_topics=8989, port_services=8990)
    client.connect()
    rows = []
    collision_events = []
    frames = []
    rover = None
    try:
        client.socket_services.recv_timeout = 120000
        world = World(client, "scene_drive_probe.jsonc", sim_config_path=str(HERE), delay_after_load_sec=1)
        root = world.parent_topic + "/robots/Car1"
        client.subscribe(root + "/collision_info", lambda _, message: collision_events.append(message))
        client.subscribe(root + "/sensors/FrontCamera/scene_camera", lambda _, message: frames.append(message))
        pose = Pose({"translation": {"x": 0, "y": 0, "z": 0},
                     "rotation": {"w": 1, "x": 0, "y": 0, "z": 0}})
        mesh = world.spawn_object_from_file("AGUrbanMesh", "gltf", payload, True, pose, [1, 1, 1], False)
        if mesh not in world.list_objects("^" + mesh + "$"):
            raise RuntimeError("Urban mesh missing from live scene")
        rover = Rover(client, world, "Car1")
        world.resume()
        rover.enable_api_control()
        rover.arm()
        await command(rover)
        await asyncio.sleep(2)
        initial, _ = pose_parts(world.get_object_pose("Car1"))
        if np.linalg.norm(initial[:2] - np.asarray(path[24])) > 5:
            raise RuntimeError("Rover spawned away from the selected road")
        start_sim = world.get_sim_time() / 1e9
        last_position, last_time = initial, start_sim
        goal_index = 0
        status = "timeout"
        deadline = time.monotonic() + args.max_wall_seconds
        while time.monotonic() < deadline:
            now = world.get_sim_time() / 1e9
            position, rotation = pose_parts(world.get_object_pose("Car1"))
            dt = now - last_time
            speed = float(np.linalg.norm(position[:2] - last_position[:2]) / dt) if dt > 0.01 else 0.0
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            if abs(position[2]) > 2.0:
                raise RuntimeError(f"Rover left road-height band: NED z={position[2]:.2f}")
            if any(not is_ground_contact(event) for event in collision_events):
                status = "obstacle_collision"
                break
            offset = distance_to_path(position[:2], reference)
            if offset > 4.0:
                status = "off_route"
                break
            while goal_index < len(goals) and np.linalg.norm(position[:2] - goals[goal_index]) < 1.5:
                goal_index += 1
            if np.linalg.norm(position[:2] - goals[-1]) < 1.5:
                status = "completed"
                break
            target = path_lookahead(reference, position, lookahead=6.0)[0] if args.corner else goals[goal_index]
            engine, steering, brake = rover_command(position, yaw, target, speed, desired_speed=1.0)
            await command(rover, engine, steering, brake)
            rows.append({"sim_time_sec": now, "x": position[0], "y": position[1], "z": position[2],
                         "yaw_rad": yaw, "speed_mps": speed, "goal_index": goal_index,
                         "engine": engine, "steering": steering, "brake": brake,
                         "cross_track_m": offset})
            last_position, last_time = position, now
            await asyncio.sleep(0.2)
        await command(rover)
        if frames:
            image = unpack_image(frames[-1])
            cv2.imwrite(str(args.output / "rover_front_raw.png"), image)
        if rows:
            with (args.output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        summary = {
            "status": status,
            "scene_id": "AGUrbanDriveProbe",
            "source_glb_sha256": hashlib.sha256(payload).hexdigest(),
            "road_report_sha256": hashlib.sha256((HERE / "generated/road_network_report.json").read_bytes()).hexdigest(),
            "mesh_actor": mesh,
            "initial_pose_ned": initial.tolist(),
            "last_pose_ned": last_position.tolist(),
            "goal_points_ned": goals.tolist(),
            "goal_indices": goal_indices,
            "reference_path_ned": reference.tolist(),
            "samples": len(rows),
            "maximum_cross_track_m": max((row["cross_track_m"] for row in rows), default=None),
            "collision_messages": len(collision_events),
            "obstacle_collisions": sum(not is_ground_contact(event) for event in collision_events),
            "sim_duration_sec": world.get_sim_time() / 1e9 - start_sim,
        }
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary))
        if status != "completed":
            raise RuntimeError("Urban drive probe did not complete: " + status)
    finally:
        if rover is not None:
            try:
                await command(rover)
            except Exception:
                pass
        client.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--glb", type=Path, default=HERE / "generated/south_lake_union_runtime_512.glb")
    parser.add_argument("--output", type=Path, default=HERE / "generated/drive_probe_straight")
    parser.add_argument("--max-wall-seconds", type=float, default=120)
    parser.add_argument("--corner", action="store_true", help="Include the unvalidated 90-degree turn")
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
