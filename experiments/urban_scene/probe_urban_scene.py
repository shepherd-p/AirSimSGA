"""Load the GLB in UrbanBase and save an unmodified simulator camera frame."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
# Camera-pose helpers are importable without an SDK or live simulator.
def sdk_source():
    from ag.portable import sdk_path
    return sdk_path()


def camera_pose(position, target, pose_type):
    delta = [b - a for a, b in zip(position, target)]
    yaw = math.atan2(delta[1], delta[0])
    pitch = -math.atan2(delta[2], math.hypot(delta[0], delta[1]))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return pose_type({
        "translation": dict(zip("xyz", position)),
        "rotation": dict(zip("wxyz", [cy * cp, -sy * sp, cy * sp, sy * cp])),
    })


def main():
    sys.path.insert(0, str(sdk_source()))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--glb", type=Path, default=HERE / "generated/south_lake_union_runtime.glb")
    parser.add_argument("--output", type=Path, default=HERE / "generated/urban_probe.png")
    parser.add_argument("--reuse", action="store_true", help="Capture the urban mesh already in the live probe scene")
    args = parser.parse_args()
    for port in (8989, 8990):
        with socket.create_connection(("127.0.0.1", port), timeout=3):
            pass
    if not args.reuse and not args.glb.is_file():
        raise FileNotFoundError(args.glb)
    from projectairsim import ProjectAirSimClient, World
    from projectairsim.types import Pose
    from projectairsim.utils import unpack_image
    import cv2

    client = ProjectAirSimClient(address="127.0.0.1", port_topics=8989, port_services=8990)
    client.connect()
    try:
        client.socket_services.recv_timeout = 300000
        parent_topic = "/Sim/AGUrbanProbe"
        world = None
        if not args.reuse:
            world = World(client, "scene_probe.jsonc", sim_config_path=str(HERE), delay_after_load_sec=1)
            parent_topic = world.parent_topic
        topic = parent_topic + "/robots/Observer/sensors/Overview/scene_camera"
        frames = []
        client.subscribe(topic, lambda _, message: frames.append(message))
        if world is not None:
            world.resume()
            pose = Pose({
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            })
            payload = args.glb.read_bytes()
            name = world.spawn_object_from_file("AGUrbanMesh", "gltf", payload, True, pose, [1, 1, 1], False)
        else:
            name = "AGUrbanMesh"
            payload = None
        listed = client.request({"method": parent_topic + "/ListObjects", "params": {"name": "^" + name + "$"}, "version": 1.0})
        if name not in listed:
            raise RuntimeError("Urban mesh was not registered by the simulator")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        views = [
            ("oblique", (0, -180, -170), (0, 0, 0)),
            ("top", (0, 0, -260), (0, 0, 0)),
            ("street", (0, -60, -18), (0, 0, 0)),
        ]
        captures = []
        for label, position, target in views:
            pose = camera_pose(position, target, Pose)
            accepted = client.request({
                "method": parent_topic + "/robots/Observer/sensors/Overview/SetPose",
                "params": {"pose": pose, "wait_for_pose_update": True},
                "version": 1.0,
            })
            if not accepted:
                raise RuntimeError("Camera pose update rejected for " + label)
            frames.clear()
            time.sleep(3)
            if not frames:
                raise TimeoutError("No urban camera frame received for " + label)
            frame = frames[-1]
            image = unpack_image(frame)
            path = args.output.with_name(args.output.stem + "_" + label + args.output.suffix)
            if not cv2.imwrite(str(path), image):
                raise RuntimeError("Unable to save camera frame " + label)
            captures.append({
                "view": label,
                "camera_position_ned": position,
                "target_ned": target,
                "camera_time_ns": int(frame["time_stamp"]),
                "image": str(path.resolve()),
                "image_shape": list(image.shape),
                "pixel_stddev": float(image.std()),
            })
        receipt = {
            "source_glb": str(args.glb.resolve()),
            "source_glb_sha256": hashlib.sha256(payload or args.glb.read_bytes()).hexdigest(),
            "scene": world.get_configuration() if world else "AGUrbanProbe (reused)",
            "mesh_actor": name,
            "camera_topic": topic,
            "captures": captures,
        }
        args.output.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"mesh_actor": name, "captures": captures}))
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
