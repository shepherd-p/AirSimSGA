"""Save original ProjectAirSim camera packets at candidate City Sample viewpoints."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"))
from urban_scene.probe_urban_scene import camera_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--center", type=float, nargs=2, default=(0, 0))
    parser.add_argument("--street-lane", type=int,
                        help="Native ZoneGraph lane index used to place the street camera")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    from projectairsim import ProjectAirSimClient, World
    from projectairsim.types import Pose
    from projectairsim.utils import unpack_image

    output = args.output or ROOT / "outputs" / "citysample_scout" / datetime.now().strftime("%Y%m%d_%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    client = ProjectAirSimClient(address="127.0.0.1", port_topics=8989, port_services=8990)
    client.connect()
    frames = []
    try:
        client.socket_services.recv_timeout = 300000
        world = World(client, "scene_scout.jsonc", sim_config_path=str(HERE), delay_after_load_sec=2)
        topic = world.parent_topic + "/robots/Observer/sensors/Overview/scene_camera"
        client.subscribe(topic, lambda _, message: frames.append(message))
        world.resume()
        x, y = args.center
        if args.street_lane is not None:
            source = ROOT.parent / "CitySample/Saved/AGCitySampleZoneGraph.json"
            data = json.loads(source.read_text(encoding="utf-8"))
            lane = next((item for item in data["datasets"][0]["lanes"]
                         if item["index"] == args.street_lane), None)
            if lane is None:
                raise ValueError(f"ZoneGraph lane not found: {args.street_lane}")
            points = np.asarray(lane["points_cm"], dtype=float) / 100
            lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
            if lengths[-1] < 30:
                raise ValueError("Street lane must be at least 30 m long")
            camera_xy = np.array([np.interp(5, lengths, points[:, axis]) for axis in (0, 1)])
            target_xy = np.array([np.interp(35, lengths, points[:, axis]) for axis in (0, 1)])
            street_view = ((camera_xy[0], camera_xy[1], -3.5),
                           (target_xy[0], target_xy[1], -2.5))
        else:
            street_view = ((x - 80, y - 35, -16), (x + 70, y + 20, -2))
        views = (
            ("oblique", (x, y - 260, -170), (x, y, 0)),
            ("top", (x, y, -230), (x, y, 0)),
            ("street", *street_view),
        )
        records = []
        for label, position, target in views:
            pose = camera_pose(position, target, Pose)
            accepted = client.request({
                "method": world.parent_topic + "/robots/Observer/sensors/Overview/SetPose",
                "params": {"pose": pose, "wait_for_pose_update": True}, "version": 1.0,
            })
            if not accepted:
                raise RuntimeError(f"Camera pose rejected: {label}")
            time.sleep(2.0)
            frames.clear()
            deadline = time.monotonic() + 15
            while not frames and time.monotonic() < deadline:
                time.sleep(.2)
            if not frames:
                raise TimeoutError(f"No camera packet for {label}")
            frame = frames[-1]
            image = unpack_image(frame)
            png = output / f"{label}.png"
            if not cv2.imwrite(str(png), image):
                raise OSError(png)
            payload = bytes(frame["data"])
            np.savez_compressed(output / f"{label}.npz",
                                payload=np.frombuffer(payload, dtype=np.uint8),
                                metadata_json=json.dumps({k: v for k, v in frame.items() if k != "data"}, default=str))
            records.append({"view": label, "png": str(png),
                            "camera_position_ned": position, "target_ned": target,
                            "time_stamp_ns": int(frame["time_stamp"]),
                            "payload_sha256": hashlib.sha256(payload).hexdigest(),
                            "image_stddev": float(image.std()), "shape": list(image.shape)})
        elevations = []
        for dx in (-160, -80, 0, 80, 160):
            for dy in (-160, -80, 0, 80, 160):
                sample_x, sample_y = x + dx, y + dy
                elevations.append({"x": sample_x, "y": sample_y,
                                   "surface_z_up_m": world.get_surface_elevation_at_point(sample_x, sample_y)})
        (output / "capture_manifest.json").write_text(
            json.dumps({"scene": world.get_configuration(), "captures": records,
                        "surface_elevations": elevations}, indent=2) + "\n",
            encoding="utf-8")
        print(output)
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
