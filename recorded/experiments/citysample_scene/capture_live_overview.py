"""Capture a live City Sample experiment through its independent Observer camera."""

from __future__ import annotations

import argparse
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
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--scene-id", default="AGCitySampleNavigationAir")
    parser.add_argument("--name", default="live_oblique")
    parser.add_argument("--position", type=float, nargs=3,
                        default=(-600, -500, -115))
    parser.add_argument("--target", type=float, nargs=3,
                        default=(-565, -295, -2))
    args = parser.parse_args()
    from projectairsim import ProjectAirSimClient
    from projectairsim.types import Pose
    from projectairsim.utils import unpack_image

    output = args.run_directory / "supplemental_overview"
    output.mkdir(exist_ok=True)
    client = ProjectAirSimClient(address="127.0.0.1", port_topics=8989,
                                 port_services=8990)
    frames = []
    client.connect()
    try:
        parent = f"/Sim/{args.scene_id}/robots/Observer/sensors/Overview"
        client.subscribe(parent + "/scene_camera",
                         lambda _, message: frames.append(message))
        pose = camera_pose(args.position, args.target, Pose)
        accepted = client.request({
            "method": parent + "/SetPose",
            "params": {"pose": pose, "wait_for_pose_update": True},
            "version": 1.0})
        if not accepted:
            raise RuntimeError("Observer camera pose request was rejected")
        frames.clear()
        deadline = time.monotonic() + 20
        while not frames and time.monotonic() < deadline:
            time.sleep(.2)
        if not frames:
            raise TimeoutError("No live Observer image received")
        frame = frames[-1]
        image = unpack_image(frame)
        png = output / f"{args.name}.png"
        if not cv2.imwrite(str(png), image):
            raise OSError(png)
        payload = bytes(frame["data"])
        npz = output / f"{args.name}.npz"
        np.savez_compressed(npz, payload=np.frombuffer(payload, dtype=np.uint8),
                            metadata_json=json.dumps({k: v for k, v in frame.items()
                                                      if k != "data"}, default=str))
        record = {"scene_id": args.scene_id, "png": str(png), "raw_packet": str(npz),
                  "time_stamp_ns": int(frame["time_stamp"]),
                  "camera_position_ned": args.position,
                  "camera_target_ned": args.target,
                  "payload_sha256": hashlib.sha256(payload).hexdigest(),
                  "image_shape": list(image.shape),
                  "image_stddev": float(image.std()),
                  "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        (output / f"{args.name}.json").write_text(
            json.dumps(record, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(record))
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
