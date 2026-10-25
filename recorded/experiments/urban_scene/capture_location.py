"""Capture an unmodified observer frame at a specified UrbanBase road location."""

import argparse
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "ProjectAirSim/client/python/projectairsim/src"))

from probe_urban_scene import camera_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="AGUrbanNavigationAir")
    parser.add_argument("--position", type=float, nargs=3, required=True)
    parser.add_argument("--target", type=float, nargs=3, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    from projectairsim import ProjectAirSimClient
    from projectairsim.types import Pose
    from projectairsim.utils import unpack_image
    import cv2

    client = ProjectAirSimClient(address="127.0.0.1", port_topics=8989, port_services=8990)
    client.connect()
    root = "/Sim/" + args.scene
    frames = []
    try:
        client.get_topic_info()
        topic = root + "/robots/Observer/sensors/Overview/scene_camera"
        if topic not in client.topics:
            raise RuntimeError(f"Observer topic unavailable: {topic}")
        client.subscribe(topic,
                         lambda _, message: frames.append(message))
        client.request({"method": root + "/Pause", "params": {"do_pause": False},
                        "version": 1.0})
        pose = camera_pose(args.position, args.target, Pose)
        accepted = client.request({
            "method": root + "/robots/Observer/sensors/Overview/SetPose",
            "params": {"pose": pose, "wait_for_pose_update": True}, "version": 1.0,
        })
        if not accepted:
            raise RuntimeError("Observer pose change was rejected")
        frames.clear()
        time.sleep(6)
        if not frames:
            raise RuntimeError("No fresh observer frame")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), unpack_image(frames[-1])):
            raise OSError(args.output)
        print(args.output.resolve())
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
