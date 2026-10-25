"""Reject ZoneGraph forks whose sampled lane surface is not drivable ground."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "ProjectAirSim/client/python/projectairsim/src"))

from urban_scene.probe_urban_scene import camera_pose


def lane_at(lane, distance):
    points = np.asarray(lane["points_cm"], dtype=float) / 100
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
    if distance < 0 or distance > lengths[-1]:
        return None
    return np.array([np.interp(distance, lengths, points[:, axis]) for axis in range(3)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=35)
    args = parser.parse_args()
    source = ROOT.parent / "CitySample/Saved/AGCitySampleZoneGraph.json"
    lanes = {lane["index"]: lane for dataset in json.loads(
        source.read_text(encoding="utf-8"))["datasets"] for lane in dataset["lanes"]}
    candidates = json.loads((HERE / "generated/pursuit_junction_candidates.json").read_text(
        encoding="utf-8"))[:args.limit]
    from projectairsim import ProjectAirSimClient, World
    from projectairsim.types import Pose

    client = ProjectAirSimClient(address="127.0.0.1", port_topics=8989,
                                 port_services=8990)
    client.connect()
    try:
        client.socket_services.recv_timeout = 300000
        world = World(client, "scene_scout.jsonc", sim_config_path=str(HERE),
                      delay_after_load_sec=2)
        frames = []
        camera_root = world.parent_topic + "/robots/Observer/sensors/Overview"
        client.subscribe(camera_root + "/scene_camera",
                         lambda _, frame: frames.append(frame))
        world.resume()
        report = []
        previous_junction = None
        for candidate in candidates:
            if candidate["junction"] != previous_junction:
                x, y = candidate["junction_xy_m"]
                frames.clear()
                accepted = client.request({
                    "method": camera_root + "/SetPose",
                    "params": {"pose": camera_pose((x, y, -160), (x, y, 0), Pose),
                               "wait_for_pose_update": True},
                    "version": 1.0})
                if not accepted:
                    raise RuntimeError("Could not move the World Partition observer")
                deadline = time.monotonic() + 15
                while not frames and time.monotonic() < deadline:
                    time.sleep(.2)
                if not frames:
                    raise TimeoutError("Observer did not receive a frame at the candidate")
                time.sleep(1)
                previous_junction = candidate["junction"]
            junction = lanes[candidate["junction"]]
            geometry = np.asarray(junction["points_cm"], dtype=float)[:, :2] / 100
            length = float(np.linalg.norm(np.diff(geometry, axis=0), axis=1).sum())
            checks = []
            for remaining in (80, 60, 40, 20, 0):
                point = lane_at(junction, length - remaining)
                if point is not None:
                    checks.append((f"approach_{remaining}m_before_fork", point))
            for label, lane_id in zip(("left", "right"), candidate["exits"]):
                for distance in (10, 20):
                    point = lane_at(lanes[lane_id], distance)
                    if point is not None:
                        checks.append((f"{label}_exit_{distance}m", point))
            samples = []
            for label, point in checks:
                surface = float(world.get_surface_elevation_at_point(
                    float(point[0]), float(point[1])))
                samples.append({"sample": label, "xy_m": point[:2].tolist(),
                                "lane_z_up_m": float(point[2]),
                                "surface_z_up_m": surface,
                                "difference_m": abs(surface - float(point[2]))})
            matched = sum(sample["difference_m"] < 1.2 for sample in samples)
            report.append({**candidate, "surface_checks": samples,
                           "ground_match_count": matched,
                           "ground_check_count": len(samples),
                           "all_on_ground": matched == len(samples) and len(samples) >= 9})
            print(json.dumps({"junction": candidate["junction"],
                              "exits": candidate["exits"],
                              "ground_match": f"{matched}/{len(samples)}"}), flush=True)
        output = HERE / "generated/pursuit_surface_validation.json"
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        accepted = [row for row in report if row["all_on_ground"]]
        print(json.dumps({"output": str(output), "accepted": accepted[:12]},
                         indent=2), flush=True)
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
