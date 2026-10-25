"""Verify recorded camera payloads and route evidence for one City Sample run."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    scenario = json.loads((run / "scenario.json").read_text(encoding="utf-8"))
    config = json.loads((run / "config.json").read_text(encoding="utf-8"))
    provenance = json.loads((run / "provenance.json").read_text(encoding="utf-8"))
    sys.path.insert(0, config["sdk_path"])
    from projectairsim.utils import unpack_image

    check(summary["status"] == "completed", "Run did not complete")
    check(summary["error"] is None, "Run has an error")
    check(not summary["collision_events"], "Non-ground collision was recorded")
    check(len(summary["survey"]) == len(summary["junction_crossings"]) == 3,
          "Three aerial surveys and UGV crossings are required")
    check(summary["committed_lanes"] == scenario["candidate_lane_ids"],
          "Committed road route differs from scenario")
    check(provenance["zonegraph_sha256"] == scenario["zonegraph_sha256"],
          "ZoneGraph provenance does not match scenario")
    for name, expected in provenance["source_sha256"].items():
        source = run / "source_snapshot" / name
        check(source.is_file() and digest(source) == expected,
              f"Source snapshot hash mismatch: {name}")

    capture_count = 0
    for capture in summary["captures"]:
        png, packet = Path(capture["png"]), Path(capture["payload"])
        check(png.parent == run and packet.parent == run,
              f"Camera file outside run folder: {png}")
        with np.load(packet) as data:
            payload = data["payload"].tobytes()
            metadata = json.loads(data["metadata_json"].item())
        check(hashlib.sha256(payload).hexdigest() == capture["payload_sha256"],
              f"Camera payload hash mismatch: {packet}")
        check(int(metadata["time_stamp"]) == capture["time_ns"],
              f"Camera timestamp mismatch: {packet}")
        decoded = unpack_image({**metadata, "data": payload})
        image = cv2.imread(str(png), cv2.IMREAD_UNCHANGED)
        check(image is not None and image.shape == tuple(capture["shape"])
              and np.array_equal(image, decoded),
              f"Camera PNG does not match raw payload: {png}")
        capture_count += 1

    for index, (survey, crossing) in enumerate(zip(
            summary["survey"], summary["junction_crossings"]), 1):
        check(survey["junction"]["lane_id"] ==
              crossing["junction"]["lane_id"] ==
              scenario["survey_junctions"][index - 1]["lane_id"],
              f"Junction {index} identity mismatch")
        check(survey["sim_time_sec"] < crossing["sim_time_sec"],
              f"UGV crossed junction {index} before UAV survey")
        check(survey["road_surface_assessment"]["visibility"] ==
              "road_surface_visible", f"Junction {index} road not visible")
        raw = Path(survey["depth_segmentation"]["file"])
        check(raw.parent == run, f"Depth packet outside run folder: {raw}")
        with np.load(raw) as data:
            metadata = json.loads(data["metadata_json"].item())
            check(data["depth_payload"].size > 0 and
                  data["segmentation_payload"].size > 0,
                  f"Missing raw UAV sensor payload: {raw}")
        check(int(metadata["depth"]["time_stamp"]) ==
              survey["depth_segmentation"]["depth_time_ns"] and
              int(metadata["segmentation"]["time_stamp"]) ==
              survey["depth_segmentation"]["segmentation_time_ns"],
              f"UAV sensor timestamps disagree: {raw}")
        distance = np.linalg.norm(
            np.asarray(crossing["car_pose_ned"][:2]) -
            np.asarray(crossing["junction"]["center_ned"][:2]))
        check(distance < 12, f"UGV did not enter junction {index}")

    with (run / "trajectories_controls.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    car = [row for row in rows if row["robot"] == "Car1"]
    drone = [row for row in rows if row["robot"] == "Drone1"]
    check(len(car) > 100 and len(drone) > 100, "Trajectories are incomplete")
    position = np.array([[float(row[axis]) for axis in "xy"] for row in car])
    timestamps = np.array([float(row["sim_time_sec"]) for row in car])
    steps = np.linalg.norm(np.diff(position, axis=0), axis=1)
    elapsed = np.diff(timestamps)
    check(np.all(elapsed > 0), "UGV timestamps are not increasing")
    max_step_speed = float(np.max(steps / elapsed))
    check(max_step_speed < 7, "UGV trajectory contains an unmodeled jump")
    check(abs(float(steps.sum()) - summary["car_distance_m"]) < 3,
          "UGV path length differs from summary")

    figures = run / "derived/annotation_manifest.json"
    figure_count = 0
    if figures.is_file():
        annotation = json.loads(figures.read_text(encoding="utf-8"))
        check(annotation["summary_sha256"] == digest(run / "summary.json"),
              "Annotation summary hash differs")
        for figure in annotation["figures"].values():
            if "output" in figure:
                check(digest(figure["output"]) == figure["output_sha256"],
                      f"Derived figure hash mismatch: {figure['output']}")
                figure_count += 1

    report = {
        "status": "verified", "run": str(run),
        "summary_sha256": digest(run / "summary.json"),
        "camera_png_raw_pairs_verified": capture_count,
        "uav_depth_segmentation_pairs_verified": 3,
        "source_snapshots_verified": len(provenance["source_sha256"]),
        "derived_figures_verified": figure_count,
        "ugv_trajectory_samples": len(car),
        "uav_trajectory_samples": len(drone),
        "ugv_distance_from_csv_m": float(steps.sum()),
        "ugv_distance_from_summary_m": summary["car_distance_m"],
        "max_sampled_ugv_step_speed_mps": max_step_speed,
        "wheel_ground_contact_feedback": "not validated in this run",
    }
    (run / "audit_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
