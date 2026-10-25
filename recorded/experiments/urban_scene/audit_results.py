"""Recheck recorded urban runs against source snapshots and sensor payloads."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
INDEX = ROOT / "delivery/urban/result_index.json"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def check_source_snapshots(path, report):
    provenance = json.loads((path / "provenance.json").read_text(encoding="utf-8"))
    hashes = provenance["source_sha256"]
    for name, expected in hashes.items():
        snapshot = path / "source_snapshot" / name
        if not snapshot.is_file() or digest(snapshot) != expected:
            report["failures"].append(f"Source snapshot hash mismatch: {snapshot}")
    report["source_snapshots_checked"] += len(hashes)


def check_captures(path, summary, report):
    for capture in summary.get("captures", []):
        png, npz = Path(capture["png"]), Path(capture["payload"])
        if not png.is_file() or not npz.is_file():
            report["failures"].append(f"Camera capture missing: {png} / {npz}")
            continue
        packet = np.load(npz)
        payload = packet["payload"].tobytes()
        if hashlib.sha256(payload).hexdigest() != capture["payload_sha256"]:
            report["failures"].append(f"Camera payload hash mismatch: {npz}")
        metadata = json.loads(str(packet["metadata_json"]))
        if int(metadata["time_stamp"]) != capture["time_ns"]:
            report["failures"].append(f"Camera timestamp mismatch: {npz}")
        report["camera_payloads_checked"] += 1


def audit_pursuit(path, report):
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    check_source_snapshots(path, report)
    check_captures(path, summary, report)
    track = rows(path / "trajectories_controls.csv")
    targets = [row for row in track if row["name"] == "Target"]
    events = [json.loads(line) for line in (path / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    kinds = {event["kind"] for event in events}
    if (summary["status"] != "completed" or summary["collision_events"] or
            summary["distances_m"]["Target"] < 80 or
            len(targets) < 100 or
            not {"aerial_target_lock", "target_brakes_for_observed_blocker", "contained"} <= kinds):
        report["failures"].append(f"Pursuit completion evidence incomplete: {path}")
    if not all((path / f"{name}_capture_raw.npz").is_file() for name in ("Drone1", "Drone2")):
        report["failures"].append(f"Capture sensor pair missing: {path}")
    if not (path / "blocker_seen_raw.npz").is_file():
        report["failures"].append(f"Blocker sensor pair missing: {path}")
    report["pursuit_runs"].append({
        "path": str(path), "status": summary["status"], "exit": summary["target_exit"],
        "target_distance_m": summary["distances_m"]["Target"],
        "tracking_rmse_m": summary["tracking_rmse_m"],
        "collision_count": len(summary["collision_events"]),
    })


def audit_navigation(path, report):
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    provenance = json.loads((path / "provenance.json").read_text(encoding="utf-8"))
    snapshot = path / "source_snapshot"
    mapping = {
        "control_sha256": "run_navigation.py",
        "graph_sha256": "road_graph.py",
        "backend_sha256": "urban_runtime.py",
        "scene_config_sha256": json.loads((path / "config.json").read_text(encoding="utf-8"))["scene"],
    }
    for field, name in mapping.items():
        source = snapshot / name
        if not source.is_file() or digest(source) != provenance[field]:
            report["failures"].append(f"Navigation source snapshot mismatch: {source}")
        report["source_snapshots_checked"] += 1
    check_captures(path, summary, report)
    if (summary["status"] != "completed" or summary["obstacle_collisions"] or
            summary["car_distance_m"] < 200 or
            not summary.get("detection_to_replan_sec") or
            len(rows(path / "samples.csv")) < 100):
        report["failures"].append(f"Navigation completion evidence incomplete: {path}")
    report["navigation_runs"].append({
        "path": str(path), "mode": json.loads((path / "config.json").read_text(encoding="utf-8"))["mode"],
        "car_distance_m": summary["car_distance_m"],
        "detection_to_replan_sec": summary["detection_to_replan_sec"],
        "collision_count": len(summary["obstacle_collisions"]),
    })


def audit_scale(path, report):
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    check_source_snapshots(path, report)
    check_captures(path, summary, report)
    fleet = summary["fleet_size_each_role"]
    rates = summary["stream_hz_per_robot"]
    if (summary["status"] != "completed" or summary["collision_events"] or
            summary["sim_duration_sec"] < 45 or len(rates) != 2*fleet or
            any(distance < 3 for distance in summary["distances_m"].values()) or
            any(rate["scene_camera"] < 3.0 for rate in rates.values()) or
            len(rows(path / "resources.csv")) < 30):
        report["failures"].append(f"Scale measurement incomplete: {path}")
    report["scale_runs"].append({
        "path": str(path), "fleet_size_each_role": fleet,
        "real_time_factor": summary["real_time_factor"],
        "sim_duration_sec": summary["sim_duration_sec"],
        "simulator_peak_rss_bytes": summary["peak_rss_bytes"]["simulator"],
        "client_peak_rss_bytes": summary["peak_rss_bytes"]["client"],
        "scene_camera_hz_min": min(rate["scene_camera"] for rate in rates.values()),
        "collision_count": len(summary["collision_events"]),
    })


def main():
    index = json.loads(INDEX.read_text(encoding="utf-8"))
    report = {"navigation_runs": [], "pursuit_runs": [], "scale_runs": [],
              "source_snapshots_checked": 0,
              "camera_payloads_checked": 0, "failures": []}
    for path in index["navigation_runs"].values():
        audit_navigation(Path(path), report)
    for path in index["pursuit_runs"].values():
        audit_pursuit(Path(path), report)
    for paths in index["scale_runs"].values():
        for path in paths:
            audit_scale(Path(path), report)
    report["status"] = "passed" if not report["failures"] else "failed"
    target = ROOT / "delivery/urban/audit_report.json"
    target.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["status"] == "passed" else 1)


if __name__ == "__main__":
    main()
