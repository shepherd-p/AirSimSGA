"""Create publication-ready figures and a machine-readable index from live runs."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       "ProjectAirSim/client/python/projectairsim/src"))
from projectairsim.image_utils import segmentation_id_to_color


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "delivery/urban"
PURSUIT = ROOT / "outputs/urban_pursuit"
SCALE = ROOT / "outputs/urban_scale"
NAVIGATION = ROOT / "outputs/urban_navigation"
COLORS = {
    "Target": "#c43b36", "CarNorth": "#26836f", "CarSouth": "#2487ad",
    "Drone1": "#e9a53c", "Drone2": "#7057a0",
    "Car1": "#c43b36", "Car2": "#26836f", "Car3": "#2487ad",
    "Drone3": "#9a625a",
}


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def latest_completed(directory, suffix):
    candidates = sorted(directory.glob(f"*_{suffix}"))
    valid = [path for path in candidates if (path / "summary.json").exists()
             and json.loads((path / "summary.json").read_text(encoding="utf-8"))["status"] == "completed"]
    if not valid:
        raise FileNotFoundError(f"No completed run for {suffix}")
    return valid[-1]


def mark_segmentation(ax, packet_path, segmentation_id, label):
    packet = np.load(packet_path)
    metadata = json.loads(str(packet["metadata_json"]))["segmentation"]
    image = packet["segmentation_payload"].reshape(metadata["height"], metadata["width"], 3)
    color = segmentation_id_to_color(segmentation_id)
    mask = np.all(image == [color.b, color.g, color.r], axis=2)
    ys, xs = np.where(mask)
    if len(xs) < 25:
        raise RuntimeError(f"No reliable segmentation for {label}: {packet_path}")
    x0, y0, x1, y1 = xs.min(), ys.min(), xs.max(), ys.max()
    ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0,
                           fill=False, edgecolor="#e43724", linewidth=2.5))
    ax.text(x0, max(8, y0-6), label, color="white", fontsize=9,
            bbox={"facecolor": "#a92218", "edgecolor": "none", "pad": 2})
    return x0, y0, x1, y1


def navigation_figure(paths, destination):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.1), dpi=180)
    for mode, color in (("air", "#c43b36"), ("ground", "#2487ad")):
        path = paths[mode]
        samples = load_rows(path / "samples.csv")
        car = np.array([[float(row["x"]), float(row["y"]), float(row["elapsed_sec"])]
                        for row in samples if row["robot"] == "Car1"])
        axes[0].plot(car[:, 0], car[:, 1], color=color, lw=2,
                     label=f"{mode}: {len(car)} samples")
        axes[0].scatter(car[0, 0], car[0, 1], color=color, s=18)
        axes[0].scatter(car[-1, 0], car[-1, 1], color=color, marker="x", s=38)
        axes[1].plot(car[:, 2], car[:, 0], color=color, lw=2, label=mode)
    axes[0].scatter(-12.3, -42.0, marker="s", color="#333333", s=45,
                    label="Physical closure")
    axes[0].set(xlabel="East / m", ylabel="North / m", title="Vehicle paths around road closure")
    axes[0].set_aspect("equal")
    axes[0].legend(fontsize=8, loc="upper left")
    axes[1].set(xlabel="Simulation time / s", ylabel="East / m",
                title="Progress along the road")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=.18)
    fig.tight_layout()
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def navigation_evidence_figure(path, destination):
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.1), dpi=160,
                             gridspec_kw={"width_ratios": [1.2, 1.2, .7]})
    axes[0].imshow(plt.imread(path / "scene_start.png"))
    axes[0].set_title("Original Seattle road scene")
    observation = plt.imread(path / "closure_detection.png")
    axes[1].imshow(observation)
    axes[1].set_title("UAV image: physical closure detected")
    x0, y0, x1, y1 = mark_segmentation(
        axes[1], path / "closure_detection_raw.npz", 75, "Observed closure")
    margin = 45
    axes[2].imshow(observation)
    axes[2].set_xlim(max(0, x0-margin), min(observation.shape[1], x1+margin))
    axes[2].set_ylim(min(observation.shape[0], y1+margin), max(0, y0-margin))
    axes[2].add_patch(Rectangle((x0, y0), x1-x0, y1-y0,
                                fill=False, edgecolor="#e43724", linewidth=2.5))
    axes[2].set_title("Same frame, enlarged crop")
    for ax in axes:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def pursuit_figure(path, destination):
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    rows = load_rows(path / "trajectories_controls.csv")
    tracks = {name: np.array([[float(row[k]) for k in ("x", "y", "z")]
                              for row in rows if row["name"] == name])
              for name in COLORS if name in {row["name"] for row in rows}}
    fig = plt.figure(figsize=(12.2, 5.5), dpi=180)
    plan = fig.add_subplot(121)
    ax = fig.add_subplot(122, projection="3d")
    for name, points in tracks.items():
        plan.plot(points[:, 0], points[:, 1], color=COLORS[name],
                  lw=2.5 if name == "Target" else 1.9, label=name)
        plan.scatter(points[0, 0], points[0, 1], color=COLORS[name], s=18)
        plan.scatter(points[-1, 0], points[-1, 1], color=COLORS[name], marker="x", s=35)
        ax.plot(points[:, 0], points[:, 1], -points[:, 2],
                color=COLORS[name], lw=2.5 if name == "Target" else 1.9)
        ax.scatter(*[points[0, 0], points[0, 1], -points[0, 2]],
                   color=COLORS[name], s=20, marker="o")
        ax.scatter(*[points[-1, 0], points[-1, 1], -points[-1, 2]],
                   color=COLORS[name], s=50, marker="x")
    for name, gate in (("North gate", [11.1, -43.0]),
                       ("South gate", [10.7, -82.0])):
        plan.scatter(gate[0], gate[1], color="#353535", marker="s", s=28)
        ax.scatter(gate[0], gate[1], 0, color="#353535", marker="s", s=22)
    plan.set(xlabel="East / m", ylabel="North / m", title="Road-plan trajectories")
    plan.set_xlim(-95, 28)
    plan.set_ylim(-95, -12)
    plan.set_aspect("equal")
    plan.grid(alpha=.18)
    ax.set(xlabel="East / m", ylabel="North / m", zlabel="Altitude / m",
           title=f"Height profile: {summary['target_exit']} exit")
    ax.set_xlim(-90, 25)
    ax.set_ylim(-95, -12)
    ax.set_zlim(0, 48)
    ax.set_box_aspect((115, 83, 48), zoom=1.02)
    ax.view_init(elev=25, azim=-62)
    plan.legend(loc="lower left", fontsize=8, ncol=2, frameon=True)
    fig.subplots_adjust(left=.06, right=.97, bottom=.09, top=.91, wspace=.15)
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def evidence_figure(path, destination):
    files = [("Original Bellevue scene", "scene_capture.png"),
             ("Target front camera: interceptor", "blocker_seen.png"),
             ("UAV 1 camera", "Drone1_capture.png"),
             ("UAV 2 camera", "Drone2_capture.png")]
    fig, axes = plt.subplots(2, 2, figsize=(13, 7.6), dpi=160)
    for ax, (title, filename) in zip(axes.flat, files):
        ax.imshow(plt.imread(path / filename))
        ax.set_title(title, fontsize=11)
        ax.axis("off")
    summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
    detections = [
        (axes[0, 1], "blocker_seen_raw.npz", 211 if summary["target_exit"] == "north" else 212,
         "Observed interceptor"),
        (axes[1, 0], "Drone1_capture_raw.npz", 210, "Observed target"),
        (axes[1, 1], "Drone2_capture_raw.npz", 210, "Observed target"),
    ]
    for ax, filename, segmentation_id, label in detections:
        mark_segmentation(ax, path / filename, segmentation_id, label)
    fig.subplots_adjust(left=.01, right=.99, bottom=.02, top=.94, hspace=.12, wspace=.02)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def scale_figure(paths, destination):
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.1), dpi=180)
    for size in (1, 2, 3):
        values = []
        for path in paths[size]:
            summary = json.loads((path / "summary.json").read_text(encoding="utf-8"))
            values.append(summary["real_time_factor"])
        axes[0].scatter([size * 2] * len(values), values, s=60,
                        color=COLORS[f"Car{size}"], zorder=3)
        axes[0].plot([size * 2] * len(values), values, color="#777777", lw=1)
        rates = [list(json.loads((path / "summary.json").read_text(encoding="utf-8"))
                      ["stream_hz_per_robot"].values()) for path in paths[size]]
        for run_index, robot_rates in enumerate(rates):
            axes[1].scatter([size * 2] * len(robot_rates),
                            [rate["scene_camera"] for rate in robot_rates],
                            s=24, alpha=.7, color=COLORS[f"Car{size}"])
    axes[0].set(xlabel="Total moving platforms", ylabel="Simulation / wall time",
                title="Real-time factor: two runs per fleet")
    axes[1].set(xlabel="Total moving platforms", ylabel="Scene-camera Hz / sim second",
                title="Per-platform image delivery")
    for ax in axes:
        ax.set_xticks([2, 4, 6])
        ax.grid(axis="y", alpha=.2)
    axes[1].set_ylim(0, 4)
    fig.tight_layout()
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    navigation = {mode: latest_completed(NAVIGATION, mode) for mode in ("air", "ground")}
    pursuit = {seed: latest_completed(PURSUIT, f"seed{seed}") for seed in (0, 1)}
    scale = {size: sorted(path for path in SCALE.glob(f"*_{size}uav_{size}ugv")
                          if (path / "summary.json").exists() and
                          json.loads((path / "summary.json").read_text(encoding="utf-8"))["status"] == "completed")
             for size in (1, 2, 3)}
    if any(len(paths) < 2 for paths in scale.values()):
        raise RuntimeError("Each scale group needs two completed runs")
    navigation_figure(navigation, OUT / "navigation_route_comparison.png")
    navigation_evidence_figure(navigation["air"], OUT / "navigation_closure_evidence.png")
    for seed, path in pursuit.items():
        pursuit_figure(path, OUT / f"pursuit_seed{seed}_3d_trajectory.png")
        evidence_figure(path, OUT / f"pursuit_seed{seed}_camera_evidence.png")
    scale_figure(scale, OUT / "platform_scale_results.png")
    index = {
        "navigation_runs": {mode: str(path) for mode, path in navigation.items()},
        "pursuit_runs": {str(seed): str(path) for seed, path in pursuit.items()},
        "scale_runs": {str(size): [str(path) for path in paths] for size, paths in scale.items()},
        "figure_inputs_are_live_run_csv_and_original_camera_png": True,
        "figures": sorted(str(path) for path in OUT.glob("*.png")),
    }
    (OUT / "result_index.json").write_text(json.dumps(index, indent=2), encoding="utf-8")
    print(json.dumps(index, indent=2))


if __name__ == "__main__":
    main()
