"""Inspect City Sample's exported native vehicle lanes and draw a route-planning map."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import networkx as nx
import numpy as np


HERE = Path(__file__).resolve().parent
import sys
sys.path.insert(0, str(HERE.parent))
from ag.portable import zonegraph_path
SOURCE = zonegraph_path()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=HERE / "generated/zonegraph_inspection")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = json.loads(args.source.read_text(encoding="utf-8"))
    datasets = data["datasets"]
    if len(datasets) != 1:
        raise RuntimeError(f"Expected one ZoneGraphData actor, found {len(datasets)}")
    lanes = {lane["index"]: lane for lane in datasets[0]["lanes"]
             if (lane["tag_mask"] & 2) and not (lane["tag_mask"] & 32)
             and not (lane["tag_mask"] & (1 << 20)) and len(lane["points_cm"]) > 1}
    graph = nx.DiGraph()
    segments = []
    for index, lane in lanes.items():
        points = np.asarray(lane["points_cm"], dtype=float) / 100
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        graph.add_node(index, start=points[0].tolist(), end=points[-1].tolist(),
                       length_m=length, tags=lane["tag_mask"])
        segments.append(points[:, :2])
    for index, lane in lanes.items():
        for successor in lane["outgoing"]:
            if successor in lanes:
                graph.add_edge(index, successor, weight=graph.nodes[successor]["length_m"])
    weak = sorted((len(component) for component in nx.weakly_connected_components(graph)), reverse=True)
    strong = sorted((len(component) for component in nx.strongly_connected_components(graph)), reverse=True)
    starts = np.asarray([graph.nodes[index]["start"] for index in graph])
    report = {
        "map": data["map"], "zonegraph_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
        "coordinate_system": "Unreal centimeters in source; plotted as Unreal x/y meters",
        "selected_tags": "Vehicle bit 1; exclude Freeway bit 5 and Closed Lane bit 20",
        "vehicle_city_lane_count": len(lanes), "directed_connection_count": graph.number_of_edges(),
        "tag_masks": dict(Counter(str(lane["tag_mask"]) for lane in lanes.values())),
        "largest_weak_component_lane_counts": weak[:10],
        "largest_strong_component_lane_counts": strong[:10],
        "bounds_m": {axis: [float(starts[:, i].min()), float(starts[:, i].max())]
                     for i, axis in enumerate("xyz")},
    }
    (args.output / "inspection.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    fig, ax = plt.subplots(figsize=(15, 12), dpi=170)
    ax.add_collection(LineCollection(segments, linewidths=.25, colors="#5b6770", alpha=.7))
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-2200, 600)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Unreal x (m)")
    ax.set_ylabel("Unreal y (m)")
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(args.output / "city_vehicle_lanes.png")
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
