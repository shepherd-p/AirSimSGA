"""Find directed City Sample lane routes and test whether a closure has a detour."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import networkx as nx
import numpy as np

from inspect_zonegraph import SOURCE


def load_graph(source: Path):
    data = json.loads(source.read_text(encoding="utf-8"))
    lanes = {lane["index"]: lane for dataset in data["datasets"] for lane in dataset["lanes"]
             if lane["tag_mask"] & 2 and not lane["tag_mask"] & 32
             and not lane["tag_mask"] & (1 << 20) and len(lane["points_cm"]) > 1}
    graph = nx.DiGraph()
    for index, lane in lanes.items():
        points = np.asarray(lane["points_cm"], dtype=float) / 100
        graph.add_node(index, length_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                       start_xy=points[0, :2].tolist(), end_xy=points[-1, :2].tolist())
    for index, lane in lanes.items():
        for successor in lane["outgoing"]:
            if successor in lanes:
                graph.add_edge(index, successor, weight=graph.nodes[successor]["length_m"])
    return graph, lanes


def detail(graph, indices):
    return [{"index": index, "start_xy_m": graph.nodes[index]["start_xy"],
             "end_xy_m": graph.nodes[index]["end_xy"],
             "length_m": graph.nodes[index]["length_m"]} for index in indices]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--start", type=int, default=79597)
    parser.add_argument("--goal", type=int, default=71907)
    parser.add_argument("--search", action="store_true",
                        help="Search nearby goals for a moderate-length directed detour")
    args = parser.parse_args()
    graph, _ = load_graph(args.source)
    if args.search:
        distances, paths = nx.single_source_dijkstra(graph, args.start, weight="weight")
        start_xy = np.asarray(graph.nodes[args.start]["start_xy"])
        candidates = [index for index, distance in distances.items()
                      if 190 <= distance <= 430 and
                      np.linalg.norm(np.asarray(graph.nodes[index]["start_xy"]) - start_xy) < 400]
        candidates.sort(key=lambda index: (abs(distances[index] - 290), index))
        results = []
        for goal in candidates[:80]:
            path = paths[goal]
            for closed in path[2:-1]:
                view = nx.subgraph_view(graph, filter_node=lambda node: node != closed)
                try:
                    alternate = nx.shortest_path(view, args.start, goal, weight="weight")
                except nx.NetworkXNoPath:
                    continue
                baseline_length = graph.nodes[args.start]["length_m"] + distances[goal]
                detour_length = graph.nodes[args.start]["length_m"] + nx.path_weight(graph, alternate, "weight")
                if 1.2 <= detour_length / baseline_length <= 2.5 and detour_length < 750:
                    results.append({"goal": goal, "closed_lane": closed,
                                    "goal_xy_m": graph.nodes[goal]["start_xy"],
                                    "closed_xy_m": graph.nodes[closed]["start_xy"],
                                    "baseline_length_m": round(baseline_length, 1),
                                    "detour_length_m": round(detour_length, 1),
                                    "baseline_lanes": path, "detour_lanes": alternate})
                    break
            if len(results) >= 12:
                break
        print(json.dumps({"start_lane": args.start, "results": results}, indent=2))
        return
    baseline = nx.shortest_path(graph, args.start, args.goal, weight="weight")
    candidates = []
    for closed in baseline[1:-1]:
        view = nx.subgraph_view(graph, filter_node=lambda node: node != closed)
        try:
            alternate = nx.shortest_path(view, args.start, args.goal, weight="weight")
        except nx.NetworkXNoPath:
            candidates.append({"closed_lane": closed, "detour": None})
            continue
        candidates.append({"closed_lane": closed, "detour": detail(graph, alternate),
                           "detour_length_m": graph.nodes[args.start]["length_m"] +
                           nx.path_weight(graph, alternate, "weight")})
    print(json.dumps({"start_lane": args.start, "goal_lane": args.goal,
                      "baseline": detail(graph, baseline),
                      "baseline_length_m": graph.nodes[args.start]["length_m"] +
                      nx.path_weight(graph, baseline, "weight"),
                      "closure_candidates": candidates}, indent=2))


if __name__ == "__main__":
    main()
