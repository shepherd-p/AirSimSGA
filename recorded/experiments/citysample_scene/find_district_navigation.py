"""Find City Sample routes with two distinct street-level rerouting decisions."""

from __future__ import annotations

import argparse
import json
import random

import networkx as nx
import numpy as np

from inspect_zonegraph import SOURCE
from plan_routes import load_graph


def route_length(graph, path):
    return graph.nodes[path[0]]["length_m"] + nx.path_weight(graph, path, "weight")


def line_points(lanes, path):
    return np.concatenate([np.asarray(lanes[index]["points_cm"], dtype=float)[:, :2] / 100
                           for index in path])


def separation(lanes, first, second):
    reference = line_points(lanes, first)[::5]
    alternative = line_points(lanes, second)[::5]
    return float(np.max(np.min(np.linalg.norm(
        alternative[:, None, :] - reference[None, :, :], axis=2), axis=1)))


def route_turns(lanes, path):
    points = line_points(lanes, path)
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    valid = segments[lengths > .5]
    if len(valid) < 2:
        return 0
    headings = np.unwrap(np.arctan2(valid[:, 1], valid[:, 0]))
    return int(np.count_nonzero(np.abs(np.diff(headings)) > np.radians(25)))


def candidates(graph, lanes, start, goal, baseline):
    distance = 0.0
    for first in baseline[1:-1]:
        distance += graph.nodes[first]["length_m"]
        if not 90 < distance < route_length(graph, baseline) - 130:
            continue
        after_first = nx.subgraph_view(graph, filter_node=lambda node: node != first)
        try:
            reroute = nx.shortest_path(after_first, start, goal, weight="weight")
        except nx.NetworkXNoPath:
            continue
        first_length = route_length(graph, reroute)
        if not 1.08 < first_length / route_length(graph, baseline) < 1.9:
            continue
        if separation(lanes, baseline, reroute) < 30:
            continue
        traveled = 0.0
        for second in reroute[1:-1]:
            traveled += graph.nodes[second]["length_m"]
            if (second in baseline or not distance + 80 < traveled < first_length - 80):
                continue
            after_both = nx.subgraph_view(
                graph, filter_node=lambda node: node not in (first, second))
            try:
                final = nx.shortest_path(after_both, start, goal, weight="weight")
            except nx.NetworkXNoPath:
                continue
            final_length = route_length(graph, final)
            if (not 1.03 < final_length / first_length < 1.7 or final_length > 900 or
                    separation(lanes, reroute, final) < 25):
                continue
            yield {
                "start_lane": start, "goal_lane": goal,
                "closure_lanes": [first, second],
                "lengths_m": [round(route_length(graph, route), 1)
                              for route in (baseline, reroute, final)],
                "turn_counts": [route_turns(lanes, route)
                                for route in (baseline, reroute, final)],
                "separations_m": [round(separation(lanes, baseline, reroute), 1),
                                  round(separation(lanes, reroute, final), 1)],
                "paths": [baseline, reroute, final],
                "start_xy": graph.nodes[start]["start_xy"],
                "goal_xy": graph.nodes[goal]["end_xy"],
                "closure_xy": [graph.nodes[index]["start_xy"]
                               for index in (first, second)],
            }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", help="Write reproducible candidates as JSON")
    parser.add_argument("--preview", help="Plot the selected candidate over native lanes")
    parser.add_argument("--candidate-index", type=int, default=3)
    parser.add_argument("--center-x", type=float)
    parser.add_argument("--center-y", type=float)
    parser.add_argument("--radius", type=float, default=180)
    args = parser.parse_args()
    graph, lanes = load_graph(SOURCE)
    rng = random.Random(21)
    starts = [index for index in graph if
              (np.linalg.norm(np.asarray(graph.nodes[index]["start_xy"]) -
                              [args.center_x, args.center_y]) < args.radius
               if args.center_x is not None and args.center_y is not None
               else -600 < graph.nodes[index]["start_xy"][0] < 600 and
                    -1100 < graph.nodes[index]["start_xy"][1] < -500) and
              graph.nodes[index]["length_m"] > 15 and
              graph.out_degree(index) > 0]
    rng.shuffle(starts)
    found = []
    for start in starts[:65]:
        distances, paths = nx.single_source_dijkstra(
            graph, start, cutoff=530, weight="weight")
        goals = [index for index, distance in distances.items()
                 if 310 < distance < 480 and
                 route_turns(lanes, paths[index]) >= 2]
        goals.sort(key=lambda index: abs(distances[index] - 400))
        for goal in goals[:12]:
            for result in candidates(graph, lanes, start, goal, paths[goal]):
                found.append(result)
                print(json.dumps({key: value for key, value in result.items()
                                  if key != "paths"}), flush=True)
                break
            if len(found) >= 15:
                break
        if len(found) >= 15:
            break
    if args.output:
        from pathlib import Path
        Path(args.output).write_text(json.dumps(found, indent=2) + "\n", encoding="utf-8")
    if args.preview and found:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        result = found[args.candidate_index]
        paths = [line_points(lanes, path) for path in result["paths"]]
        joined = np.concatenate(paths)
        lower = joined.min(axis=0) - 65
        upper = joined.max(axis=0) + 65
        fig, ax = plt.subplots(figsize=(10, 9), dpi=180)
        for lane in lanes.values():
            points = np.asarray(lane["points_cm"], dtype=float)[:, :2] / 100
            if (np.all(points.max(axis=0) >= lower) and
                    np.all(points.min(axis=0) <= upper)):
                ax.plot(points[:, 0], points[:, 1], color="#bec8c8", lw=.5)
        for points, color, label in zip(paths,
                                        ("#e2a337", "#16869b", "#d34d55"),
                                        ("Initial", "After closure 1", "After closure 2")):
            ax.plot(points[:, 0], points[:, 1], color=color, lw=2.5, label=label)
        for index, point in enumerate(result["closure_xy"], 1):
            ax.scatter(*point, marker="x", s=170, linewidths=3,
                       color="#161b1d", zorder=5)
            ax.annotate(f"Closure {index}", point, xytext=(8, 8),
                        textcoords="offset points", fontsize=9)
        ax.scatter(*result["start_xy"], marker="o", s=55, color="black", zorder=5)
        ax.scatter(*result["goal_xy"], marker="*", s=160, color="black", zorder=5)
        ax.set_xlim(*[lower[0], upper[0]])
        ax.set_ylim(*[lower[1], upper[1]])
        ax.set_aspect("equal")
        ax.legend()
        ax.set_xlabel("City Sample x / m")
        ax.set_ylabel("City Sample y / m")
        fig.savefig(args.preview, bbox_inches="tight")
        plt.close(fig)
    print(f"found={len(found)}", flush=True)


if __name__ == "__main__":
    main()
