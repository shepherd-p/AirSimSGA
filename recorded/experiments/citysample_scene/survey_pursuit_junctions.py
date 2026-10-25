"""Find plausible two-exit road forks away from the navigation district."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from inspect_zonegraph import SOURCE
from plan_routes import load_graph


def heading(lane):
    points = np.asarray(lane["points_cm"], dtype=float)[:, :2] / 100
    delta = points[-1] - points[0]
    return delta / max(float(np.linalg.norm(delta)), 1e-6)


def main():
    graph, lanes = load_graph(SOURCE)
    candidates = []
    for junction in graph:
        x, y = graph.nodes[junction]["end_xy"]
        if not (-1300 < x < -250 and -1300 < y < -650):
            continue
        successors = [index for index in graph.successors(junction)
                      if graph.nodes[index]["length_m"] >= 18]
        for i, first in enumerate(successors):
            for second in successors[i + 1:]:
                angle = math.degrees(math.acos(float(np.clip(
                    np.dot(heading(lanes[first]), heading(lanes[second])), -1, 1))))
                exits = np.asarray([graph.nodes[first]["end_xy"],
                                    graph.nodes[second]["end_xy"]])
                separation = float(np.linalg.norm(exits[0] - exits[1]))
                if not (35 < angle < 150 and 20 < separation < 130):
                    continue
                predecessors = [index for index in graph.predecessors(junction)
                                if graph.nodes[index]["length_m"] >= 20]
                if not predecessors:
                    continue
                candidates.append({
                    "junction": junction, "approach": max(
                        predecessors, key=lambda index: graph.nodes[index]["length_m"]),
                    "exits": [first, second], "junction_xy_m": [x, y],
                    "angle_deg": round(angle, 1),
                    "exit_separation_m": round(separation, 1),
                    "approach_length_m": round(graph.nodes[junction]["length_m"] +
                                               max(graph.nodes[index]["length_m"]
                                                   for index in predecessors), 1),
                    "exit_lengths_m": [round(graph.nodes[index]["length_m"], 1)
                                       for index in (first, second)],
                })
    candidates.sort(key=lambda row: (-row["approach_length_m"],
                                     -row["exit_separation_m"]))
    output = Path(__file__).resolve().parent / "generated/pursuit_junction_candidates.json"
    output.write_text(json.dumps(candidates[:100], indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"candidate_count": len(candidates),
                      "output": str(output), "top": candidates[:20]}, indent=2))


if __name__ == "__main__":
    main()
