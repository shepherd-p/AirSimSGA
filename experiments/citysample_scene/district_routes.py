"""Directed routes on City Sample's exported vehicle ZoneGraph."""

from __future__ import annotations

import heapq
import json
from pathlib import Path

import numpy as np


def load_lanes(path: Path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {lane["index"]: lane for dataset in data["datasets"]
            for lane in dataset["lanes"]
            if lane["tag_mask"] & 2 and not lane["tag_mask"] & 32
            and not lane["tag_mask"] & (1 << 20) and len(lane["points_cm"]) > 1}


def lane_points(lane):
    points = np.asarray(lane["points_cm"], dtype=float) / 100
    points[:, 2] *= -1
    return points


def route(lanes, start, goal, blocked=()):
    forbidden = set(blocked)
    if start in forbidden or goal in forbidden:
        raise ValueError("Start or goal lane is closed")
    lengths = {key: float(np.linalg.norm(np.diff(lane_points(lane)[:, :2],
                                           axis=0), axis=1).sum())
               for key, lane in lanes.items()}
    costs, parent = {start: lengths[start]}, {}
    queue = [(costs[start], start)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == goal:
            result = [goal]
            while result[-1] != start:
                result.append(parent[result[-1]])
            result.reverse()
            return result, cost
        for successor in lanes[current]["outgoing"]:
            if successor in forbidden or successor not in lanes:
                continue
            alternate = cost + lengths[successor]
            if alternate < costs.get(successor, float("inf")):
                costs[successor], parent[successor] = alternate, current
                heapq.heappush(queue, (alternate, successor))
    raise ValueError("No directed road route reaches the goal")


def route_points(lanes, indices):
    points = [lane_points(lanes[index]) for index in indices]
    return np.concatenate([points[0], *(part[1:] for part in points[1:])])


def nearest_route_lane(lanes, indices, position):
    position = np.asarray(position)[:2]
    distances = []
    for index in indices:
        points = lane_points(lanes[index])[:, :2]
        starts, delta = points[:-1], np.diff(points, axis=0)
        fraction = np.clip(np.sum((position - starts) * delta, axis=1) /
                           np.maximum(np.sum(delta * delta, axis=1), 1e-9), 0, 1)
        nearest = starts + fraction[:, None] * delta
        distances.append(float(np.linalg.norm(nearest - position, axis=1).min()))
    nearest_index = int(np.argmin(distances))
    return indices[nearest_index], distances[nearest_index]


def cross_track(position, path):
    starts = path[:-1, :2]
    delta = np.diff(path[:, :2], axis=0)
    fractions = np.clip(np.sum((np.asarray(position)[:2] - starts) * delta, axis=1) /
                        np.maximum(np.sum(delta * delta, axis=1), 1e-9), 0, 1)
    nearest = starts + fractions[:, None] * delta
    return float(np.linalg.norm(nearest - np.asarray(position)[:2], axis=1).min())
