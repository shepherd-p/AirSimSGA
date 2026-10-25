"""OSM road centerlines registered to the independent UrbanBase GLB."""

from __future__ import annotations

from collections import defaultdict
import heapq
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


HERE = Path(__file__).resolve().parent
DRIVABLE = {
    "primary", "secondary", "tertiary", "unclassified", "residential",
    "service", "living_street",
}


class RoadGraph:
    def __init__(self, osm=HERE / "source/south_lake_union_clipped.osm",
                 report=HERE / "generated/road_network_report.json"):
        self.report = json.loads(Path(report).read_text(encoding="utf-8"))
        transform = np.asarray(self.report["metric_east_north_to_glb_x_z"], float)
        latitude0, longitude0 = self.report.get("registration_origin_lat_lon", [47.624, -122.337])
        root = ET.parse(osm).getroot()
        self.points = {}
        for element in root.findall("node"):
            lat, lon = float(element.get("lat")), float(element.get("lon"))
            east = (lon - longitude0) * 111320 * math.cos(math.radians(latitude0))
            north = (lat - latitude0) * 111320
            self.points[element.get("id")] = np.array([east, north, 1.0]) @ transform
        self.edges = defaultdict(dict)
        for way in root.findall("way"):
            tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
            if tags.get("highway") not in DRIVABLE or tags.get("access") == "private":
                continue
            refs = [ref.get("ref") for ref in way.findall("nd")]
            for a, b in zip(refs, refs[1:]):
                if a not in self.points or b not in self.points:
                    continue
                distance = float(np.linalg.norm(self.points[a] - self.points[b]))
                if distance < 0.01:
                    continue
                self.edges[a][b] = min(distance, self.edges[a].get(b, math.inf))
                self.edges[b][a] = min(distance, self.edges[b].get(a, math.inf))

    def nearest_node(self, xy, maximum_distance=12.0):
        xy = np.asarray(xy, float)[:2]
        node = min(self.edges, key=lambda key: np.linalg.norm(self.points[key] - xy))
        distance = float(np.linalg.norm(self.points[node] - xy))
        if distance > maximum_distance:
            raise ValueError(f"No road node within {maximum_distance} m of {xy}: {distance:.2f} m")
        return node

    def nearest_edge(self, xy, maximum_distance=8.0):
        xy = np.asarray(xy, float)[:2]
        best = (math.inf, None)
        for a, neighbors in self.edges.items():
            start = self.points[a]
            for b in neighbors:
                if a >= b:
                    continue
                vector = self.points[b] - start
                fraction = np.clip(np.dot(xy - start, vector) / np.dot(vector, vector), 0, 1)
                distance = float(np.linalg.norm(xy - (start + fraction * vector)))
                if distance < best[0]:
                    best = distance, (a, b)
        if best[0] > maximum_distance:
            raise ValueError(f"Detected obstacle is {best[0]:.2f} m from the nearest road")
        return best[1], best[0]

    def shortest(self, start, goal, blocked=()):
        blocked = {frozenset(edge) for edge in blocked}
        distance, parent = {start: 0.0}, {}
        queue = [(0.0, start)]
        while queue:
            cost, node = heapq.heappop(queue)
            if cost != distance[node]:
                continue
            if node == goal:
                break
            for neighbor, length in self.edges[node].items():
                if frozenset((node, neighbor)) in blocked:
                    continue
                candidate = cost + length
                if candidate < distance.get(neighbor, math.inf):
                    distance[neighbor] = candidate
                    parent[neighbor] = node
                    heapq.heappush(queue, (candidate, neighbor))
        if goal not in distance:
            raise ValueError("No road route remains after closure")
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        path.reverse()
        return path, distance[goal]

    def xy(self, path):
        return np.asarray([self.points[node] for node in path], float)


def rounded_xy(points, radius=5.0, spacing=0.75):
    """Round road-centerline corners without extending outside adjacent edges."""
    points = np.asarray(points, float)
    if len(points) < 3:
        return points.copy()
    keep = [points[0]]
    for i in range(1, len(points) - 1):
        previous, current, following = keep[-1], points[i], points[i + 1]
        a, b = current - previous, following - current
        if np.linalg.norm(a) < 1e-6 or np.linalg.norm(b) < 1e-6:
            continue
        direction = a / np.linalg.norm(a)
        next_direction = b / np.linalg.norm(b)
        if np.dot(direction, next_direction) > math.cos(math.radians(8)):
            continue
        keep.append(current)
    keep.append(points[-1])
    points = np.asarray(keep)
    output = [points[0]]
    for i in range(1, len(points) - 1):
        previous, corner, following = points[i - 1:i + 2]
        incoming, outgoing = corner - previous, following - corner
        length_in, length_out = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        u, v = incoming / length_in, outgoing / length_out
        angle = math.acos(float(np.clip(np.dot(u, v), -1.0, 1.0)))
        turn = np.cross(u, v)
        if angle < math.radians(8) or abs(turn) < 1e-6 or angle > math.radians(170):
            output.append(corner)
            continue
        tangent = min(radius * math.tan(angle / 2), 0.42 * length_in, 0.42 * length_out)
        actual_radius = tangent / math.tan(angle / 2)
        start, end = corner - tangent * u, corner + tangent * v
        normal = np.array([-u[1], u[0]]) * np.sign(turn)
        center = start + actual_radius * normal
        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        angles = start_angle + np.linspace(0, np.sign(turn) * angle,
                                           max(3, int(math.ceil(actual_radius * angle / spacing)) + 1))
        output.extend(center + actual_radius * np.column_stack([np.cos(angles), np.sin(angles)]))
    output.append(points[-1])
    return np.asarray(output)


if __name__ == "__main__":
    graph = RoadGraph()
    candidate = graph.report["candidate_route"]
    for index in (24, 27, 28, 29, 30, 31, 34, 37):
        node = candidate["osm_node_ids"][index]
        print(index, node, graph.points[node].round(2).tolist(),
              "degree", len(graph.edges[node]))
