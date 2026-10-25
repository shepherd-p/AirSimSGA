"""Audit drivable OSM connectivity and register it to the generated GLB."""

import argparse
from collections import defaultdict
import hashlib
import heapq
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


HERE = Path(__file__).resolve().parent
DRIVABLE = {
    "primary", "secondary", "tertiary", "unclassified", "residential",
    "service", "living_street",
}


def load_glb(path):
    with path.open("rb") as stream:
        header = stream.read(20)
        magic, version, total, chunk_size, chunk_type = struct.unpack("<IIIII", header)
        if magic != 0x46546C67 or version != 2 or chunk_type != 0x4E4F534A:
            raise ValueError("Expected a binary glTF 2.0 file")
        model = json.loads(stream.read(chunk_size))
    if total != path.stat().st_size:
        raise ValueError("GLB declared size does not match file size")
    return model


def road_junction_centers(model):
    centers = {}
    for node in model["nodes"]:
        name = node.get("name", "")
        if not name.startswith("RoadJunction n") or "mesh" not in node:
            continue
        positions = []
        for primitive in model["meshes"][node["mesh"]]["primitives"]:
            accessor = model["accessors"][primitive["attributes"]["POSITION"]]
            positions.append((np.asarray(accessor["min"]), np.asarray(accessor["max"])))
        minimum = np.min([p[0] for p in positions], axis=0)
        maximum = np.max([p[1] for p in positions], axis=0)
        centers[name.split(" n", 1)[1]] = (minimum + maximum)[[0, 2]] / 2
    return centers


def dijkstra(graph, start):
    distance, previous = {start: 0.0}, {}
    queue = [(0.0, start)]
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != distance[node]:
            continue
        for neighbor, weight in graph[node].items():
            proposal = cost + weight
            if proposal < distance.get(neighbor, math.inf):
                distance[neighbor], previous[neighbor] = proposal, node
                heapq.heappush(queue, (proposal, neighbor))
    return distance, previous


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--osm", type=Path, default=HERE / "source/south_lake_union_clipped.osm")
    parser.add_argument("--glb", type=Path, default=HERE / "generated/south_lake_union_clipped.glb")
    parser.add_argument("--output", type=Path, default=HERE / "generated/road_network_report.json")
    parser.add_argument("--latitude0", type=float, default=47.624)
    parser.add_argument("--longitude0", type=float, default=-122.337)
    args = parser.parse_args()
    root = ET.parse(args.osm).getroot()
    nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.findall("node")}
    latitude0 = args.latitude0
    longitude0 = args.longitude0

    def metric(node_id):
        latitude, longitude = nodes[node_id]
        return np.array([
            (longitude - longitude0) * 111320 * math.cos(math.radians(latitude0)),
            (latitude - latitude0) * 111320,
        ])

    graph = defaultdict(dict)
    road_ways = []
    for way in root.findall("way"):
        tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        if tags.get("highway") not in DRIVABLE or tags.get("access") == "private":
            continue
        refs = [ref.get("ref") for ref in way.findall("nd")]
        road_ways.append(refs)
        for left, right in zip(refs, refs[1:]):
            length = float(np.linalg.norm(metric(left) - metric(right)))
            graph[left][right] = min(length, graph[left].get(right, math.inf))
            graph[right][left] = min(length, graph[right].get(left, math.inf))
    if not graph:
        raise ValueError("No connected drivable roads")

    remaining = set(graph)
    components = []
    while remaining:
        seed = min(remaining)
        reached = {seed}
        stack = [seed]
        while stack:
            for neighbor in graph[stack.pop()]:
                if neighbor not in reached:
                    reached.add(neighbor)
                    stack.append(neighbor)
        remaining.difference_update(reached)
        components.append(reached)
    largest = max(components, key=lambda component: (len(component), min(component)))
    first = min(largest)
    initial_distances, _ = dijkstra(graph, first)
    far = max(initial_distances, key=lambda node: (initial_distances[node], node))
    distances, previous = dijkstra(graph, far)
    end = max(distances, key=lambda node: (distances[node], node))
    route = [end]
    while route[-1] != far:
        route.append(previous[route[-1]])
    route.reverse()

    model = load_glb(args.glb)
    junctions = road_junction_centers(model)
    matches = [(metric(node_id), point) for node_id, point in junctions.items() if node_id in nodes]
    if len(matches) < 15:
        raise ValueError("Too few common OSM road junctions in GLB")
    source = np.array([[*point, 1.0] for point, _ in matches])
    target = np.array([point for _, point in matches])
    transform, _, _, _ = np.linalg.lstsq(source, target, rcond=None)
    errors = np.linalg.norm(source @ transform - target, axis=1)
    if float(np.median(errors)) > 3.0:
        raise ValueError("OSM-to-GLB registration residual is too large")

    def model_point(node_id):
        x, z = [float(x) for x in np.append(metric(node_id), 1) @ transform]
        return [x, z]

    report = {
        "source_osm_sha256": hashlib.sha256(args.osm.read_bytes()).hexdigest(),
        "model_glb_sha256": hashlib.sha256(args.glb.read_bytes()).hexdigest(),
        "registration_origin_lat_lon": [latitude0, longitude0],
        "road_way_count": len(road_ways),
        "road_graph_nodes": len(graph),
        "connected_components": len(components),
        "largest_component_nodes": len(largest),
        "largest_component_intersections_degree_at_least_3": sum(len(graph[n]) >= 3 for n in largest),
        "matched_junctions": len(matches),
        "registration_median_error_m": float(np.median(errors)),
        "registration_p95_error_m": float(np.percentile(errors, 95)),
        "metric_east_north_to_glb_x_z": transform.tolist(),
        "candidate_route": {
            "length_m": distances[end],
            "osm_node_ids": route,
            "glb_x_z_m": [model_point(node_id) for node_id in route],
        },
    }
    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "candidate_route"}, indent=2))
    print("candidate_route_length_m", round(distances[end], 1))


if __name__ == "__main__":
    main()
