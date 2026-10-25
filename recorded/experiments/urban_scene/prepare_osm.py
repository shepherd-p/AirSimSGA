"""Clip a downloaded OSM API extract to the study area without inventing roads."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


BBOX = (-122.341, 47.621, -122.333, 47.627)  # west, south, east, north
ROOT = Path(__file__).resolve().parent
SOURCE_URL = (
    "https://api.openstreetmap.org/api/0.6/map"
    "?bbox=-122.341,47.621,-122.333,47.627"
)


def inside(node, bbox):
    west, south, east, north = bbox
    return south <= float(node.get("lat")) <= north and west <= float(node.get("lon")) <= east


def clipped_tree(source, bbox):
    original = ET.parse(source).getroot()
    output = ET.Element("osm", original.attrib)
    ET.SubElement(
        output, "bounds",
        minlon=str(bbox[0]),
        minlat=str(bbox[1]),
        maxlon=str(bbox[2]),
        maxlat=str(bbox[3]),
    )
    nodes = {node.get("id") for node in original.findall("node") if inside(node, bbox)}
    counts = {"nodes": 0, "ways": 0, "buildings": 0, "motor_roads": 0}
    for node in original.findall("node"):
        if node.get("id") in nodes:
            output.append(node)
            counts["nodes"] += 1
    for way in original.findall("way"):
        refs = [ref.get("ref") for ref in way.findall("nd")]
        if not refs or any(ref not in nodes for ref in refs):
            continue
        output.append(way)
        counts["ways"] += 1
        tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        counts["buildings"] += int("building" in tags)
        counts["motor_roads"] += int(
            tags.get("highway") in {
                "motorway", "trunk", "primary", "secondary", "tertiary",
                "unclassified", "residential", "service", "living_street",
            }
        )
    # Relations crossing the API bounding box are incomplete. Keep their member
    # ways, but omit the partial relation to avoid creating distant geometry.
    return ET.ElementTree(output), counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "source/south_lake_union.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "source/south_lake_union_clipped.osm")
    args = parser.parse_args()
    tree, counts = clipped_tree(args.source, BBOX)
    if counts["buildings"] < 30 or counts["motor_roads"] < 30:
        raise ValueError(f"Insufficient urban geometry after clipping: {counts}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    record = {
        "source_url": SOURCE_URL,
        "source_data_license": "OpenStreetMap contributors, ODbL 1.0",
        "converter": "OSM2World 0.4.0, MIT",
        "bbox_west_south_east_north": BBOX,
        "clip_rule": "Keep nodes inside bbox and ways with every referenced node inside; omit incomplete relations",
        "source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
        "clipped_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "counts": counts,
    }
    (args.output.parent / "provenance.json").write_text(
        json.dumps(record, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
