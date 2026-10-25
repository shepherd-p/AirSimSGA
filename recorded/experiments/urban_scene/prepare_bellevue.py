"""Download and clip a separate OSM district for the interception study."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import urllib.request

from prepare_osm import clipped_tree


HERE = Path(__file__).resolve().parent
BBOX = (-122.207, 47.607, -122.199, 47.613)
URL = ("https://api.openstreetmap.org/api/0.6/map?"
       "bbox=-122.207,47.607,-122.199,47.613")


def main():
    source = HERE / "source/bellevue_downtown.osm"
    clipped = HERE / "source/bellevue_downtown_clipped.osm"
    if not source.exists():
        request = urllib.request.Request(URL, headers={"User-Agent": "ProjectAirSim-AG-urban-study/1.0"})
        with urllib.request.urlopen(request, timeout=90) as response:
            source.write_bytes(response.read())
    tree, counts = clipped_tree(source, BBOX)
    if counts["buildings"] < 25 or counts["motor_roads"] < 25:
        raise ValueError(f"Insufficient independent downtown geometry: {counts}")
    tree.write(clipped, encoding="utf-8", xml_declaration=True)
    record = {
        "source_url": URL,
        "source_data_license": "OpenStreetMap contributors, ODbL 1.0",
        "converter": "OSM2World 0.4.0, MIT",
        "bbox_west_south_east_north": BBOX,
        "clip_rule": "Keep nodes inside bbox and ways with every referenced node inside; omit incomplete relations",
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "clipped_sha256": hashlib.sha256(clipped.read_bytes()).hexdigest(),
        "counts": counts,
    }
    (HERE / "source/bellevue_provenance.json").write_text(
        json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
