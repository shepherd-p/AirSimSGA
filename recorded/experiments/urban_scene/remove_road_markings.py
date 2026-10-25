"""Remove raised OSM2World paint meshes that interfere with road collision."""

import hashlib
import json
from pathlib import Path
import struct


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "generated/south_lake_union_runtime_512.glb"
OUTPUT = HERE / "generated/south_lake_union_runtime_512_no_markings.glb"
MARKING_NAMES = {
    "road_marking_crossing", "road_arrow_right", "road_arrow_through_right",
    "road_arrow_through", "road_marking_dash",
}


def main():
    source = SOURCE.read_bytes()
    magic, version, total = struct.unpack_from("<III", source)
    if (magic, version, total) != (0x46546C67, 2, len(source)):
        raise ValueError("Invalid GLB source")
    json_size, json_type = struct.unpack_from("<II", source, 12)
    if json_type != 0x4E4F534A:
        raise ValueError("Missing JSON chunk")
    model = json.loads(source[20:20 + json_size])
    binary_start = 20 + json_size
    binary_size, binary_type = struct.unpack_from("<II", source, binary_start)
    if binary_type != 0x004E4942:
        raise ValueError("Missing binary chunk")
    binary = source[binary_start + 8:binary_start + 8 + binary_size]
    removed = []
    for mesh in model["meshes"]:
        retained = []
        for primitive in mesh["primitives"]:
            name = model["materials"][primitive["material"]].get("name", "")
            if name in MARKING_NAMES:
                accessor = model["accessors"][primitive["indices"]]
                removed.append({"material": name, "triangles": accessor["count"] // 3})
            else:
                retained.append(primitive)
        mesh["primitives"] = retained
    if set(item["material"] for item in removed) != MARKING_NAMES:
        raise ValueError("Unexpected marking material inventory")
    encoded = json.dumps(model, separators=(",", ":")).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    rebuilt = (struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(encoded) + 8 + len(binary))
               + struct.pack("<I4s", len(encoded), b"JSON") + encoded
               + struct.pack("<I4s", len(binary), b"BIN\x00") + binary)
    OUTPUT.write_bytes(rebuilt)
    receipt = {
        "source": str(SOURCE), "source_sha256": hashlib.sha256(source).hexdigest(),
        "output": str(OUTPUT), "output_sha256": hashlib.sha256(rebuilt).hexdigest(),
        "change": "Removed only five named road marking primitives; binary vertex data and all other primitives retained",
        "removed": removed,
    }
    OUTPUT.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
