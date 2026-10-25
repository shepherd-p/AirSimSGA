"""Make OSM2World materials usable in ProjectAirSim's runtime PBR importer."""

import argparse
import hashlib
import json
from pathlib import Path
import struct


HERE = Path(__file__).resolve().parent
GLB_MAGIC = 0x46546C67
JSON_CHUNK = 0x4E4F534A
BIN_CHUNK = 0x004E4942
SPECULAR_MATERIALS = {"Metal002", "GLASS_WALL", "Windows"}
HIGH_FREQUENCY_NORMALS = {"Concrete034", "RoofingTiles010", "Plaster002", "TERRAIN_DEFAULT"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=HERE / "generated/south_lake_union_runtime_512.glb")
    parser.add_argument("--output", type=Path, default=HERE / "generated/south_lake_union_runtime_matte.glb")
    args = parser.parse_args()
    source = args.input.read_bytes()
    magic, version, declared_size = struct.unpack_from("<III", source)
    if magic != GLB_MAGIC or version != 2 or declared_size != len(source):
        raise ValueError("Invalid glTF 2.0 binary header")
    json_size, json_type = struct.unpack_from("<II", source, 12)
    if json_type != JSON_CHUNK:
        raise ValueError("Missing GLB JSON chunk")
    model = json.loads(source[20:20 + json_size])
    binary_header = 20 + json_size
    binary_size, binary_type = struct.unpack_from("<II", source, binary_header)
    if binary_type != BIN_CHUNK:
        raise ValueError("Missing GLB binary chunk")
    binary = source[binary_header + 8:binary_header + 8 + binary_size]
    changes = []
    for material in model["materials"]:
        name = material.get("name", "")
        pbr = material.setdefault("pbrMetallicRoughness", {})
        pbr["metallicFactor"] = 0.15 if name in SPECULAR_MATERIALS else 0.0
        pbr["roughnessFactor"] = 0.65 if name in SPECULAR_MATERIALS else 0.85
        if name in HIGH_FREQUENCY_NORMALS:
            material.pop("normalTexture", None)
        changes.append({"material": name, "metallic_factor": pbr["metallicFactor"],
                        "roughness_factor": pbr["roughnessFactor"],
                        "normal_texture_removed": name in HIGH_FREQUENCY_NORMALS})
    json_data = json.dumps(model, separators=(",", ":")).encode("utf-8")
    json_data += b" " * ((-len(json_data)) % 4)
    total = 12 + 8 + len(json_data) + 8 + len(binary)
    with args.output.open("wb") as stream:
        stream.write(struct.pack("<III", GLB_MAGIC, 2, total))
        stream.write(struct.pack("<II", len(json_data), JSON_CHUNK))
        stream.write(json_data)
        stream.write(struct.pack("<II", len(binary), BIN_CHUNK))
        stream.write(binary)
    receipt = {"source_sha256": hashlib.sha256(source).hexdigest(),
               "output_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
               "changes": changes}
    args.output.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "bytes": total, "materials": len(changes)}))


if __name__ == "__main__":
    main()
