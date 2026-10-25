"""Resize embedded GLB textures while preserving geometry and material links."""

import argparse
from io import BytesIO
import json
from pathlib import Path
import struct

from PIL import Image


HERE = Path(__file__).resolve().parent
GLB_MAGIC = 0x46546C67
JSON_CHUNK = 0x4E4F534A
BIN_CHUNK = 0x004E4942


def padded(data, fill=b"\0"):
    return data + fill * ((-len(data)) % 4)


def resize(source, output, max_size):
    data = source.read_bytes()
    magic, version, declared_size = struct.unpack_from("<III", data)
    if magic != GLB_MAGIC or version != 2 or declared_size != len(data):
        raise ValueError("Invalid glTF 2.0 binary header")
    json_size, json_type = struct.unpack_from("<II", data, 12)
    if json_type != JSON_CHUNK:
        raise ValueError("Missing GLB JSON chunk")
    model = json.loads(data[20:20 + json_size])
    binary_header = 20 + json_size
    binary_size, binary_type = struct.unpack_from("<II", data, binary_header)
    if binary_type != BIN_CHUNK:
        raise ValueError("Missing GLB binary chunk")
    binary = data[binary_header + 8:binary_header + 8 + binary_size]
    image_views = {image["bufferView"]: image for image in model["images"]}
    views = sorted(enumerate(model["bufferViews"]), key=lambda item: item[1]["byteOffset"])
    rebuilt = bytearray()
    resized = []
    last_end = 0
    for index, view in views:
        start = view["byteOffset"]
        end = start + view["byteLength"]
        if start < last_end or end > len(binary):
            raise ValueError("Overlapping or invalid GLB buffer views")
        rebuilt.extend(binary[last_end:start])
        rebuilt.extend(b"\0" * ((-len(rebuilt)) % 4))
        payload = binary[start:end]
        image_info = image_views.get(index)
        if image_info is not None:
            image = Image.open(BytesIO(payload))
            original_size = image.size
            image.thumbnail((max_size, max_size), Image.Resampling.LANCZOS)
            encoded = BytesIO()
            mime = image_info["mimeType"]
            if mime == "image/jpeg":
                image.convert("RGB").save(encoded, format="JPEG", quality=88, subsampling=0)
            elif mime == "image/png":
                image.save(encoded, format="PNG", optimize=True)
            else:
                raise ValueError("Unsupported embedded image: " + mime)
            payload = encoded.getvalue()
            resized.append({"image": len(resized), "original": original_size, "output": image.size})
        view["byteOffset"] = len(rebuilt)
        view["byteLength"] = len(payload)
        rebuilt.extend(payload)
        last_end = end
    rebuilt.extend(binary[last_end:])
    rebuilt = padded(bytes(rebuilt))
    model["buffers"][0]["byteLength"] = len(rebuilt)
    json_payload = padded(json.dumps(model, separators=(",", ":")).encode("utf-8"), b" ")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(struct.pack("<III", GLB_MAGIC, 2, 12 + 8 + len(json_payload) + 8 + len(rebuilt)))
        stream.write(struct.pack("<II", len(json_payload), JSON_CHUNK))
        stream.write(json_payload)
        stream.write(struct.pack("<II", len(rebuilt), BIN_CHUNK))
        stream.write(rebuilt)
    return resized


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=HERE / "generated/south_lake_union_runtime.glb")
    parser.add_argument("--output", type=Path, default=HERE / "generated/south_lake_union_runtime_512.glb")
    parser.add_argument("--max-size", type=int, default=512)
    args = parser.parse_args()
    if args.max_size < 128:
        parser.error("--max-size must be at least 128")
    resized = resize(args.input, args.output, args.max_size)
    print(json.dumps({"source": str(args.input), "output": str(args.output),
                      "image_count": len(resized), "reduced": sum(x["output"] != x["original"] for x in resized),
                      "bytes": args.output.stat().st_size}))


if __name__ == "__main__":
    main()
