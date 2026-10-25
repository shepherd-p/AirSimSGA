"""Identify GLB triangles near a measured ProjectAirSim collision point."""

import argparse
import json
from pathlib import Path
import struct

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("glb", type=Path)
    parser.add_argument("--xyz", type=float, nargs=3, default=[-19.5997, .8425, 87.4439],
                        help="Impact in GLB X, height Y, GLB Z coordinates")
    args = parser.parse_args()
    source = args.glb.read_bytes()
    json_size = struct.unpack_from("<I", source, 12)[0]
    model = json.loads(source[20:20 + json_size])
    binary = memoryview(source)[20 + json_size + 8:]
    target = np.asarray(args.xyz)
    hits = []

    def array(accessor_id):
        accessor = model["accessors"][accessor_id]
        view = model["bufferViews"][accessor["bufferView"]]
        components = {"SCALAR": 1, "VEC3": 3}[accessor["type"]]
        dtype = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32,
                 5126: np.float32}[accessor["componentType"]]
        offset = view["byteOffset"] + accessor.get("byteOffset", 0)
        stride = view.get("byteStride", np.dtype(dtype).itemsize * components)
        values = np.ndarray((accessor["count"], components), dtype=dtype,
                            buffer=binary, offset=offset,
                            strides=(stride, np.dtype(dtype).itemsize))
        return values.copy()

    for mesh in model["meshes"]:
        for primitive in mesh["primitives"]:
            points = array(primitive["attributes"]["POSITION"])
            indices = array(primitive["indices"]).reshape(-1, 3)
            triangles = points[indices]
            minimum = triangles.min(axis=1)
            maximum = triangles.max(axis=1)
            # Include triangles above and below the vehicle bumper impact.
            nearby = np.where(np.all(minimum <= target + [1, 2, 2], axis=1) &
                              np.all(maximum >= target - [1, 2, 2], axis=1))[0]
            material = model["materials"][primitive["material"]].get("name", "")
            for index in nearby:
                triangle = triangles[index].astype(float)
                normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
                length = np.linalg.norm(normal)
                if length < 1e-9:
                    continue
                normal /= length
                distance = abs(np.dot(target - triangle[0], normal))
                hits.append({"material": material, "triangle": int(index),
                             "normal": normal.round(4).tolist(),
                             "plane_distance_m": round(float(distance), 4),
                             "vertices": triangle.round(4).tolist()})
    hits.sort(key=lambda item: item["plane_distance_m"])
    print(json.dumps(hits[:30], indent=2))


if __name__ == "__main__":
    main()
