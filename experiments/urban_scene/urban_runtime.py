"""Live ProjectAirSim loading and compact evidence capture for urban studies."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct

import numpy as np

from ag.backend import Backend
from ag.contact import is_ground_contact
from ag.storage import serial, write_json
from ag.portable import output_root


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MESH = HERE / "generated/south_lake_union_runtime_512_no_markings.glb"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def new_run(study, variant):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = output_root() / study / f"{stamp}_{variant}"
    path.mkdir(parents=True, exist_ok=False)
    return path


def barrier_glb():
    """A unit box with flat normals and a distinct material, encoded as glTF 2.0."""
    faces = [
        ((1, 0, 0), [( .5,-.5,-.5), ( .5, .5,-.5), ( .5, .5, .5), ( .5,-.5, .5)]),
        ((-1, 0, 0), [(-.5, .5,-.5), (-.5,-.5,-.5), (-.5,-.5, .5), (-.5, .5, .5)]),
        ((0, 1, 0), [( .5, .5,-.5), (-.5, .5,-.5), (-.5, .5, .5), ( .5, .5, .5)]),
        ((0,-1, 0), [(-.5,-.5,-.5), ( .5,-.5,-.5), ( .5,-.5, .5), (-.5,-.5, .5)]),
        ((0, 0, 1), [(-.5,-.5, .5), ( .5,-.5, .5), ( .5, .5, .5), (-.5, .5, .5)]),
        ((0, 0,-1), [( .5,-.5,-.5), (-.5,-.5,-.5), (-.5, .5,-.5), ( .5, .5,-.5)]),
    ]
    positions = [coordinate for _, vertices in faces for vertex in vertices for coordinate in vertex]
    normals = [coordinate for normal, _ in faces for _ in range(4) for coordinate in normal]
    indices = [4 * face + index for face in range(6) for index in (0, 1, 2, 0, 2, 3)]
    position_bytes = struct.pack("<" + "f" * len(positions), *positions)
    normal_bytes = struct.pack("<" + "f" * len(normals), *normals)
    index_bytes = struct.pack("<" + "H" * len(indices), *indices)
    binary = position_bytes + normal_bytes + index_bytes
    binary += b"\x00" * (-len(binary) % 4)
    model = {
        "asset": {"version": "2.0", "generator": "AG urban barrier_glb"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(position_bytes), "target": 34962},
            {"buffer": 0, "byteOffset": len(position_bytes), "byteLength": len(normal_bytes), "target": 34962},
            {"buffer": 0, "byteOffset": len(position_bytes) + len(normal_bytes),
             "byteLength": len(index_bytes), "target": 34963},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 24, "type": "VEC3",
             "min": [-.5, -.5, -.5], "max": [.5, .5, .5]},
            {"bufferView": 1, "componentType": 5126, "count": 24, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5123, "count": 36, "type": "SCALAR"},
        ],
        "materials": [{"name": "RoadClosureOrange", "doubleSided": True,
                       "pbrMetallicRoughness": {"baseColorFactor": [.96, .25, .025, 1],
                                                 "metallicFactor": 0, "roughnessFactor": 1}}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1},
                                      "indices": 2, "material": 0}]}],
    }
    json_bytes = json.dumps(model, separators=(",", ":")).encode("utf-8")
    json_bytes += b" " * (-len(json_bytes) % 4)
    total = 12 + 8 + len(json_bytes) + 8 + len(binary)
    return (struct.pack("<III", 0x46546C67, 2, total)
            + struct.pack("<I4s", len(json_bytes), b"JSON") + json_bytes
            + struct.pack("<I4s", len(binary), b"BIN\x00") + binary)


class UrbanBackend(Backend):
    def load(self):
        from projectairsim import Drone, Rover, World
        from projectairsim.types import Pose

        self.world = World(self.client, self.config["scene"],
                           sim_config_path=str(HERE), delay_after_load_sec=1)
        scene = self.world.get_configuration()
        camera_leaves = {0: "scene_camera", 2: "depth_camera", 3: "segmentation_camera"}
        for actor in scene["actors"]:
            name = actor["name"]
            root = f"{self.world.parent_topic}/robots/{name}"
            if name != "Observer":
                kind = "uav" if name.startswith("Drone") else "ugv"
                self.kinds[name] = kind
                self.robots[name] = (Drone if kind == "uav" else Rover)(self.client, self.world, name)
                self.subscribe(name + "/pose", root + "/actual_pose")
                self.subscribe(name + "/collision", root + "/collision_info")
            for sensor in actor["robot-config"].get("sensors", []):
                if not sensor.get("enabled", False):
                    continue
                base = root + "/sensors/" + sensor["id"] + "/"
                if sensor["type"] == "lidar":
                    self.subscribe(name + "/lidar", base + "lidar")
                elif sensor["type"] == "camera":
                    for setting in sensor["capture-settings"]:
                        if setting.get("streaming-enabled") and setting["image-type"] in camera_leaves:
                            leaf = camera_leaves[setting["image-type"]]
                            key = name + ("/Overview" if name == "Observer" else "") + "/" + leaf
                            self.subscribe(key, base + leaf)
        self.mesh_path = Path(self.config.get("mesh_path", MESH))
        payload = self.mesh_path.read_bytes()
        origin = Pose({"translation": {"x": 0, "y": 0, "z": 0},
                       "rotation": {"w": 1, "x": 0, "y": 0, "z": 0}})
        self.mesh_actor = self.world.spawn_object_from_file(
            "AGUrbanMesh", "gltf", payload, True, origin, [1, 1, 1], False)
        if self.mesh_actor not in self.world.list_objects("^" + self.mesh_actor + "$"):
            raise RuntimeError("Urban mesh did not register in the loaded scene")
        self.mesh_sha256 = hashlib.sha256(payload).hexdigest()
        self.closure_actor = None
        if "closure" in self.config:
            closure = self.config["closure"]
            position = closure["center_ned"]
            pose = Pose({"translation": dict(zip("xyz", position)),
                         "rotation": {"w": 1, "x": 0, "y": 0, "z": 0}})
            self.closure_payload = barrier_glb()
            self.closure_actor = self.world.spawn_object_from_file(
                "AGUrbanRoadClosure", "gltf", self.closure_payload, True,
                pose, closure["size_m"], False)
            if not self.closure_actor:
                raise RuntimeError("Could not spawn physical road closure")
            if not self.world.set_segmentation_id_by_name(
                    self.closure_actor, closure["segmentation_id"], False, True):
                raise RuntimeError("Could not label road closure for camera observation")
        if set(self.world.list_actors()) != {a["name"] for a in scene["actors"]}:
            raise RuntimeError("Configured and live actors differ")
        self.world.resume()
        return scene


def save_camera(backend, key, output):
    """Preserve the original payload alongside its lossless decoded PNG."""
    import cv2
    from projectairsim.utils import unpack_image

    message = backend.buffer.latest(key, 3.0)
    if message is None:
        return None
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    image = unpack_image(message)
    if not cv2.imwrite(str(output), image):
        raise OSError(f"Could not write {output}")
    payload = np.frombuffer(bytes(message["data"]), dtype=np.uint8)
    metadata = {field: value for field, value in message.items() if field != "data"}
    np.savez_compressed(output.with_suffix(".npz"), payload=payload,
                        metadata_json=json.dumps(serial(metadata)))
    return {"png": str(output), "payload": str(output.with_suffix(".npz")),
            "time_ns": int(message["time_stamp"]), "shape": list(image.shape),
            "payload_sha256": hashlib.sha256(bytes(message["data"])).hexdigest()}


def save_observation_pair(backend, name, output):
    pair = backend.buffer.pair(name + "/depth_camera", name + "/segmentation_camera", 2, .05)
    if pair is None:
        return None
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    arrays = {}
    metadata = {}
    for label, message in zip(("depth", "segmentation"), pair):
        arrays[label + "_payload"] = np.frombuffer(bytes(message["data"]), dtype=np.uint8)
        metadata[label] = {field: value for field, value in message.items() if field != "data"}
    np.savez_compressed(output, **arrays, metadata_json=json.dumps(serial(metadata)))
    return {"file": str(output), "depth_time_ns": int(pair[0]["time_stamp"]),
            "segmentation_time_ns": int(pair[1]["time_stamp"]),
            "pair_delta_ms": abs(int(pair[0]["time_stamp"]) - int(pair[1]["time_stamp"])) / 1e6}


def collision_snapshot(backend, names, last_stamps):
    events = []
    for name in names:
        event = backend.collision(name)
        if event is None or event["time_stamp"] == last_stamps.get(name):
            continue
        last_stamps[name] = event["time_stamp"]
        if not is_ground_contact(event):
            events.append({"robot": name, "event": event})
    return events


def receipt(path, status, **fields):
    write_json(Path(path) / "summary.json", {
        "status": status, "recorded_utc": datetime.now(timezone.utc).isoformat(), **fields,
    })
