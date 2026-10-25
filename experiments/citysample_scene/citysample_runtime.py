"""ProjectAirSim backend for City Sample's native world geometry."""

from __future__ import annotations

from pathlib import Path
import math
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from ag.backend import Backend


class CitySampleBackend(Backend):
    async def drone_velocity(self, name, velocity, yaw_rate=0.):
        await self.settle_task(self.robots[name].move_by_velocity_async(
            *[float(value) for value in velocity], duration=.5,
            yaw_is_rate=True, yaw=float(yaw_rate)), 5)

    def load(self):
        from projectairsim import Drone, Rover, World
        from projectairsim.types import Pose
        from urban_scene.urban_runtime import barrier_glb

        self.world = World(self.client, self.config["scene"],
                           sim_config_path=str(HERE), delay_after_load_sec=2)
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
                        leaf = camera_leaves.get(setting["image-type"])
                        if leaf and setting.get("streaming-enabled"):
                            key = name + ("/Overview" if name == "Observer" else "") + "/" + leaf
                            self.subscribe(key, base + leaf)
        if set(self.world.list_actors()) != {actor["name"] for actor in scene["actors"]}:
            raise RuntimeError("Loaded robot list does not match City Sample scene config")
        self.closure_actor = None
        self.closure_actors = {}
        if "closure" in self.config:
            closure = self.config["closure"]
            yaw = math.radians(closure["yaw_deg"])
            pose = Pose({"translation": dict(zip("xyz", closure["center_ned"])),
                         "rotation": {"w": math.cos(yaw / 2), "x": 0, "y": 0,
                                      "z": math.sin(yaw / 2)}})
            self.closure_payload = barrier_glb()
            self.closure_actor = self.world.spawn_object_from_file(
                "AGCitySampleLaneClosure", "gltf", self.closure_payload, True,
                pose, closure["size_m"], False)
            if not self.closure_actor:
                raise RuntimeError("Could not spawn City Sample lane closure")
            self.spawned.append(self.closure_actor)
            if not self.world.set_segmentation_id_by_name(
                    self.closure_actor, closure["segmentation_id"], False, True):
                raise RuntimeError("Could not label the lane closure for camera perception")
        if self.config.get("closures"):
            self.closure_payload = barrier_glb()
            for closure in self.config["closures"]:
                yaw = math.radians(closure["yaw_deg"])
                pose = Pose({"translation": dict(zip("xyz", closure["center_ned"])),
                             "rotation": {"w": math.cos(yaw / 2), "x": 0, "y": 0,
                                          "z": math.sin(yaw / 2)}})
                actor = self.world.spawn_object_from_file(
                    closure["name"], "gltf", self.closure_payload, True,
                    pose, closure["size_m"], False)
                if not actor:
                    raise RuntimeError(f"Could not spawn {closure['name']}")
                self.spawned.append(actor)
                self.closure_actors[closure["name"]] = actor
                if self.config.get("allocate_segmentation_ids"):
                    used = set(self.world.get_segmentation_id_map().values())
                    free = next((identifier for identifier in range(255, 0, -1)
                                 if identifier not in used), None)
                    if free is None:
                        raise RuntimeError("No free segmentation ID for road closure")
                    closure["segmentation_id"] = free
                if not self.world.set_segmentation_id_by_name(
                        actor, closure["segmentation_id"], False, True):
                    raise RuntimeError(f"Could not label {closure['name']}")
        self.world.resume()
        return scene
