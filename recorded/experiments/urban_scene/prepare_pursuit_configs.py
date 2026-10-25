"""Derive role-specific, matched low-bandwidth robot configs from validated vehicles."""

import copy
import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
CONFIGS = HERE.parent / "configs"
OUTPUT = HERE / "robots"


def build(source, destination, image_types, interval):
    robot = copy.deepcopy(json.loads((CONFIGS / source).read_text(encoding="utf-8")))
    for sensor in robot["sensors"]:
        if sensor["type"] == "lidar":
            sensor["enabled"] = False
        if sensor["type"] == "camera":
            sensor["capture-interval"] = interval
            sensor["capture-settings"] = [setting for setting in sensor["capture-settings"]
                                          if setting["image-type"] in image_types]
            for setting in sensor["capture-settings"]:
                setting["width"], setting["height"] = 640, 360
    OUTPUT.mkdir(exist_ok=True)
    (OUTPUT / destination).write_text(json.dumps(robot, indent=2) + "\n", encoding="utf-8")


def main():
    build("robot_uav_navigation_study.jsonc", "pursuit_uav.jsonc", {0, 2, 3}, .3)
    build("robot_ugv_navigation_study.jsonc", "pursuit_target.jsonc", {0, 2, 3}, .3)
    build("robot_ugv_navigation_study.jsonc", "pursuit_interceptor.jsonc", {0}, .5)


if __name__ == "__main__":
    main()
