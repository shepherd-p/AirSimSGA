"""Create traceable navigation figures from a completed aerial-route run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from annotate_district_navigation import camera_projector, digest, font, label


UGV = "#e34a3b"
UAV = "#00b6c7"
PLAN = "#f4c43a"
INK = "#14252a"


def panel_label(draw, text, anchor, color, image_size):
    face = font(28)
    x = max(8, min(int(anchor[0] + 12), image_size[0] - 110))
    y = max(8, min(int(anchor[1] - 47), image_size[1] - 45))
    bounds = draw.textbbox((0, 0), text, font=face)
    width = bounds[2] - bounds[0] + 20
    draw.rectangle((x, y, x + width, y + 40), fill=INK,
                   outline=color, width=2)
    draw.text((x + 10, y + 5 - bounds[1]), text, font=face, fill="white")


def rows_for(run, actor):
    with (run / "trajectories_controls.csv").open(newline="", encoding="utf-8") as stream:
        return [row for row in csv.DictReader(stream) if row["robot"] == actor]


def positions(rows):
    return np.array([[float(row[axis]) for axis in "xyz"] for row in rows])


def line(draw, pixels, color, width):
    points = [tuple(map(float, pixel)) for pixel in pixels]
    if len(points) > 1:
        draw.line(points, fill=INK, width=width + 3, joint="curve")
        draw.line(points, fill=color, width=width, joint="curve")


def overview(run, scenario, plan, output):
    source = run / "scene_capture.png"
    packet = source.with_suffix(".npz")
    image = Image.open(source).convert("RGB")
    project, _ = camera_projector(packet, image.size)
    draw = ImageDraw.Draw(image)
    route = np.asarray(plan["waypoints_ned"], dtype=float)
    line(draw, project(route), PLAN, 2)
    paths = {}
    for actor, color, name in (("Car1", UGV, "UGV"),
                               ("Drone1", UAV, "UAV")):
        rows = rows_for(run, actor)
        if not rows:
            raise RuntimeError(f"No recorded pose for {actor}")
        world = positions(rows)
        pixels = project(world)
        line(draw, pixels, color, 3)
        endpoint = tuple(map(float, pixels[-1]))
        x, y = endpoint
        draw.ellipse((x - 8, y - 8, x + 8, y + 8),
                     fill=color, outline="white", width=2)
        label(draw, name, endpoint, color, image.size,
              offset=(-85, -44) if name == "UAV" else (12, -44))
        paths[name] = {"pose_samples": len(rows), "last_world_ned": world[-1].tolist(),
                       "last_pixel": pixels[-1].tolist()}
    for index, junction in enumerate(scenario["survey_junctions"], 1):
        x, y = map(float, project([junction["center_ned"]])[0])
        draw.ellipse((x - 12, y - 12, x + 12, y + 12),
                     fill=INK, outline=PLAN, width=3)
        draw.text((x - 7, y - 10), str(index), font=font(20), fill="white")
    for text, point, offset in (("Start", scenario["start_ned"], (10, 10)),
                                ("Goal", scenario["goal_ned"], (-90, -40))):
        pixel = project([point])[0]
        label(draw, text, pixel, PLAN, image.size, offset=offset)
    image.save(output, optimize=True)
    return {"source": str(source), "source_sha256": digest(source),
            "camera_packet": str(packet), "camera_packet_sha256": digest(packet),
            "trajectory_sha256": digest(run / "trajectories_controls.csv"),
            "plan_sha256": digest(run / "committed_plan.json"),
            "paths": paths, "output": str(output),
            "output_sha256": digest(output)}


def aerial_panel(run, junction, index, route, output):
    source = run / f"aerial_junction_{index}.png"
    packet = source.with_suffix(".npz")
    source_image = Image.open(source).convert("RGB")
    project, _ = camera_projector(packet, source_image.size)
    scale = 640 / source_image.width
    image = source_image.resize((640, 360), Image.Resampling.LANCZOS)
    draw = ImageDraw.Draw(image)
    center = np.asarray(junction["center_ned"], dtype=float)
    near = np.linalg.norm(route[:, :2] - center[:2], axis=1) < 75
    selected = route[near]
    if len(selected) > 1:
        line(draw, project(selected) * scale, PLAN, 2)
    x, y = map(float, project([center])[0] * scale)
    draw.ellipse((x - 12, y - 12, x + 12, y + 12),
                 fill=INK, outline=PLAN, width=3)
    panel_label(draw, f"J{index}", (x, y), PLAN, image.size)
    image.save(output, optimize=True)
    return {"source": str(source), "source_sha256": digest(source),
            "camera_packet_sha256": digest(packet),
            "projected_route_points": len(selected),
            "output": str(output), "output_sha256": digest(output)}


def vehicle_box(pose, yaw):
    center = np.asarray(pose, dtype=float)
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    right = np.array([-forward[1], forward[0]])
    corners = []
    for length in (-2.2, 2.2):
        for width in (-1.1, 1.1):
            xy = center[:2] + length * forward + width * right
            for height in (-1.1, .6):
                corners.append([xy[0], xy[1], center[2] + height])
    return corners


def ugv_panel(run, crossing, index, output):
    source = run / f"ugv_junction_{index}.png"
    packet = source.with_suffix(".npz")
    image = Image.open(source).convert("RGB")
    project, _ = camera_projector(packet, image.size)
    rows = rows_for(run, "Car1")
    nearest = min(rows, key=lambda row: abs(float(row["sim_time_sec"]) -
                                        crossing["sim_time_sec"]))
    yaw = float(nearest["yaw_rad"])
    corners = project(vehicle_box(crossing["car_pose_ned"], yaw))
    minimum, maximum = corners.min(axis=0), corners.max(axis=0)
    center = (minimum + maximum) / 2
    crop_width, crop_height = 320, 180
    left = max(0, min(image.width - crop_width, round(center[0] - crop_width / 2)))
    top = max(0, min(image.height - crop_height, round(center[1] - crop_height / 2)))
    image = image.crop((left, top, left + crop_width, top + crop_height))
    image = image.resize((640, 360), Image.Resampling.LANCZOS)
    box = [(float(p[0]) - left) * 2 for p in (minimum, maximum)]
    box_y = [(float(p[1]) - top) * 2 for p in (minimum, maximum)]
    bounds = (box[0] - 5, box_y[0] - 5, box[1] + 5, box_y[1] + 5)
    draw = ImageDraw.Draw(image)
    draw.rectangle(bounds, outline="white", width=6)
    draw.rectangle(bounds, outline=UGV, width=3)
    panel_label(draw, "UGV", (bounds[0], bounds[1]), UGV, image.size)
    image.save(output, optimize=True)
    return {"source": str(source), "source_sha256": digest(source),
            "camera_packet_sha256": digest(packet),
            "crossing_sim_time_sec": crossing["sim_time_sec"],
            "recorded_car_pose_ned": crossing["car_pose_ned"],
            "nearest_pose_time_sec": float(nearest["sim_time_sec"]),
            "box_method": "projected approximate 4.4 x 2.2 x 1.7 m vehicle extent",
            "crop_box_px": [left, top, left + crop_width, top + crop_height],
            "output": str(output), "output_sha256": digest(output)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    if summary["status"] != "completed" or len(summary["junction_crossings"]) != 3:
        raise RuntimeError("Figures require a completed three-junction UGV run")
    scenario = json.loads((run / "scenario.json").read_text(encoding="utf-8"))
    plan = json.loads((run / "committed_plan.json").read_text(encoding="utf-8"))
    route = np.asarray(plan["waypoints_ned"], dtype=float)
    output = run / "derived"
    output.mkdir(exist_ok=True)
    manifest = {"run": str(run),
                "annotation_source_sha256": digest(Path(__file__)),
                "summary_sha256": digest(run / "summary.json"),
                "scenario_sha256": digest(run / "scenario.json"),
                "overlay_note": "Paths and vehicle boxes are projected from recorded poses and the saved plan; they are postprocessing, not RGB detections.",
                "figures": {}}
    manifest["figures"]["overview"] = overview(
        run, scenario, plan, output / "aerial_route_overview.png")
    contact_sheet = Image.new("RGB", (1920, 720), "white")
    for index, (junction, crossing) in enumerate(zip(
            scenario["survey_junctions"], summary["junction_crossings"]), 1):
        aerial = output / f"aerial_junction_{index}_marked.png"
        ground = output / f"ugv_junction_{index}_marked.png"
        manifest["figures"][f"aerial_junction_{index}"] = aerial_panel(
            run, junction, index, route, aerial)
        manifest["figures"][f"ugv_junction_{index}"] = ugv_panel(
            run, crossing, index, ground)
        contact_sheet.paste(Image.open(aerial), ((index - 1) * 640, 0))
        contact_sheet.paste(Image.open(ground), ((index - 1) * 640, 360))
    sheet = output / "aerial_ugv_junctions.png"
    contact_sheet.save(sheet, optimize=True)
    manifest["figures"]["junctions"] = {
        "output": str(sheet), "output_sha256": digest(sheet)}
    (output / "annotation_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
