"""Make a three-frame UGV bypass figure from synchronized simulator captures."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from annotate_district_navigation import (
    CLOSURE_COLOR, INK, UGV_COLOR, camera_projector, digest, font,
)


CROPS = {
    "approach": (405, 125, 855, 575),
    "beside_closure": (480, 200, 800, 520),
    "departure": (420, 135, 870, 585),
}
PANEL_SIZE = 820


def box_label(draw, text, x, y, color):
    face = font(25)
    bounds = draw.textbbox((0, 0), text, font=face)
    width, height = bounds[2] + 18, bounds[3] - bounds[1] + 12
    x = max(5, min(round(x), PANEL_SIZE - width - 5))
    y = max(5, min(round(y), PANEL_SIZE - height - 5))
    draw.rectangle((x, y, x + width, y + height), fill=INK,
                   outline=color, width=2)
    draw.text((x + 9, y + 5 - bounds[1]), text, font=face, fill="white")


def draw_box(draw, box, color):
    draw.rectangle(box, outline="white", width=7)
    draw.rectangle(box, outline=color, width=4)


def panel(run, capture, rows, output):
    stage = capture["stage"]
    source = Path(capture["frame"]["png"])
    packet = Path(capture["frame"]["payload"])
    image = Image.open(source).convert("RGB")
    frame_time = capture["frame"]["time_ns"] / 1e9
    car = min(rows, key=lambda row: abs(float(row["sim_time_sec"]) - frame_time))
    pose_time = float(car["sim_time_sec"])
    if abs(pose_time - frame_time) > .4:
        raise RuntimeError(f"No near-time UGV pose for {source}: {pose_time}")
    position = np.array([float(car[axis]) for axis in "xyz"])
    yaw = float(car["yaw_rad"])
    corners = np.array([
        [position[0] + math.cos(yaw) * length - math.sin(yaw) * width,
         position[1] + math.sin(yaw) * length + math.cos(yaw) * width,
         position[2] - .5]
        for length in (-2.4, 2.4) for width in (-1.2, 1.2)
    ])
    project, _ = camera_projector(packet, image.size)
    pixels = project(corners)
    car_box = [*pixels.min(axis=0), *pixels.max(axis=0)]
    closure_box = capture["closure_bbox_px"]
    closure_center = project([capture["closure_pose_ned"]])[0]
    closure_error = float(np.linalg.norm(
        closure_center - np.array([(closure_box[0] + closure_box[2]) / 2,
                                   (closure_box[1] + closure_box[3]) / 2])))
    if closure_error > 25:
        raise RuntimeError(f"Unsynchronized closure in {source}: {closure_error:.1f} px")

    crop = CROPS[stage]
    for box in (car_box, closure_box):
        if not (crop[0] < box[0] < box[2] < crop[2] and
                crop[1] < box[1] < box[3] < crop[3]):
            raise RuntimeError(f"{stage} subject is outside the crop: {box}")
    scale = PANEL_SIZE / (crop[2] - crop[0])

    def transformed(box, padding=0):
        return [(box[0] - crop[0]) * scale - padding,
                (box[1] - crop[1]) * scale - padding,
                (box[2] - crop[0]) * scale + padding,
                (box[3] - crop[1]) * scale + padding]

    image = image.crop(crop).resize((PANEL_SIZE, PANEL_SIZE),
                                    Image.Resampling.LANCZOS)
    draw = ImageDraw.Draw(image)
    car_panel_box = transformed(car_box, 8)
    closure_panel_box = transformed(closure_box, 7)
    draw_box(draw, car_panel_box, UGV_COLOR)
    draw_box(draw, closure_panel_box, CLOSURE_COLOR)
    box_label(draw, "UGV", car_panel_box[0], car_panel_box[3] + 8, UGV_COLOR)
    box_label(draw, "Road closure", closure_panel_box[0],
              closure_panel_box[1] - 47, CLOSURE_COLOR)
    image.save(output, optimize=True)
    return {
        "stage": stage,
        "source_png": str(source), "source_png_sha256": digest(source),
        "raw_packet": str(packet), "raw_packet_sha256": digest(packet),
        "frame_time_sec": frame_time, "car_pose_time_sec": pose_time,
        "car_pose_ned": position.tolist(), "car_yaw_rad": yaw,
        "car_bbox_source_px": car_box,
        "closure_bbox_source_px": closure_box,
        "closure_reprojection_error_px": closure_error,
        "crop_source_px": crop, "car_bbox_panel_px": car_panel_box,
        "closure_bbox_panel_px": closure_panel_box,
        "output": str(output), "output_sha256": digest(output),
        "annotation_method": "UGV box projects its recorded pose and 4.8 x 2.4 m footprint; closure box is an orange RGB component verified against the recorded camera pose",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    summary_path = run / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if summary["status"] != "completed" or summary["collision_events"]:
        raise RuntimeError("Only a completed, collision-free run may be illustrated")
    captures = {capture["stage"]: capture
                for capture in summary.get("vehicle_captures", [])}
    if set(captures) != set(CROPS):
        raise RuntimeError(f"Missing evidence stages: {set(CROPS) - set(captures)}")
    trajectory_path = run / "trajectories_controls.csv"
    with trajectory_path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["robot"] == "Car1"]
    output_dir = run / "vehicle_evidence" / "derived"
    output_dir.mkdir(exist_ok=True)
    manifest = {
        "run": str(run), "status": summary["status"],
        "car_distance_m": summary["car_distance_m"],
        "summary_sha256": digest(summary_path),
        "trajectory_sha256": digest(trajectory_path),
        "annotation_script_sha256": digest(Path(__file__)),
        "projection_helper_sha256": digest(
            Path(__file__).with_name("annotate_district_navigation.py")),
        "panels": {},
    }
    strips = []
    for stage in CROPS:
        output = output_dir / f"{stage}_marked.png"
        manifest["panels"][stage] = panel(run, captures[stage], rows, output)
        strips.append(Image.open(output))
    montage = Image.new("RGB", (PANEL_SIZE * 3 + 16, PANEL_SIZE), "white")
    for index, image in enumerate(strips):
        montage.paste(image, (index * (PANEL_SIZE + 8), 0))
    figure = output_dir / "ugv_bypass_sequence.png"
    montage.save(figure, optimize=True)
    manifest["figure"] = str(figure)
    manifest["figure_sha256"] = digest(figure)
    (output_dir / "annotation_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(figure)


if __name__ == "__main__":
    main()
