"""Annotate a recorded district-navigation attempt without altering raw frames."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


UGV_COLOR = "#e34a3b"
UAV_COLOR = "#1dc6d3"
CLOSURE_COLOR = "#f2b637"
INK = "#14252a"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def font(size: int):
    from ag.fonts import font as portable_font
    return portable_font(size, bold=True)


def label(draw, text, anchor, color, image_size, offset=(12, -33)):
    x = int(anchor[0] + offset[0])
    y = int(anchor[1] + offset[1])
    face = font(20)
    box = draw.textbbox((0, 0), text, font=face)
    width, height = box[2] + 18, box[3] - box[1] + 12
    x = max(5, min(x, image_size[0] - width - 5))
    y = max(5, min(y, image_size[1] - height - 5))
    draw.line((anchor[0], anchor[1], x + width / 2, y + height / 2),
              fill="white", width=4)
    draw.line((anchor[0], anchor[1], x + width / 2, y + height / 2),
              fill=color, width=2)
    draw.rectangle((x, y, x + width, y + height), fill=INK,
                   outline=color, width=2)
    draw.text((x + 9, y + 5 - box[1]), text, font=face, fill="white")


def camera_frame(run: Path, index: int, output: Path):
    source = run / f"closure_{index}.png"
    frame = cv2.imread(str(source))
    if frame is None:
        raise FileNotFoundError(source)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # The inserted barrier uses saturated orange. This locates the visible
    # object in this RGB frame; the earlier segmentation packet is not synced.
    orange = cv2.inRange(hsv, (8, 140, 150), (25, 255, 255))
    _, _, stats, _ = cv2.connectedComponentsWithStats(orange)
    candidates = [tuple(map(int, row)) for row in stats[1:]
                  if row[4] > 800 and 25 <= row[2] <= 130 and
                  20 <= row[3] <= 110 and row[2] / row[3] < 3]
    candidates.sort(key=lambda row: row[4], reverse=True)
    if not candidates or candidates[0][4] < 2000 or (
        len(candidates) > 1 and candidates[0][4] < 2 * candidates[1][4]
    ):
        raise RuntimeError(f"Cannot isolate the visible orange closure in "
                           f"{source}: {candidates}")
    x, y, w, h, area = candidates[0]
    image = Image.open(source).convert("RGB")
    crop_w, crop_h = 480, 288
    left = max(0, min(image.width - crop_w, x + w // 2 - crop_w // 2))
    top = max(0, min(image.height - crop_h, y + h // 2 - crop_h // 2))
    image = image.crop((left, top, left + crop_w, top + crop_h))
    image = image.resize((600, 360), Image.Resampling.LANCZOS)
    rect = tuple(round(value) for value in (
        (x - left - 6) * 600 / crop_w, (y - top - 6) * 360 / crop_h,
        (x + w - left + 6) * 600 / crop_w,
        (y + h - top + 6) * 360 / crop_h))
    draw = ImageDraw.Draw(image)
    draw.rectangle(rect, outline="white", width=7)
    draw.rectangle(rect, outline=CLOSURE_COLOR, width=4)
    label(draw, f"Road closure {index}", (rect[0], rect[1]),
          CLOSURE_COLOR, image.size, offset=(0, -42))
    image.save(output, optimize=True)
    return {
        "source": str(source), "source_sha256": digest(source),
        "method": "saturated-orange connected component in the displayed RGB frame",
        "hsv_opencv": {"lower": [8, 140, 150], "upper": [25, 255, 255]},
        "rgb_bbox_px": [x, y, x + w, y + h], "mask_area_px": area,
        "crop_box_px": [left, top, left + crop_w, top + crop_h],
        "output": str(output), "output_sha256": digest(output),
    }


def camera_projector(packet: Path, image_size):
    metadata = json.loads(str(np.load(packet)["metadata_json"]))
    position = np.array([metadata[f"pos_{axis}"] for axis in "xyz"])
    w, x, y, z = [metadata[f"rot_{axis}"] for axis in "wxyz"]
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
    ])
    focal = image_size[0] / (2 * np.tan(np.radians(80 / 2)))

    def project(world):
        points = np.asarray(world, dtype=float).reshape(-1, 3)
        local = (points - position) @ rotation
        if np.any(local[:, 0] <= 1):
            raise RuntimeError("Requested marker is behind the overview camera")
        pixels = np.column_stack((image_size[0] / 2 + focal * local[:, 1] / local[:, 0],
                                  image_size[1] / 2 + focal * local[:, 2] / local[:, 0]))
        return pixels

    return project, metadata


def overview(run: Path, summary, output: Path):
    source = run / "junction_2.png"
    packet = run / "junction_2.npz"
    image = Image.open(source).convert("RGB")
    project, metadata = camera_projector(packet, image.size)
    camera_time = metadata["time_stamp"] / 1e9
    with (run / "trajectories_controls.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    draw = ImageDraw.Draw(image)
    markers = {}
    for actor, color in (("Car1", UGV_COLOR), ("Drone1", UAV_COLOR)):
        samples = [row for row in rows if row["robot"] == actor and
                   float(row["sim_time_sec"]) <= camera_time + .15]
        if len(samples) < 2:
            raise RuntimeError(f"Missing measured path for {actor}")
        world = np.array([[float(row[axis]) for axis in "xyz"] for row in samples])
        pixel = project(world)
        points = [tuple(map(float, point)) for point in pixel]
        draw.line(points, fill="white", width=4, joint="curve")
        draw.line(points, fill=color, width=2, joint="curve")
        endpoint = pixel[-1]
        cx, cy = map(int, endpoint)
        if actor == "Car1":
            draw.ellipse((cx - 8, cy - 8, cx + 8, cy + 8),
                         fill=color, outline="white", width=3)
            offset = (18, -56)
            visible_name = "UGV"
        else:
            draw.polygon([(cx, cy - 10), (cx + 10, cy),
                          (cx, cy + 10), (cx - 10, cy)],
                         fill=color, outline="white", width=3)
            offset = (14, -42)
            visible_name = "UAV"
        label(draw, visible_name, endpoint, color, image.size, offset=offset)
        markers[visible_name] = {
            "sample_time_sec": float(samples[-1]["sim_time_sec"]),
            "world_ned": world[-1].tolist(), "pixel": endpoint.tolist(),
            "path_samples": len(samples),
        }
    scenario = json.loads((run / "scenario.json").read_text(encoding="utf-8"))
    for index, closure in enumerate(scenario["evaluation_closures"], 1):
        pixel = project([closure["center_ned"]])[0]
        cx, cy = map(int, pixel)
        draw.rectangle((cx - 11, cy - 11, cx + 11, cy + 11),
                       outline="white", width=7)
        draw.rectangle((cx - 11, cy - 11, cx + 11, cy + 11),
                       outline=CLOSURE_COLOR, width=4)
        offset = (-120, -47) if index == 1 else (16, 11)
        label(draw, f"Closure {index}", pixel, CLOSURE_COLOR,
              image.size, offset=offset)
        markers[f"Closure {index}"] = {
            "source": "configured evaluation object pose",
            "world_ned": closure["center_ned"], "pixel": pixel.tolist(),
        }
    if summary["status"] != "completed":
        status_text = "Diagnostic run: UGV off route"
        face = font(17)
        bounds = draw.textbbox((0, 0), status_text, font=face)
        draw.rectangle((12, image.height - 39, bounds[2] + 28,
                        image.height - 9), fill=INK,
                       outline=UGV_COLOR, width=2)
        draw.text((20, image.height - 34 - bounds[1]), status_text,
                  font=face, fill="white")
    image.save(output, optimize=True)
    return {
        "source": str(source), "source_sha256": digest(source),
        "camera_packet": str(packet), "camera_packet_sha256": digest(packet),
        "camera_time_sec": camera_time,
        "trajectory": str(run / "trajectories_controls.csv"),
        "trajectory_sha256": digest(run / "trajectories_controls.csv"),
        "scenario": str(run / "scenario.json"),
        "scenario_sha256": digest(run / "scenario.json"),
        "markers": markers, "output": str(output),
        "output_sha256": digest(output),
        "note": "UGV/UAV markers and paths are projected from saved poses; closure markers are configured object poses. Markers are postprocessing, not visible airframes or RGB detections.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    if len(summary.get("decisions", [])) != 2:
        raise RuntimeError("This annotation requires two recorded decisions")
    output = run / "derived"
    output.mkdir(exist_ok=True)
    manifest = {
        "run": str(run), "run_status": summary["status"],
        "summary_sha256": digest(run / "summary.json"),
        "completed_experiment": summary["status"] == "completed",
        "panels": {},
    }
    for index in (1, 2):
        manifest["panels"][f"closure_{index}"] = camera_frame(
            run, index, output / f"closure_{index}_marked.png")
    manifest["panels"]["overview"] = overview(
        run, summary, output / "district_overview_marked.png")
    canvas = Image.new("RGB", (1884, 720), "white")
    canvas.paste(Image.open(output / "district_overview_marked.png"), (0, 0))
    for index in (1, 2):
        canvas.paste(Image.open(output / f"closure_{index}_marked.png"),
                     (1282, (index - 1) * 360))
    canvas.save(output / "district_navigation_marked.png", optimize=True)
    manifest["combined"] = {
        "output": str(output / "district_navigation_marked.png"),
        "output_sha256": digest(output / "district_navigation_marked.png"),
        "meaning": "Diagnostic two-decision run; UGV did not reach its goal",
    }
    (output / "annotation_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
