"""Post-process the recorded run: contact sheet + annotated field overview.

Pure OpenCV/JSON - no 3D, no simulator dependency. This is how the pipeline
team consumes the frames.

Run:  .venv\\Scripts\\python.exe spike\\05_poster.py
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).with_name("out") / "run_spike"
FONT = cv2.FONT_HERSHEY_SIMPLEX


def load(name: str) -> np.ndarray:
    return cv2.imread(str(OUT / "frames" / f"{name}.png"))


def world_to_pixel(x, y, pose, hfov, w, h):
    f_px = (w / 2) / math.tan(math.radians(hfov / 2))
    yaw = pose["yaw"]
    dx, dy = x - pose["x"], y - pose["y"]
    # camera looks along +y; yaw>0 rotates the image clockwise
    rx = dx * math.cos(yaw) + dy * math.sin(yaw)
    ry = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return w / 2 + rx / pose["alt"] * f_px, h / 2 - ry / pose["alt"] * f_px


def contact_sheet(run: dict) -> Path:
    names = [n for n in run["frames"] if n.startswith("f")]
    tw, th = 320, 240
    cols, rows = 6, math.ceil(len(names) / 6)
    sheet = np.full((rows * (th + 22), cols * tw, 3), 30, np.uint8)
    for i, name in enumerate(names):
        img = cv2.resize(load(name), (tw, th), interpolation=cv2.INTER_AREA)
        x, y = (i % cols) * tw, (i // cols) * (th + 22)
        sheet[y:y + th, x:x + tw] = img
        cv2.putText(sheet, name, (x + 6, y + th + 17), FONT, 0.55, (230, 230, 230), 1,
                    cv2.LINE_AA)
    for c in range(cols + 1):
        cv2.line(sheet, (c * tw, 0), (c * tw, sheet.shape[0]), (60, 60, 60), 1)
    path = OUT / "contact_sheet.png"
    cv2.imwrite(str(path), sheet)
    return path


def annotated_overview(run: dict, gt: dict) -> Path:
    meta = json.loads((OUT / "frames" / "overview.json").read_text(encoding="utf-8"))
    img = load("overview").copy()
    pose = meta["pose"]
    hfov = run["camera"]["hfov_deg"]
    h, w = img.shape[:2]

    def px(p):
        return tuple(int(round(v)) for v in world_to_pixel(p["x"], p["y"], pose, hfov, w, h))

    zone = np.array([px(p) for p in gt["no_fly"]], np.int32)
    overlay = img.copy()
    cv2.fillPoly(overlay, [zone], (60, 60, 200))
    img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)
    cv2.polylines(img, [zone], True, (40, 40, 220), 2)

    colours = {"A": (40, 40, 220), "B": (200, 90, 30), "U": (160, 160, 160)}
    for p in gt["plots"]:
        c = px(p)
        inside = cv2.pointPolygonTest(zone, (float(c[0]), float(c[1])), False) >= 0
        cv2.circle(img, c, 12, colours[p["class"]], 2)
        label = f"{p['id']} {p['class']}{' (no-fly)' if inside else ''}"
        cv2.putText(img, label, (c[0] + 14, c[1] + 4), FONT, 0.42, (255, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(img, label, (c[0] + 14, c[1] + 4), FONT, 0.42, (20, 20, 20), 1,
                    cv2.LINE_AA)

    cv2.putText(img, "field F: 6 x 4 m, x right, y up", (10, 22), FONT, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    path = OUT / "overview_annotated.png"
    cv2.imwrite(str(path), img)
    return path


def main() -> None:
    run = json.loads((OUT / "run.json").read_text(encoding="utf-8"))
    gt = json.loads((OUT / "ground_truth.json").read_text(encoding="utf-8"))
    print("contact sheet:", contact_sheet(run))
    print("overview     :", annotated_overview(run, gt))


if __name__ == "__main__":
    main()
