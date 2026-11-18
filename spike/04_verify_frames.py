"""Spike acceptance test: does the recorded data actually match the geometry?

Checks:
  1. Calibration frame: fit H on the 4 calibration markers, evaluate on the 2
     independent check points -> error must be sub-millimetre (perfect render).
  2. Every scan frame: project the known plot centres with the recorded pose and
     confirm the expected class colour is really there.

Run:  .venv\\Scripts\\python.exe spike\\04_verify_frames.py
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).with_name("out") / "run_spike"

MARKER_BGR = {
    "CM1": (255, 0, 255),
    "CM2": (255, 255, 0),
    "CM3": (0, 165, 255),
    "CM4": (255, 0, 128),
    "CP1": (0, 255, 0),
    "CP2": (180, 105, 255),
}


def centroid(frame: np.ndarray, bgr: tuple[int, int, int], tol: int = 40):
    d = np.abs(frame.astype(np.int16) - np.array(bgr, np.int16)).sum(axis=2)
    mask = (d < tol).astype(np.uint8)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return cents[i]


def main() -> None:
    run = json.loads((OUT / "run.json").read_text(encoding="utf-8"))
    calib = run["calibration"]
    hfov = run["camera"]["hfov_deg"]
    w, h = run["camera"]["resolution"]

    # ---------------- 1. calibration frame ----------------
    frame = cv2.imread(str(OUT / "frames" / "calib.png"))
    found = {}
    for name, bgr in MARKER_BGR.items():
        c = centroid(frame, bgr)
        if c is None:
            print(f"  {name}: NOT FOUND")
        else:
            found[name] = c
    print(f"markers detected: {len(found)}/6")

    fit_ids = list(calib["markers"])
    src = np.array([found[i] for i in fit_ids], np.float32)
    dst = np.array([[calib["markers"][i]["x"], calib["markers"][i]["y"]] for i in fit_ids],
                   np.float32)
    Hm = cv2.getPerspectiveTransform(src, dst)

    print("\ncheck points (never used in the fit):")
    worst = 0.0
    for name, p in calib["check_points"].items():
        px = np.array([[[*found[name]]]], np.float32)
        fx, fy = cv2.perspectiveTransform(px, Hm)[0, 0]
        err = math.hypot(fx - p["x"], fy - p["y"]) * 1000
        worst = max(worst, err)
        print(f"  {name}: got=({fx:.4f},{fy:.4f}) want=({p['x']},{p['y']}) err={err:.2f} mm")
    print(f"worst check-point error: {worst:.2f} mm")

    # ---------------- 2. every scan frame ----------------
    f_px = (w / 2) / math.tan(math.radians(hfov / 2))
    gt = json.loads((OUT / "ground_truth.json").read_text(encoding="utf-8"))
    expected_color = {"A": (30, 30, 210), "B": (200, 70, 30), "U": (185, 185, 185)}

    total_hits = total_checks = 0
    print("\nscan frames: predicted plot centre vs actual pixel colour")
    for name in run["frames"]:
        if not name.startswith("f"):
            continue
        meta = json.loads((OUT / "frames" / f"{name}.json").read_text(encoding="utf-8"))
        img = cv2.imread(str(OUT / meta["image"]))
        pose = meta["pose"]
        hits = checks = 0
        for plot in gt["plots"]:
            dx = plot["x"] - pose["x"]
            dy = plot["y"] - pose["y"]
            u = w / 2 + dx / pose["alt"] * f_px
            v = h / 2 - dy / pose["alt"] * f_px
            if not (20 <= u < w - 20 and 20 <= v < h - 20):
                continue
            checks += 1
            want = np.array(expected_color[plot["class"]], np.int16)
            patch = img[int(v) - 12:int(v) + 12, int(u) - 12:int(u) + 12].astype(np.int16)
            dmin = np.abs(patch - want).sum(axis=2).min()
            if dmin < 90:
                hits += 1
        total_hits += hits
        total_checks += checks
        if checks:
            print(f"  {name}: {hits}/{checks} plots where expected")

    print(f"\noverall: {total_hits}/{total_checks} = "
          f"{100 * total_hits / max(total_checks, 1):.1f}% of visible plots matched")


if __name__ == "__main__":
    main()
