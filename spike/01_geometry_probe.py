"""Probe 2: measure the exact pixel<->field mapping of the virtual down camera.

Four markers fit the homography; two independent check points validate it.
Markers are solid colours so this probe tests *geometry only* (real texture
recognition is a separate task).

Run:  .venv\\Scripts\\python.exe spike\\01_geometry_probe.py
"""
from pathlib import Path

import cv2
import numpy as np
from panda3d.core import Filename, PNMImage
from ursina import Entity, Ursina, camera, color, scene, window

OUT = Path(__file__).with_name("out")
OUT.mkdir(exist_ok=True)

W, H = 640, 480
ALT = 1.2
FOV = 70.0

# field coords: x along the 6 m side, y along the 4 m side (metres)
CALIB = {  # used to fit H
    "red": (2.60, 1.60),
    "green": (3.40, 1.60),
    "blue": (3.40, 2.40),
    "yellow": (2.60, 2.40),
}
CHECK = {  # independent points, never used in the fit
    "magenta": (3.00, 2.00),
    "cyan": (2.80, 2.20),
}
MARKER = 0.12  # square side, metres

app = Ursina(
    title="geometry probe",
    size=(W, H),
    development_mode=False,
    editor_ui_enabled=False,
    show_ursina_splash=False,
)
window.color = color.black

# Ursina space: X right, Y up, Z forward -> the field is the XZ plane.
Entity(
    model="quad",
    texture="white_cube",
    color=color.rgb(90, 90, 90),
    scale=(6, 4),
    position=(3, 0, 2),
    rotation_x=90,
    double_sided=True,
)

for name, (fx, fy) in {**CALIB, **CHECK}.items():
    Entity(
        model="quad",
        color=getattr(color, name),
        scale=MARKER,
        position=(fx, 0.01, fy),
        rotation_x=90,
        double_sided=True,
    )

camera.position = (3.0, ALT, 2.0)
camera.rotation = (90, 0, 0)
camera.fov = FOV
camera.clip_plane_far = 100

for _ in range(4):
    app.step()

path = OUT / "geometry.png"
img = PNMImage()
base.win.getScreenshot(img)
img.write(Filename.fromOsSpecific(str(path)))
print("saved:", path)

# ---- analysis ----
frame = cv2.imread(str(path))
print("frame shape:", frame.shape)

HEADLESS_BGR = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "magenta": (255, 0, 255),
    "cyan": (255, 255, 0),
}


def centroid(bgr):
    target = np.array(bgr, np.int16)
    dist = np.abs(frame.astype(np.int16) - target).sum(axis=2)
    mask = (dist < 60).astype(np.uint8) * 255
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return cents[i], int(stats[i, cv2.CC_STAT_AREA])


found = {}
for name, bgr in HEADLESS_BGR.items():
    res = centroid(bgr)
    if res is None:
        print(f"{name:8s} NOT FOUND")
        continue
    (cx, cy), area = res
    found[name] = (cx, cy)
    print(f"{name:8s} pixel=({cx:7.2f},{cy:7.2f}) area={area:6d}px")

names = list(CALIB)
src = np.array([found[n] for n in names], np.float32)
dst = np.array([CALIB[n] for n in names], np.float32)
Hm = cv2.getPerspectiveTransform(src, dst)

print("\n--- pixel -> field (metres) ---")
print("check points (not used in the fit):")
for n, (ex, ey) in CHECK.items():
    px = np.array([[[*found[n]]]], np.float32)
    fx, fy = cv2.perspectiveTransform(px, Hm)[0, 0]
    print(f"  {n:8s} got=({fx:6.4f},{fy:6.4f}) expected=({ex:6.4f},{ey:6.4f}) "
          f"err={np.hypot(fx - ex, fy - ey) * 1000:5.2f} mm")

print("\ncalibration points (fit residuals):")
for n in names:
    px = np.array([[[*found[n]]]], np.float32)
    fx, fy = cv2.perspectiveTransform(px, Hm)[0, 0]
    ex, ey = CALIB[n]
    print(f"  {n:8s} err={np.hypot(fx - ex, fy - ey) * 1000:5.2f} mm")

mpp = 2 * ALT * np.tan(np.radians(FOV / 2)) / W
print(f"\nmodel: alt={ALT} m, hfov={FOV} deg, {W}x{H} px")
print(f"expected footprint {2*ALT*np.tan(np.radians(FOV/2)):.3f} m wide, {mpp*1000:.2f} mm/px")
print("H (pixel -> field):\n", np.array2string(Hm, precision=6))

base.userExit()
