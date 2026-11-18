"""Field definition shared by the demo and the frame recorder.

Single source of truth for coordinates, classes, no-fly zone and plot textures.
Frame F: origin at the field corner, x along the 6 m side, y along the 4 m side.
Ursina world: X right, Y up, Z forward -> field (x, y) maps to world (x, y, height).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FIELD_W, FIELD_H = 6.0, 4.0
PLOT_SIZE = 0.5
MARKER_SIZE = 0.12
TEX_SIZE = 256

ALT_NOMINAL = 1.2
ALT_CORRIDOR = (1.0, 1.4)
HFOV = 70.0
DRONE_RADIUS = 0.10
SPEED_LIMIT = 0.4

# (id, class, centre) - 6 available A, 2 A inside the no-fly zone, 1 B, 1 U
PLOTS: list[tuple[str, str, tuple[float, float]]] = [
    ("P01", "A", (2.10, 0.70)),
    ("P02", "A", (3.00, 1.40)),
    ("P03", "A", (1.60, 2.30)),
    ("P04", "A", (2.60, 3.20)),
    ("P05", "A", (4.00, 0.70)),
    ("P06", "A", (4.20, 2.20)),
    ("P07", "A", (5.10, 2.10)),
    ("P08", "A", (5.30, 3.10)),
    ("P09", "B", (3.60, 2.40)),
    ("P10", "U", (3.20, 3.50)),
]

NO_FLY: list[tuple[float, float]] = [
    (4.60, 1.50), (5.90, 1.20), (5.90, 3.80), (4.60, 3.40),
]

CALIB_MARKERS = {
    "CM1": (0.40, 0.40, (255, 0, 255)),
    "CM2": (1.20, 0.40, (0, 255, 255)),
    "CM3": (1.20, 1.20, (255, 165, 0)),
    "CM4": (0.40, 1.20, (128, 0, 255)),
}
CHECK_MARKERS = {
    "CP1": (0.80, 0.60, (0, 255, 0)),
    "CP2": (0.60, 1.00, (255, 105, 180)),
}


def project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def textures_dir() -> Path:
    path = project_root() / "data" / "textures"
    path.mkdir(parents=True, exist_ok=True)
    return path


def point_in_poly(px: float, py: float, poly: list[tuple[float, float]]) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def validate_layout() -> dict[str, int]:
    """Fail loudly if the field stops matching the task; returns the counts."""
    a_total = sum(1 for _, c, _ in PLOTS if c == "A")
    a_free = sum(1 for _, c, (x, y) in PLOTS if c == "A" and not point_in_poly(x, y, NO_FLY))
    a_zone = sum(1 for _, c, (x, y) in PLOTS if c == "A" and point_in_poly(x, y, NO_FLY))
    if (len(PLOTS), a_total, a_free, a_zone) != (10, 8, 6, 2):
        raise ValueError(
            f"field layout broken: plots={len(PLOTS)} A={a_total} "
            f"allowed={a_free} in_no_fly={a_zone}, expected 10/8/6/2"
        )
    for i in range(len(PLOTS)):
        for j in range(i + 1, len(PLOTS)):
            (xi, yi), (xj, yj) = PLOTS[i][2], PLOTS[j][2]
            if math.dist((xi, yi), (xj, yj)) < PLOT_SIZE:
                raise ValueError(f"plots {PLOTS[i][0]} and {PLOTS[j][0]} overlap")
    return {"plots": len(PLOTS), "a_total": a_total,
            "a_free": a_free, "a_zone": a_zone}


def make_plot_texture(cls: str, path: Path) -> Path:
    """One plot. Class is encoded by both colour and shape on purpose."""
    if path.exists():
        return path
    s = TEX_SIZE
    img = Image.new("RGB", (s, s), (250, 248, 240))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, s - 1, s - 1], outline=(20, 20, 20), width=8)
    pad = 34
    if cls == "A":
        d.polygon([(s / 2, pad), (s - pad, s - pad), (pad, s - pad)], fill=(210, 30, 30))
    elif cls == "B":
        d.ellipse([pad, pad, s - pad, s - pad], fill=(30, 70, 200))
    elif cls == "U":
        img = Image.new("RGB", (s, s), (185, 185, 185))
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, s - 1, s - 1], outline=(90, 90, 90), width=8)
        try:
            font = ImageFont.truetype("arialbd.ttf", 150)
        except OSError:
            font = ImageFont.load_default()
        d.text((s / 2, s / 2), "?", font=font, fill=(240, 240, 240),
               anchor="mm", stroke_width=6, stroke_fill=(60, 60, 60))
    else:
        raise ValueError(f"unknown plot class {cls!r}")
    img.save(path)
    return path


def plot_texture_paths(tex_dir: Path | None = None) -> dict[str, Path]:
    tex_dir = tex_dir or textures_dir()
    return {
        pid: make_plot_texture(cls, tex_dir / f"{pid}_{cls}.png")
        for pid, cls, _ in PLOTS
    }


def class_texture_paths(tex_dir: Path | None = None) -> dict[str, Path]:
    """One texture per class, reused by every plot of an editable layout."""
    tex_dir = tex_dir or textures_dir()
    return {
        cls: make_plot_texture(cls, tex_dir / f"class_{cls}.png")
        for cls in ("A", "B", "U")
    }


def route_number_texture(number: int, tex_dir: Path | None = None) -> Path:
    """Round badge with the visit order, drawn once per number."""
    tex_dir = tex_dir or textures_dir()
    path = tex_dir / f"route_{number:02d}.png"
    if path.exists():
        return path
    s = 128
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.ellipse([4, 4, s - 4, s - 4], fill=(10, 20, 34, 232),
              outline=(80, 230, 255, 255), width=7)
    try:
        font = ImageFont.truetype("arialbd.ttf", 72)
    except OSError:
        font = ImageFont.load_default()
    d.text((s / 2, s / 2 - 3), str(number), font=font,
           fill=(255, 255, 255, 255), anchor="mm")
    img.save(path)
    return path


@dataclass
class Pose:
    x: float
    y: float
    alt: float
    yaw: float = 0.0  # radians, 0 = looking along +y


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
