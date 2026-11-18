"""A2: detect the 10 plot cards and classify A/B/U.

The simulated plots are deliberately simple: a white/grey card with a black
border and a coloured shape.  Detection therefore uses classic CV only:

* bright low-saturation card mask;
* perspective warp of each candidate card;
* colour + shape score inside the warped card.

The returned score is a heuristic confidence, not a probability.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from shared.world import IMG_H, IMG_W

CARD_SIZE = 128
MIN_CARD_AREA = 500
MAX_CARD_AREA = 80000


@dataclass
class Detection:
    klass: str
    u: float
    v: float
    score: float
    color_fraction: float
    shape_score: float
    frame_id: str = ""
    details: dict = field(default_factory=dict)
    x: float | None = None     # field F, filled by the pipeline
    y: float | None = None


def _order_points(pts: np.ndarray) -> np.ndarray:
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).reshape(-1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(d)]
    rect[3] = pts[np.argmax(d)]
    return rect


def _warp_card(frame: np.ndarray, contour: np.ndarray) -> np.ndarray | None:
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(np.float32)
    src = _order_points(box)
    dst = np.array([[0, 0], [CARD_SIZE - 1, 0],
                    [CARD_SIZE - 1, CARD_SIZE - 1], [0, CARD_SIZE - 1]], np.float32)
    m = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, m, (CARD_SIZE, CARD_SIZE))


def _shape_stats(mask: np.ndarray) -> tuple[float, float, float]:
    """Return (fill_fraction, circularity, triangle_score) for a binary mask."""
    area = float(np.count_nonzero(mask))
    if area == 0:
        return 0.0, 0.0, 0.0
    fill = area / float(mask.size)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return fill, 0.0, 0.0
    c = max(contours, key=cv2.contourArea)
    area_c = cv2.contourArea(c)
    per = cv2.arcLength(c, True)
    circularity = 0.0 if per <= 0 else float(4.0 * np.pi * area_c / (per * per))
    approx = cv2.approxPolyDP(c, 0.04 * per, True)
    # A solid triangle has 3 vertices and fills roughly half of its bounding box.
    triangle = 1.0 if len(approx) == 3 else 0.0
    return fill, circularity, triangle


def classify_card(card: np.ndarray) -> tuple[str | None, float, float, float, dict]:
    """Return (class, score, colour_fraction, shape_score, details)."""
    hsv = cv2.cvtColor(card, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    # Ignore the outer border: the shape never reaches it.
    inner = np.zeros((CARD_SIZE, CARD_SIZE), np.uint8)
    inner[16:CARD_SIZE - 16, 16:CARD_SIZE - 16] = 1
    inner_bool = inner.astype(bool)

    red = (((h < 10) | (h > 170)) & (s > 80) & (v > 55) & inner_bool)
    blue = ((h > 95) & (h < 135) & (s > 80) & (v > 45) & inner_bool)
    grey = ((s < 45) & (v > 120) & (v < 225) & inner_bool)
    white = ((s < 35) & (v >= 225) & inner_bool)
    dark = ((v < 95) & inner_bool)

    red_frac = float(np.count_nonzero(red)) / float(np.count_nonzero(inner_bool))
    blue_frac = float(np.count_nonzero(blue)) / float(np.count_nonzero(inner_bool))
    grey_frac = float(np.count_nonzero(grey)) / float(np.count_nonzero(inner_bool))
    white_frac = float(np.count_nonzero(white)) / float(np.count_nonzero(inner_bool))
    dark_frac = float(np.count_nonzero(dark)) / float(np.count_nonzero(inner_bool))

    red_fill, red_circ, red_tri = _shape_stats(red.astype(np.uint8))
    blue_fill, blue_circ, _ = _shape_stats(blue.astype(np.uint8))

    details = {
        "red_frac": round(red_frac, 4),
        "blue_frac": round(blue_frac, 4),
        "grey_frac": round(grey_frac, 4),
        "white_frac": round(white_frac, 4),
        "dark_frac": round(dark_frac, 4),
        "red_triangle": red_tri,
        "red_circularity": round(red_circ, 3),
        "blue_circularity": round(blue_circ, 3),
    }

    # A: red triangle.  The triangle fills ~0.45 of the inner square.
    if red_frac > 0.05:
        shape = 0.55 + 0.45 * red_tri
        score = red_frac * shape
        return "A", float(score), red_frac, red_tri, details

    # B: blue circle.  Circularity near 1 for a filled disc.
    if blue_frac > 0.05:
        shape = 0.55 + 0.45 * min(1.0, blue_circ / 0.85)
        score = blue_frac * shape
        return "B", float(score), blue_frac, blue_circ, details

    # U: grey card with a dark question mark, no strong colour.
    if grey_frac > 0.18 and red_frac < 0.03 and blue_frac < 0.03:
        shape = 0.6 + 0.4 * min(1.0, dark_frac / 0.06)
        score = grey_frac * shape
        return "U", float(score), grey_frac, shape, details

    return None, 0.0, max(red_frac, blue_frac, grey_frac), 0.0, details


def detect_plots(frame: np.ndarray, frame_id: str = "") -> list[Detection]:
    """Find plot cards in one 640x480 nadir frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    # White cards and the grey U card.  Brown soil and coloured markers fail.
    mask = (((v > 140) & (s < 105)).astype(np.uint8) * 255)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out: list[Detection] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CARD_AREA or area > MAX_CARD_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w < 18 or h < 18:
            continue
        # A card cut by the frame edge has a biased centroid; skip it and let
        # another frame provide a complete observation.
        if x <= 2 or y <= 2 or x + w >= IMG_W - 2 or y + h >= IMG_H - 2:
            continue
        ratio = w / float(h)
        if ratio < 0.35 or ratio > 2.8:
            continue
        card = _warp_card(frame, c)
        if card is None:
            continue
        klass, score, color_frac, shape_score, details = classify_card(card)
        if klass is None or score < 0.055:
            continue
        mm = cv2.moments(c)
        if mm["m00"] == 0:
            continue
        u = float(mm["m10"] / mm["m00"])
        vv = float(mm["m01"] / mm["m00"])
        if not (2 <= u < IMG_W - 2 and 2 <= vv < IMG_H - 2):
            continue
        out.append(Detection(klass=klass, u=u, v=vv, score=float(score),
                             color_fraction=float(color_frac),
                             shape_score=float(shape_score),
                             frame_id=frame_id, details=details))
    # One card can produce a couple of contours at the frame edge.
    out.sort(key=lambda d: d.score, reverse=True)
    unique: list[Detection] = []
    for d in out:
        if all((d.u - o.u) ** 2 + (d.v - o.v) ** 2 > 18 ** 2 for o in unique):
            unique.append(d)
    return unique


def annotate(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    img = frame.copy()
    colors = {"A": (40, 40, 220), "B": (220, 120, 30), "U": (160, 160, 160)}
    for d in detections:
        c = (int(round(d.u)), int(round(d.v)))
        cv2.circle(img, c, 9, colors.get(d.klass, (255, 255, 255)), 2)
        cv2.putText(img, f"{d.klass} {d.score:.2f}", (c[0] + 11, c[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"{d.klass} {d.score:.2f}", (c[0] + 11, c[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 1, cv2.LINE_AA)
    return img
