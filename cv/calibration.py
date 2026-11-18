"""A1: camera / local-frame calibration.

Pixel -> local navigation frame uses the known nadir camera model.  The
local -> field F similarity is fitted from the four calibration markers and
validated on two independent check points.  A direct pixel -> F homography is
also fitted on the calibration frame and reported as a second, independent
check of the flat-field assumption.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from shared.frames import apply_similarity, similarity_from_points, transform_record
from shared.world import CALIB_MARKERS, CHECK_MARKERS, HFOV, IMG_H, IMG_W


class CalibrationError(RuntimeError):
    pass


# The independent check points tell us how well the flat-field model holds.
# Above this residual the coordinates are not trustworthy enough for a 0.13 m
# recon budget, so the run is refused and reconnaissance must be repeated
# (task C7) instead of acting on a doubtful coordinate.
MAX_CHECK_ERROR_MM = 30.0


def camera_f_px(width: int = IMG_W, hfov_deg: float = HFOV) -> float:
    return (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def pixel_to_local(u: float, v: float, pose: dict,
                   f_px: float | None = None) -> tuple[float, float]:
    """Pixel -> local navigation frame, given the pose in that local frame."""
    f_px = f_px or camera_f_px()
    alt = float(pose["alt"])
    yaw = math.radians(float(pose["yaw"]))
    rx = (u - IMG_W / 2.0) * alt / f_px
    ry = -(v - IMG_H / 2.0) * alt / f_px
    # Invert the camera yaw: the image is rotated by +yaw relative to L.
    dx = rx * math.cos(yaw) + ry * math.sin(yaw)
    dy = -rx * math.sin(yaw) + ry * math.cos(yaw)
    return float(pose["x"]) + dx, float(pose["y"]) + dy


def local_to_pixel(x: float, y: float, pose: dict,
                   f_px: float | None = None) -> tuple[float, float]:
    f_px = f_px or camera_f_px()
    alt = float(pose["alt"])
    yaw = math.radians(float(pose["yaw"]))
    dx, dy = x - float(pose["x"]), y - float(pose["y"])
    rx = dx * math.cos(yaw) - dy * math.sin(yaw)
    ry = dx * math.sin(yaw) + dy * math.cos(yaw)
    return IMG_W / 2.0 + rx / alt * f_px, IMG_H / 2.0 - ry / alt * f_px


def _bgr(rgb: tuple[int, int, int]) -> tuple[int, int, int]:
    return int(rgb[2]), int(rgb[1]), int(rgb[0])


def centroid(frame: np.ndarray, bgr: tuple[int, int, int], tol: int = 45):
    d = np.abs(frame.astype(np.int16) - np.array(bgr, np.int16)).sum(axis=2)
    mask = (d < tol).astype(np.uint8)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[i, cv2.CC_STAT_AREA] < 30:
        return None
    return float(cents[i][0]), float(cents[i][1])


def detect_markers(frame: np.ndarray) -> dict[str, tuple[float, float]]:
    found = {}
    for name, (_, _, rgb) in {**CALIB_MARKERS, **CHECK_MARKERS}.items():
        c = centroid(frame, _bgr(rgb))
        if c is not None:
            found[name] = c
    return found


@dataclass
class Calibration:
    f_px: float
    transform: tuple[float, float, float, float]   # L -> F similarity
    homography: np.ndarray | None                  # pixel -> F on calib frame
    check_errors_mm: dict[str, float]
    fit_errors_mm: dict[str, float]
    markers: dict[str, tuple[float, float]]
    check_points: dict[str, tuple[float, float]]
    worst_check_mm: float

    def as_dict(self) -> dict:
        return {
            "f_px": self.f_px,
            "local_to_field": transform_record(self.transform),
            "homography_pixel_to_field": (None if self.homography is None
                                          else self.homography.tolist()),
            "fit_errors_mm": self.fit_errors_mm,
            "check_errors_mm": self.check_errors_mm,
            "worst_check_mm": self.worst_check_mm,
            "markers_px": {k: list(v) for k, v in self.markers.items()},
            "check_points_px": {k: list(v) for k, v in self.check_points.items()},
        }


def calibrate(frame: np.ndarray, pose_local: dict) -> Calibration:
    """Fit L -> F and pixel -> F from the calibration frame.

    ``pose_local`` is the camera pose in the local navigation frame.
    """
    f_px = camera_f_px()
    found = detect_markers(frame)
    missing = [k for k in CALIB_MARKERS if k not in found]
    if missing:
        raise CalibrationError(f"calibration markers not found: {missing}")

    src_local: list[tuple[float, float]] = []
    dst_field: list[tuple[float, float]] = []
    fit_errors: dict[str, float] = {}
    for name, (x, y, _rgb) in CALIB_MARKERS.items():
        u, v = found[name]
        lx, ly = pixel_to_local(u, v, pose_local, f_px)
        src_local.append((lx, ly))
        dst_field.append((x, y))
    transform = similarity_from_points(src_local, dst_field)
    for name, (x, y, _rgb) in CALIB_MARKERS.items():
        lx, ly = pixel_to_local(*found[name], pose_local, f_px)
        fx, fy = apply_similarity(transform, lx, ly)
        fit_errors[name] = math.hypot(fx - x, fy - y) * 1000.0

    check_errors: dict[str, float] = {}
    check_pts: dict[str, tuple[float, float]] = {}
    for name, (x, y, _rgb) in CHECK_MARKERS.items():
        if name not in found:
            check_errors[name] = float("inf")
            continue
        check_pts[name] = found[name]
        lx, ly = pixel_to_local(*found[name], pose_local, f_px)
        fx, fy = apply_similarity(transform, lx, ly)
        check_errors[name] = math.hypot(fx - x, fy - y) * 1000.0

    homography = None
    try:
        src_px = np.array([found[n] for n in CALIB_MARKERS], np.float32)
        dst = np.array([[CALIB_MARKERS[n][0], CALIB_MARKERS[n][1]]
                        for n in CALIB_MARKERS], np.float32)
        homography = cv2.getPerspectiveTransform(src_px, dst)
    except cv2.error:
        homography = None

    worst = max(check_errors.values()) if check_errors else float("inf")
    if worst > MAX_CHECK_ERROR_MM:
        raise CalibrationError(
            f"контрольные точки вне допуска: худшая {worst:.1f} мм "
            f"при пределе {MAX_CHECK_ERROR_MM:.0f} мм; нужна повторная разведка")
    return Calibration(f_px=f_px, transform=transform, homography=homography,
                       check_errors_mm=check_errors, fit_errors_mm=fit_errors,
                       markers=found, check_points=check_pts,
                       worst_check_mm=worst)
