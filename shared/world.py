"""Shared world model: geometry, limits and task constants.

This module is the only source of truth for the CV/planner/executor side.
It deliberately does NOT contain the ground-truth plot layout: the pipeline
must never see where the plots really are.  The simulator builds its scene
from ``sim/field.py``; a consistency check in ``sim/layout.py`` keeps the two
copies of the field parameters honest.
"""
from __future__ import annotations

import math

# --------------------------------------------------------------------------- #
# Field / camera (task AT11-2, table 2)
# --------------------------------------------------------------------------- #
FIELD_W, FIELD_H = 6.0, 4.0
PLOT_SIZE = 0.5

IMG_W, IMG_H = 640, 480
HFOV = 70.0

ALT_NOMINAL = 1.2
ALT_MIN, ALT_MAX = 1.0, 1.4
SPEED_LIMIT = 0.4

# Default for free flight, legacy missions and the executor's base layout.
HOME = (3.0, 2.0)
FLIGHT_FIELD_MARGIN = 0.15

# Forbidden zone, metres in F (same polygon as sim/field.py).
NO_FLY: list[tuple[float, float]] = [
    (4.60, 1.50), (5.90, 1.20), (5.90, 3.80), (4.60, 3.40),
]

# Calibration markers and independent check points, metres in F.
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
MARKER_SIZE = 0.12

# --------------------------------------------------------------------------- #
# Task accuracy / safety budget (task 1.3 and 1.4)
# --------------------------------------------------------------------------- #
TARGET_RADIUS = 0.25          # R_target, operation circle
IMPACT_RADIUS = 0.04          # r_impact, geometric check only
DRONE_RADIUS = 0.10
SAFETY_MARGIN = 0.05
EXECUTOR_ERROR = 0.06
ALIGNMENT_ERROR = 0.02
RECON_ERROR = 0.07            # planning upper estimate from the task

# 0.10 + 0.05 + 0.06 + 0.02 = 0.23 (task 1.4)
FORBIDDEN_EXPANSION = DRONE_RADIUS + SAFETY_MARGIN + EXECUTOR_ERROR + ALIGNMENT_ERROR
# The error budget caps the recon error at 0.25 - 0.06 - 0.02 - 0.04 = 0.13.
MAX_RECON_ERROR = TARGET_RADIUS - EXECUTOR_ERROR - ALIGNMENT_ERROR - IMPACT_RADIUS

STABILIZE_SECONDS = 1.0
OPERATION_SECONDS = 2.0
POSE_FRESH_SECONDS = 0.3

MAP_TTL_SECONDS = 15 * 60
RECON_LIMIT_SECONDS = 6 * 60
PROCESS_LIMIT_SECONDS = 3 * 60
EXEC_LIMIT_SECONDS = 6 * 60

# Legacy navigation frames for standalone runs. The app places each origin
# at the corresponding take-off point and retains the independent headings.
LOCAL_FRAMES = {
    "L_recon": {"origin": (0.20, 0.30), "yaw_deg": 7.0},
    "L_exec": {"origin": (5.55, 3.70), "yaw_deg": -160.0},
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


# --------------------------------------------------------------------------- #
# 2-D geometry
# --------------------------------------------------------------------------- #
def point_in_poly(px: float, py: float, poly: list[tuple[float, float]] | tuple) -> bool:
    """Ray casting; same convention as sim/field.py."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def point_segment_distance(px: float, py: float,
                           ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def distance_to_polygon(px: float, py: float,
                        poly: list[tuple[float, float]] | tuple) -> float:
    if len(poly) < 2:
        raise ValueError("polygon needs at least 2 vertices")
    if point_in_poly(px, py, poly):
        return 0.0
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        best = min(best, point_segment_distance(px, py, ax, ay, bx, by))
    return best


def signed_area(poly: list[tuple[float, float]] | tuple) -> float:
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def polygon_area(poly: list[tuple[float, float]] | tuple) -> float:
    return abs(signed_area(poly))


def _orientation(ax: float, ay: float, bx: float, by: float,
                 cx: float, cy: float) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def _on_segment(ax: float, ay: float, bx: float, by: float,
                px: float, py: float) -> bool:
    return (min(ax, bx) - 1e-12 <= px <= max(ax, bx) + 1e-12
            and min(ay, by) - 1e-12 <= py <= max(ay, by) + 1e-12)


def segments_intersect(a: tuple[float, float], b: tuple[float, float],
                       c: tuple[float, float], d: tuple[float, float]) -> bool:
    o1 = _orientation(a[0], a[1], b[0], b[1], c[0], c[1])
    o2 = _orientation(a[0], a[1], b[0], b[1], d[0], d[1])
    o3 = _orientation(c[0], c[1], d[0], d[1], a[0], a[1])
    o4 = _orientation(c[0], c[1], d[0], d[1], b[0], b[1])
    if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0):
        return True
    if abs(o1) < 1e-12 and _on_segment(a[0], a[1], b[0], b[1], c[0], c[1]):
        return True
    if abs(o2) < 1e-12 and _on_segment(a[0], a[1], b[0], b[1], d[0], d[1]):
        return True
    if abs(o3) < 1e-12 and _on_segment(c[0], c[1], d[0], d[1], a[0], a[1]):
        return True
    if abs(o4) < 1e-12 and _on_segment(c[0], c[1], d[0], d[1], b[0], b[1]):
        return True
    return False


def segment_intersects_polygon(a: tuple[float, float], b: tuple[float, float],
                               poly: list[tuple[float, float]] | tuple) -> bool:
    if point_in_poly(a[0], a[1], poly) or point_in_poly(b[0], b[1], poly):
        return True
    n = len(poly)
    for i in range(n):
        c = poly[i]
        d = poly[(i + 1) % n]
        if segments_intersect(a, b, c, d):
            return True
    return False


def polygon_is_simple(poly: list[tuple[float, float]] | tuple) -> bool:
    """True for a non-degenerate simple polygon (no self-intersections)."""
    n = len(poly)
    if n < 3 or polygon_area(poly) < 1e-9:
        return False
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or j == (i + 1) % n:
                continue
            c = poly[j]
            d = poly[(j + 1) % n]
            if segments_intersect(a, b, c, d):
                return False
    return True


def _line_intersection(l1, l2):
    x1, y1, dx1, dy1 = l1
    x2, y2, dx2, dy2 = l2
    den = dx1 * dy2 - dy1 * dx2
    if abs(den) < 1e-12:
        return None
    t = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / den
    return x1 + t * dx1, y1 + t * dy1


def offset_polygon(poly: list[tuple[float, float]] | tuple, distance: float):
    """Offset a simple polygon outwards by ``distance`` metres.

    Works for convex polygons (our no-fly zone).  For concave polygons the
    result may self-intersect; callers must validate it.
    """
    n = len(poly)
    if n < 3:
        raise ValueError("polygon needs at least 3 vertices")
    ccw = signed_area(poly) > 0
    lines = []
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-12:
            raise ValueError("degenerate polygon edge")
        # Outward normal for a CCW polygon is (dy, -dx).
        nx, ny = dy / length, -dx / length
        if not ccw:
            nx, ny = -nx, -ny
        lines.append((ax + nx * distance, ay + ny * distance, dx, dy))
    out = []
    for i in range(n):
        p = _line_intersection(lines[(i - 1) % n], lines[i])
        if p is None:
            p = (lines[i][0], lines[i][1])
        out.append(p)
    return out


def expand_polygon(poly: list[tuple[float, float]] | tuple, distance: float):
    return offset_polygon(poly, distance)
