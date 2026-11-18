"""Two-dimensional local navigation frames and their relation to field F.

The recon and executor drones must not assume that their local navigation
frame equals F.  The simulator knows the true transform (for rendering and
ground truth), while the CV pipeline *estimates* L -> F from the four
calibration markers.  Keeping the transforms in one place prevents sign and
rotation mistakes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from shared.world import LOCAL_FRAMES, wrap_deg


@dataclass(frozen=True)
class LocalFrame:
    name: str
    origin: tuple[float, float]
    yaw_deg: float

    @classmethod
    def from_spec(cls, name: str, spec: dict | None = None) -> "LocalFrame":
        spec = spec or LOCAL_FRAMES[name]
        return cls(name, tuple(spec["origin"]), float(spec["yaw_deg"]))

    def to_local(self, x: float, y: float) -> tuple[float, float]:
        """Field F (metres) -> local L (metres)."""
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        return dx * c + dy * s, -dx * s + dy * c

    def to_field(self, x: float, y: float) -> tuple[float, float]:
        """Local L (metres) -> field F (metres)."""
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        dx = x * c - y * s
        dy = x * s + y * c
        return dx + self.origin[0], dy + self.origin[1]

    def pose_to_local(self, x: float, y: float, yaw_deg: float) -> tuple[float, float, float]:
        """Field pose -> local position and *image* yaw.

        The camera is rigidly attached to the airframe, so the image is rotated
        by the field-frame heading.  A navigation solution reports the heading
        relative to L, therefore the image yaw is ``yaw_L + frame_yaw`` (the
        local frame rotation is added back).  Getting this sign wrong rotates
        every detection by twice the frame yaw.
        """
        lx, ly = self.to_local(x, y)
        return lx, ly, wrap_deg(yaw_deg + self.yaw_deg)


def similarity_from_points(src: list[tuple[float, float]],
                           dst: list[tuple[float, float]]):
    """Least-squares similarity (scale + rotation + translation) src -> dst.

    Returns ``(scale, yaw_deg, tx, ty)`` so that
    ``dst = scale * R(yaw) * src + (tx, ty)``.
    Implemented directly with the closed-form 2-D Umeyama solution; no OpenCV
    dependency, so the transform can be tested in isolation.
    """
    n = len(src)
    if n < 2 or len(dst) != n:
        raise ValueError("need at least two point pairs")
    sx = sum(p[0] for p in src) / n
    sy = sum(p[1] for p in src) / n
    dx = sum(p[0] for p in dst) / n
    dy = sum(p[1] for p in dst) / n
    num = 0.0
    den = 0.0
    for (ax, ay), (bx, by) in zip(src, dst):
        ax -= sx
        ay -= sy
        bx -= dx
        by -= dy
        num += ax * by - ay * bx
        den += ax * bx + ay * by
    if abs(den) < 1e-15 and abs(num) < 1e-15:
        raise ValueError("degenerate point set")
    scale = math.hypot(den, num) / sum((p[0] - sx) ** 2 + (p[1] - sy) ** 2 for p in src)
    yaw = math.atan2(num, den)
    c, s = math.cos(yaw), math.sin(yaw)
    tx = dx - scale * (c * sx - s * sy)
    ty = dy - scale * (s * sx + c * sy)
    return scale, math.degrees(yaw), tx, ty


def apply_similarity(transform, x: float, y: float) -> tuple[float, float]:
    scale, yaw_deg, tx, ty = transform
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return scale * (c * x - s * y) + tx, scale * (s * x + c * y) + ty


def transform_record(transform) -> dict:
    scale, yaw_deg, tx, ty = transform
    return {"scale": scale, "yaw_deg": yaw_deg, "tx": tx, "ty": ty}
