"""A3: mission-map validator.

Adapted from ``prototype_validator_AT11_2.py`` with the fixes agreed in the
review:

* the checksum is actually recomputed, not just checked for presence;
* the map lifetime is driven by ``valid_until`` (last used recon frame + 15 min);
* ``valid_until`` must be after ``created_at`` and ``created_at`` must not be
  in the future;
* polygons are checked for size, self-intersection and field bounds;
* the effective recon-error limit is derived from the accuracy budget (0.13 m),
  not the contradictory 0.15 m;
* the operation gate also accepts the geometric checks that the original
  function left to the caller.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from typing import Iterable

from shared.contracts import checksum_of, parse_iso, utc_now
from shared.world import (
    ALIGNMENT_ERROR, EXECUTOR_ERROR, FIELD_H, FIELD_W, FORBIDDEN_EXPANSION,
    IMPACT_RADIUS, MAX_RECON_ERROR, TARGET_RADIUS, distance_to_polygon,
    point_in_poly, polygon_area, polygon_is_simple,
)


@dataclass(frozen=True)
class Point:
    x: float
    y: float


@dataclass(frozen=True)
class Target:
    target_id: str
    action_id: str
    klass: str
    point: Point
    position_error_m: float


@dataclass(frozen=True)
class Polygon:
    zone_id: str
    vertices: tuple[Point, ...]


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    dx, dy = b.x - a.x, b.y - a.y
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(p.x - a.x, p.y - a.y)
    t = max(0.0, min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / denom))
    q = Point(a.x + t * dx, a.y + t * dy)
    return math.hypot(p.x - q.x, p.y - q.y)


def point_in_polygon(p: Point, poly: Polygon) -> bool:
    return point_in_poly(p.x, p.y, [(v.x, v.y) for v in poly.vertices])


def distance_to_polygon_poly(p: Point, poly: Polygon) -> float:
    return distance_to_polygon(p.x, p.y, [(v.x, v.y) for v in poly.vertices])


def validate_polygon(poly: Polygon) -> list[str]:
    errors = []
    if not poly.zone_id:
        errors.append("missing_zone_id")
    pts = [(v.x, v.y) for v in poly.vertices]
    if len(pts) < 3:
        errors.append("zone_too_few_vertices")
        return errors
    if polygon_area(pts) < 1e-9:
        errors.append("zone_zero_area")
    if not polygon_is_simple(pts):
        errors.append("zone_self_intersecting")
    for x, y in pts:
        if not (0 <= x <= FIELD_W and 0 <= y <= FIELD_H):
            errors.append("zone_outside_field_F")
            break
    return errors


def validate_target(t: Target, forbidden: Iterable[Polygon]) -> list[str]:
    errors = []
    if not t.target_id or not t.action_id:
        errors.append("missing_unique_ids")
    if t.klass != "A":
        errors.append("class_not_permitted")
    if not (0 <= t.point.x <= FIELD_W and 0 <= t.point.y <= FIELD_H):
        errors.append("outside_field_F")
    if t.position_error_m < 0 or t.position_error_m > MAX_RECON_ERROR:
        errors.append("position_error_out_of_limit")
    if t.position_error_m + EXECUTOR_ERROR + ALIGNMENT_ERROR + IMPACT_RADIUS > TARGET_RADIUS:
        errors.append("operation_accuracy_budget_exceeded")
    for zone in forbidden:
        if distance_to_polygon_poly(t.point, zone) <= FORBIDDEN_EXPANSION:
            errors.append(f"inside_expanded_forbidden_zone:{zone.zone_id}")
    return errors


def validate_map_header(data: dict, now: datetime | None = None) -> list[str]:
    errors = []
    if data.get("coordinate_frame") != "F":
        errors.append("wrong_or_missing_coordinate_frame")
    if data.get("units") != "m":
        errors.append("units_must_be_m")
    if not data.get("mission_id"):
        errors.append("missing_mission_id")
    if not data.get("calibration_version"):
        errors.append("missing_calibration_version")
    if not data.get("checksum"):
        errors.append("missing_checksum")
    try:
        created = parse_iso(data["created_at"])
        valid_until = parse_iso(data["valid_until"])
        now = now or utc_now()
        if created.tzinfo is None or valid_until.tzinfo is None or now.tzinfo is None:
            errors.append("timestamps_must_include_timezone")
        else:
            if valid_until <= created:
                errors.append("valid_until_not_after_created_at")
            if created > now:
                errors.append("created_at_in_future")
            if now > valid_until:
                errors.append("map_expired")
    except (KeyError, ValueError, TypeError):
        errors.append("invalid_or_missing_timestamp")
    if not isinstance(data.get("targets"), list):
        errors.append("targets_must_be_list")
    return errors


def duplicate_ids(targets: list[Target]) -> list[str]:
    ids = [t.target_id for t in targets]
    actions = [t.action_id for t in targets]
    errors = []
    if len(ids) != len(set(ids)):
        errors.append("duplicate_target_id")
    if len(actions) != len(set(actions)):
        errors.append("duplicate_action_id")
    return errors


def verify_checksum(data: dict) -> bool:
    return bool(data.get("checksum")) and data["checksum"] == checksum_of(data)


def _target_from_dict(item: dict) -> Target:
    point = item["point"]
    return Target(str(item["target_id"]), str(item["action_id"]), str(item["klass"]),
                  Point(float(point["x"]), float(point["y"])),
                  float(item["position_error_m"]))


def _zone_from_dict(item: dict) -> Polygon:
    return Polygon(str(item["zone_id"]),
                   tuple(Point(float(p["x"]), float(p["y"])) for p in item["vertices"]))


def validate_mission(data: dict, now: datetime | None = None,
                     check_checksum: bool = True) -> list[str]:
    errors = validate_map_header(data, now)
    if check_checksum and not verify_checksum(data):
        errors.append("checksum_mismatch")
    zones = []
    for item in data.get("forbidden", []) or []:
        try:
            zone = _zone_from_dict(item)
        except (KeyError, TypeError, ValueError):
            errors.append("invalid_forbidden_zone")
            continue
        errors.extend(validate_polygon(zone))
        zones.append(zone)
    targets = []
    for item in data.get("targets", []) or []:
        try:
            target = _target_from_dict(item)
        except (KeyError, TypeError, ValueError):
            errors.append("invalid_target_entry")
            continue
        errors.extend(validate_target(target, zones))
        targets.append(target)
    errors.extend(duplicate_ids(targets))
    if not targets:
        errors.append("no_targets")
    return errors


def operation_gate(*, map_valid: bool, class_a: bool, position_fresh: bool,
                   altitude_m: float, stable_inside_area_seconds: float,
                   position_error_m: float, already_actioned: bool,
                   distance_to_target_m: float | None = None,
                   target_in_forbidden: bool = False,
                   drone_in_forbidden: bool = False) -> tuple[bool, str]:
    """Pure safety decision; does not actuate hardware."""
    if not map_valid:
        return False, "invalid_map"
    if target_in_forbidden or drone_in_forbidden:
        return False, "forbidden_zone"
    if not class_a:
        return False, "target_not_A"
    if not position_fresh:
        return False, "stale_localization"
    if not (1.0 <= altitude_m <= 1.4):
        return False, "altitude_out_of_range"
    if stable_inside_area_seconds < 1.0:
        return False, "not_stable_for_1s"
    if distance_to_target_m is not None and distance_to_target_m > TARGET_RADIUS:
        return False, "outside_target_radius"
    if position_error_m + EXECUTOR_ERROR + ALIGNMENT_ERROR + IMPACT_RADIUS > TARGET_RADIUS:
        return False, "accuracy_budget_exceeded"
    if already_actioned:
        return False, "duplicate_action_blocked"
    return True, "permitted_for_simulation_only"
