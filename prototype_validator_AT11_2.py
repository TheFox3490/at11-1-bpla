"""
Учебный валидатор карты задания АТ11-2.
Это не автопилот и не код управления реальным БПЛА.
Включение нагрузки намеренно не реализовано: исполнительный контур должен
быть отдельно проверен и сертифицирован для конкретной платформы.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
from math import hypot
from typing import Iterable

FIELD_X, FIELD_Y = 6.0, 4.0
MAX_POSITION_ERROR = 0.15
TARGET_RADIUS = 0.25
IMPACT_RADIUS = 0.04
FORBIDDEN_EXPANSION = 0.23
MAX_MAP_AGE_SECONDS = 15 * 60

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
    dx, dy = b.x-a.x, b.y-a.y
    denom = dx*dx + dy*dy
    if denom == 0:
        return hypot(p.x-a.x, p.y-a.y)
    t = max(0.0, min(1.0, ((p.x-a.x)*dx+(p.y-a.y)*dy)/denom))
    q = Point(a.x+t*dx, a.y+t*dy)
    return hypot(p.x-q.x, p.y-q.y)

def point_in_polygon(p: Point, poly: Polygon) -> bool:
    inside = False
    v = poly.vertices
    j = len(v)-1
    for i in range(len(v)):
        xi, yi = v[i].x, v[i].y
        xj, yj = v[j].x, v[j].y
        if ((yi > p.y) != (yj > p.y)):
            cross_x = (xj-xi)*(p.y-yi)/(yj-yi) + xi
            if p.x < cross_x:
                inside = not inside
        j = i
    return inside

def distance_to_polygon(p: Point, poly: Polygon) -> float:
    if point_in_polygon(p, poly):
        return 0.0
    v = poly.vertices
    return min(point_segment_distance(p, v[i], v[(i+1) % len(v)])
               for i in range(len(v)))

def validate_target(t: Target, forbidden: Iterable[Polygon]) -> list[str]:
    errors = []
    if not t.target_id or not t.action_id:
        errors.append("missing_unique_ids")
    if t.klass != "A":
        errors.append("class_not_permitted")
    if not (0 <= t.point.x <= FIELD_X and 0 <= t.point.y <= FIELD_Y):
        errors.append("outside_field_F")
    if t.position_error_m < 0 or t.position_error_m > MAX_POSITION_ERROR:
        errors.append("position_error_out_of_limit")
    if t.position_error_m + 0.06 + 0.02 + IMPACT_RADIUS > TARGET_RADIUS:
        errors.append("operation_accuracy_budget_exceeded")
    for zone in forbidden:
        if distance_to_polygon(t.point, zone) <= FORBIDDEN_EXPANSION:
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
        created = datetime.fromisoformat(data["created_at"].replace("Z","+00:00"))
        valid_until = datetime.fromisoformat(data["valid_until"].replace("Z","+00:00"))
        now = now or datetime.now(timezone.utc)
        if created.tzinfo is None or valid_until.tzinfo is None or now.tzinfo is None:
            errors.append("timestamps_must_include_timezone")
        elif now > valid_until:
            errors.append("map_expired")
        elif (now-created).total_seconds() > MAX_MAP_AGE_SECONDS:
            errors.append("map_older_than_15_minutes")
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

def operation_gate(*, map_valid: bool, class_a: bool, position_fresh: bool,
                   altitude_m: float, stable_inside_area_seconds: float,
                   position_error_m: float, already_actioned: bool) -> tuple[bool, str]:
    """Pure safety decision; does not actuate hardware."""
    if not map_valid: return False, "invalid_map"
    if not class_a: return False, "target_not_A"
    if not position_fresh: return False, "stale_localization"
    if not (1.0 <= altitude_m <= 1.4): return False, "altitude_out_of_range"
    if stable_inside_area_seconds < 1.0: return False, "not_stable_for_1s"
    if position_error_m + 0.06 + 0.02 + IMPACT_RADIUS > TARGET_RADIUS:
        return False, "accuracy_budget_exceeded"
    if already_actioned: return False, "duplicate_action_blocked"
    return True, "permitted_for_simulation_only"

if __name__ == "__main__":
    sample = Target("A-01", "ACT-01", "A", Point(1.0, 1.0), 0.07)
    print("Target validation:", validate_target(sample, []))
    print("Gate:", operation_gate(
        map_valid=True, class_a=True, position_fresh=True,
        altitude_m=1.2, stable_inside_area_seconds=1.2,
        position_error_m=0.07, already_actioned=False))
