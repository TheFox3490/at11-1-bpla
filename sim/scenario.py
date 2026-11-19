"""Scenario layout store: what the simulator actually draws on the field.

Mode 2 edits this layout, mode 3 flies the recon over it.  The CV pipeline
never reads this file: the mission coordinates still come only from the
frames.  That is the difference between "editing the polygon" (allowed) and
"entering target coordinates after recon" (forbidden by the task).

The file lives outside the run/mission artifacts so a drawn map survives an
application restart.  ``data/layout.json`` holds the applied layout; the
editor keeps a draft in memory until the operator presses "Применить к полю".
"""
from __future__ import annotations

import math
from pathlib import Path

from shared.contracts import iso, read_json, write_json
from shared.world import (
    CALIB_MARKERS, CHECK_MARKERS, FIELD_H, FIELD_W, FLIGHT_FIELD_MARGIN,
    FORBIDDEN_EXPANSION, HOME, NO_FLY, PLOT_SIZE, expand_polygon,
    segment_intersects_polygon,
)
from sim.field import PLOTS

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LAYOUT_PATH = PROJECT_ROOT / "data" / "layout.json"

KLASSES = ("A", "B", "U")
MIN_GAP = PLOT_SIZE * 1.02          # plots must not overlap
MARKER_CLEARANCE = 0.18             # keep the calibration pattern readable
HOME_CLEARANCE = 0.30               # clear space around each take-off point
FIELD_MARGIN = PLOT_SIZE / 2 + 0.01
CALIBRATION_BOUNDS = (0.34, 1.26)
START_NAMES = {"recon": "Разведчик", "executor": "Исполнитель"}
START_FORBIDDEN = expand_polygon(NO_FLY, FORBIDDEN_EXPANSION)


def default_starts() -> dict[str, tuple[float, float]]:
    return {"recon": (0.95, 1.70), "executor": HOME}


def default_objects() -> list[dict]:
    """The base layout from the task: 6 reachable A, 2 A in the NFZ, 1 B, 1 U."""
    return [
        {"id": pid, "klass": klass, "x": x, "y": y, "priority": index}
        for index, (pid, klass, (x, y)) in enumerate(PLOTS, 1)
    ]


def copy_objects(objects: list[dict]) -> list[dict]:
    return [dict(obj) for obj in objects]


def next_id(objects: list[dict]) -> str:
    used = {str(obj.get("id", "")) for obj in objects}
    index = 1
    while f"P{index:02d}" in used:
        index += 1
    return f"P{index:02d}"


def validate(objects: list[dict], strict_default: bool = False, *,
             starts: dict[str, tuple[float, float]] | None = None) -> list[str]:
    """Physical checks for a drawable layout; returns human-readable errors."""
    errors: list[str] = []
    if not objects:
        return ["карта пуста: нужен хотя бы один участок"]
    if len(objects) > 20:
        errors.append(f"слишком много участков: {len(objects)} (максимум 20)")

    starts = default_starts() if starts is None else starts
    margin = FLIGHT_FIELD_MARGIN
    for role, name in START_NAMES.items():
        point = starts[role]
        x, y = point
        if not (margin <= x <= FIELD_W - margin and margin <= y <= FIELD_H - margin):
            errors.append(f"{name}: старт выходит за рабочие границы поля")
        elif segment_intersects_polygon(point, point, START_FORBIDDEN):
            errors.append(f"{name}: старт в расширенной запретной зоне")
        low, high = CALIBRATION_BOUNDS
        if low <= x <= high and low <= y <= high:
            errors.append(f"{name}: старт на площадке калибровки")

    seen: set[str] = set()
    for obj in objects:
        oid = str(obj.get("id", "?"))
        if oid in seen:
            errors.append(f"{oid}: повторяющийся id")
        seen.add(oid)
        if obj.get("klass") not in KLASSES:
            errors.append(f"{oid}: неизвестный класс {obj.get('klass')!r}")
        try:
            x, y = float(obj["x"]), float(obj["y"])
        except (KeyError, TypeError, ValueError):
            errors.append(f"{oid}: нет координат")
            continue
        if not (FIELD_MARGIN <= x <= FIELD_W - FIELD_MARGIN
                and FIELD_MARGIN <= y <= FIELD_H - FIELD_MARGIN):
            errors.append(f"{oid}: выходит за поле ({x:.2f}, {y:.2f})")

    points = []
    for obj in objects:
        try:
            points.append((str(obj.get("id", "?")), float(obj["x"]), float(obj["y"])))
        except (KeyError, TypeError, ValueError):
            continue
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            (a, ax, ay), (b, bx, by) = points[i], points[j]
            if math.dist((ax, ay), (bx, by)) < MIN_GAP:
                errors.append(f"{a} и {b}: участки перекрываются")
    for oid, x, y in points:
        for name, marker in {**CALIB_MARKERS, **CHECK_MARKERS}.items():
            if math.dist((x, y), marker[:2]) < MARKER_CLEARANCE:
                errors.append(f"{oid}: закрывает маркер калибровки {name}")
        for role, start in starts.items():
            if math.dist((x, y), start) < HOME_CLEARANCE:
                errors.append(f"{oid}: слишком близко к старту ({START_NAMES[role]})")

    if strict_default and not errors:
        counts = {k: sum(1 for o in objects if o.get("klass") == k) for k in KLASSES}
        if counts != {"A": 8, "B": 1, "U": 1}:
            errors.append("базовая раскладка должна содержать A=8, B=1, U=1")
    return errors


def load_layout() -> tuple[list[dict], dict[str, tuple[float, float]]]:
    """Applied layout; falls back to the task layout when the file is absent."""
    try:
        payload = read_json(LAYOUT_PATH)
        objects = payload["objects"]
        if not isinstance(objects, list) or not objects:
            raise ValueError("empty layout")
        objects = [{**obj, "x": float(obj["x"]), "y": float(obj["y"]),
                    "klass": str(obj["klass"])} for obj in objects]
        if "starts" in payload:
            starts = {role: (float(payload["starts"][role]["x"]),
                             float(payload["starts"][role]["y"])) for role in START_NAMES}
        else:
            starts = dict.fromkeys(START_NAMES, HOME)
        if validate(objects, starts=starts):
            raise ValueError("layout fails physical checks")
        return objects, starts
    except (OSError, ValueError, KeyError, TypeError):
        return default_objects(), default_starts()


def save_layout(objects: list[dict], starts: dict[str, tuple[float, float]]) -> Path:
    payload = {
        "version": 2,
        "updated_at": iso(),
        "units": "m",
        "coordinate_frame": "F",
        "objects": [{**obj, "x": round(float(obj["x"]), 3),
                     "y": round(float(obj["y"]), 3)} for obj in objects],
        "starts": {role: {"x": x, "y": y} for role, (x, y) in starts.items()},
    }
    return write_json(LAYOUT_PATH, payload)


def is_default(objects: list[dict]) -> bool:
    default = {(o["id"], o["klass"], round(o["x"], 3), round(o["y"], 3))
               for o in default_objects()}
    current = {(str(o.get("id")), str(o.get("klass")),
                round(float(o["x"]), 3), round(float(o["y"]), 3))
               for o in objects}
    return default == current
