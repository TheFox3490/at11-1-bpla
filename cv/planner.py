"""A4: route planning with expanded forbidden zones and a time budget."""
from __future__ import annotations

import itertools
import math

from shared.world import (
    FORBIDDEN_EXPANSION, OPERATION_SECONDS, SPEED_LIMIT, STABILIZE_SECONDS,
    NO_FLY, expand_polygon, point_in_poly, segment_intersects_polygon,
)


def _dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _pt(target) -> tuple[float, float]:
    """Target point as ``(x, y)``; missions store it as a dict."""
    point = target["point"]
    if isinstance(point, dict):
        return float(point["x"]), float(point["y"])
    return float(point[0]), float(point[1])


def _path_length(points) -> float:
    return sum(_dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def _find_leg(a, b, expanded, detour_points):
    """Shortest safe polyline a -> b, with a detour around a convex obstacle."""
    if not segment_intersects_polygon(a, b, expanded):
        return [a, b]
    best = None
    best_len = float("inf")
    for v in detour_points:
        if (segment_intersects_polygon(a, v, expanded)
                or segment_intersects_polygon(v, b, expanded)):
            continue
        length = _dist(a, v) + _dist(v, b)
        if length < best_len:
            best_len, best = length, [a, v, b]
    if best is not None:
        return best
    # Two intermediate vertices (needed when both endpoints are on the same side).
    for v in detour_points:
        for w in detour_points:
            if v == w:
                continue
            if (segment_intersects_polygon(a, v, expanded)
                    or segment_intersects_polygon(v, w, expanded)
                    or segment_intersects_polygon(w, b, expanded)):
                continue
            length = _dist(a, v) + _dist(v, w) + _dist(w, b)
            if length < best_len:
                best_len, best = length, [a, v, w, b]
    return best


def _route_for_order(order, start, expanded, detour_points):
    points = [start] + [_pt(t) for t in order]
    full = [start]
    total = 0.0
    for a, b in zip(points, points[1:]):
        leg = _find_leg(a, b, expanded, detour_points)
        if leg is None:
            return None
        full.extend(leg[1:])
        total += _path_length(leg)
    # Return to the take-off point.
    leg = _find_leg(points[-1], start, expanded, detour_points)
    if leg is None:
        return None
    full.extend(leg[1:])
    total += _path_length(leg)
    return full, total


def _estimate(total_length: float, n_targets: int, speed: float) -> tuple[float, float]:
    work = n_targets * (STABILIZE_SECONDS + OPERATION_SECONDS + 0.5)
    estimated = total_length / speed + work
    reserve = max(20.0, 0.2 * estimated)
    return estimated, reserve


def _best_order(targets, start, expanded, detour_points):
    if not targets:
        return [], [], 0.0, 0.0, "empty"
    nn = _nearest_neighbour(targets, start, expanded, detour_points)
    nn_route = _route_for_order(nn, start, expanded, detour_points)
    best_order, best_route = nn, nn_route
    method = "nearest_neighbour"
    if len(targets) <= 7:
        best_len = best_route[1] if best_route else float("inf")
        for perm in itertools.permutations(targets):
            route = _route_for_order(list(perm), start, expanded, detour_points)
            if route is not None and route[1] < best_len - 1e-9:
                best_order, best_route, best_len = list(perm), route, route[1]
                method = "exhaustive"
    if best_route is None:
        return [], [], 0.0, 0.0, "no_route"
    nn_len = nn_route[1] if nn_route else float("inf")
    return best_order, best_route[0], best_route[1], nn_len, method


def _nearest_neighbour(targets, start, expanded, detour_points):
    remaining = list(targets)
    order = []
    cur = start
    while remaining:
        feasible = []
        for t in remaining:
            leg = _find_leg(cur, _pt(t), expanded, detour_points)
            if leg is not None:
                feasible.append((_path_length(leg), t))
        if not feasible:
            break
        _, nxt = min(feasible, key=lambda item: item[0])
        order.append(nxt)
        remaining.remove(nxt)
        cur = _pt(nxt)
    return order


def plan(start: tuple[float, float], targets: list[dict],
         forbidden: list[tuple[float, float]] | None = None,
         speed: float = SPEED_LIMIT, max_time: float = 360.0) -> dict:
    forbidden = forbidden or NO_FLY
    expanded = expand_polygon(forbidden, FORBIDDEN_EXPANSION)
    # Detour points sit slightly outside the expanded zone so a segment does
    # not merely touch it.
    detour = expand_polygon(forbidden, FORBIDDEN_EXPANSION + 0.02)
    remaining = [dict(t) for t in targets]
    skipped: list[dict] = []
    comparison = {}
    while True:
        order, route, length, nn_length, method = _best_order(remaining, start, expanded, detour)
        estimated, reserve = _estimate(length, len(order), speed)
        comparison = {"nearest_neighbour_length_m": round(nn_length, 3),
                      "chosen_length_m": round(length, 3),
                      "chosen_method": method}
        if estimated + reserve <= max_time or not order:
            break
        # Drop the lowest-priority (highest number) target and try again.
        drop = max(order, key=lambda t: t.get("priority", 0))
        remaining.remove(drop)
        skipped.append({"target_id": drop["target_id"],
                        "reason": "insufficient_time_with_reserve"})
    estimated, reserve = _estimate(length, len(order), speed)
    return {
        "start": {"x": start[0], "y": start[1]},
        "order": [t["target_id"] for t in order],
        "targets": [t for t in order],
        "route": [{"x": round(p[0], 4), "y": round(p[1], 4)} for p in route],
        "length_m": round(length, 3),
        "estimated_s": round(estimated, 1),
        "reserve_s": round(reserve, 1),
        "total_with_reserve_s": round(estimated + reserve, 1),
        "method": method,
        "comparison": comparison,
        "skipped": skipped,
        "expanded_forbidden": [{"x": round(x, 4), "y": round(y, 4)}
                               for x, y in expanded],
        "valid": bool(order),
    }
