"""A4: route planning with expanded forbidden zones and a time budget.

Two order-selection methods are always evaluated and recorded, as the task
requires ("сравните хотя бы два способа выбора порядка целей"):

* ``nearest_neighbour`` — greedy nearest first, then return home;
* ``exhaustive`` — every permutation (up to 7 targets) or a 2-opt refinement
  of the greedy order for larger sets.

The shortest safe tour wins.  If even it does not fit into the time budget
with the reserve, the least important target (highest priority number) is
dropped and the reasons are written into ``skipped`` — the A4/C8 behaviour.
"""
from __future__ import annotations

import itertools
import math

from shared.world import (
    EXEC_LIMIT_SECONDS, FORBIDDEN_EXPANSION, OPERATION_SECONDS, SPEED_LIMIT,
    STABILIZE_SECONDS, NO_FLY, expand_polygon, segment_intersects_polygon,
)

MAX_EXHAUSTIVE = 7
TRANSACTION_SECONDS = 0.5


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
    work = n_targets * (STABILIZE_SECONDS + OPERATION_SECONDS + TRANSACTION_SECONDS)
    estimated = total_length / speed + work
    reserve = max(20.0, 0.2 * estimated)
    return estimated, reserve


def _summary(method: str, order, route, speed: float) -> dict:
    estimated, reserve = _estimate(route[1], len(order), speed)
    return {
        "method": method,
        "order": [str(t["target_id"]) for t in order],
        "length_m": round(route[1], 3),
        "estimated_s": round(estimated, 1),
        "reserve_s": round(reserve, 1),
        "total_with_reserve_s": round(estimated + reserve, 1),
        "route": route[0],
    }


def _nearest_neighbour(targets, start, expanded, detour_points):
    remaining = list(targets)
    order = []
    cur = start
    while remaining:
        feasible = []
        for t in remaining:
            leg = _find_leg(cur, _pt(t), expanded, detour_points)
            if leg is not None:
                feasible.append((_path_length(leg), str(t["target_id"]), t))
        if not feasible:
            break
        _, _, nxt = min(feasible, key=lambda item: (item[0], item[1]))
        order.append(nxt)
        remaining.remove(nxt)
        cur = _pt(nxt)
    return order


def _exhaustive(targets, start, expanded, detour_points):
    best_key, best_order, best_route = None, None, None
    for perm in itertools.permutations(targets):
        route = _route_for_order(list(perm), start, expanded, detour_points)
        if route is None:
            continue
        key = (round(route[1], 9), tuple(str(t["target_id"]) for t in perm))
        if best_key is None or key < best_key:
            best_key, best_order, best_route = key, list(perm), route
    return best_order, best_route


def _two_opt(order, start, expanded, detour_points):
    """Improve the greedy order for large sets; still a distinct method."""
    best = list(order)
    route = _route_for_order(best, start, expanded, detour_points)
    best_len = route[1] if route else float("inf")
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                candidate = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                candidate_route = _route_for_order(candidate, start, expanded, detour_points)
                if candidate_route is None:
                    continue
                if candidate_route[1] < best_len - 1e-9:
                    best, best_len, improved = candidate, candidate_route[1], True
    return best


def _method_results(targets, start, expanded, detour, speed=SPEED_LIMIT):
    results: list[dict] = []
    greedy = _nearest_neighbour(targets, start, expanded, detour)
    greedy_route = _route_for_order(greedy, start, expanded, detour) if greedy else None
    if greedy_route is not None:
        results.append(_summary("nearest_neighbour", greedy, greedy_route, speed))

    if len(targets) <= MAX_EXHAUSTIVE:
        order, route = _exhaustive(targets, start, expanded, detour)
        if route is not None:
            results.append(_summary("exhaustive", order, route, speed))
    else:
        improved = _two_opt(greedy, start, expanded, detour)
        route = _route_for_order(improved, start, expanded, detour)
        if route is not None:
            results.append(_summary("nearest_neighbour_2opt", improved, route, speed))
    return results


def plan(start: tuple[float, float], targets: list[dict],
         forbidden: list[tuple[float, float]] | None = None,
         speed: float = SPEED_LIMIT,
         max_time: float = EXEC_LIMIT_SECONDS) -> dict:
    forbidden = forbidden or NO_FLY
    expanded = expand_polygon(forbidden, FORBIDDEN_EXPANSION)
    # Detour points sit slightly outside the expanded zone so a segment does
    # not merely touch it.
    detour = expand_polygon(forbidden, FORBIDDEN_EXPANSION + 0.02)
    remaining = [dict(t) for t in targets]
    skipped: list[dict] = []
    comparison: list[dict] = []
    chosen: dict | None = None
    method = "none"

    while True:
        results = _method_results(remaining, start, expanded, detour, speed)
        for result in results:
            result["feasible"] = result["total_with_reserve_s"] <= max_time
        comparison = [{k: v for k, v in r.items() if k != "route"} for r in results]
        feasible = [r for r in results if r["feasible"]]
        pool = feasible or results
        if not pool:
            chosen = None
            break
        chosen = min(pool, key=lambda r: (r["length_m"], tuple(r["order"])))
        method = chosen["method"]
        if feasible or not chosen["order"]:
            break
        # Not even the shortest tour fits: drop the least important target.
        by_id = {str(t["target_id"]): t for t in remaining}
        drop_id = max(chosen["order"],
                      key=lambda tid: (int(by_id[tid].get("priority", 0)), tid))
        drop = by_id[drop_id]
        remaining.remove(drop)
        skipped.append({
            "target_id": drop_id,
            "priority": int(drop.get("priority", 0)),
            "point": dict(drop["point"]) if isinstance(drop["point"], dict)
                     else {"x": drop["point"][0], "y": drop["point"][1]},
            "reason": "insufficient_time_with_reserve",
            "budget_s": round(max_time, 1),
        })
        if not remaining:
            chosen = None
            break

    if chosen is None:
        return {
            "start": {"x": start[0], "y": start[1]},
            "order": [], "targets": [], "route": [],
            "length_m": 0.0, "estimated_s": 0.0, "reserve_s": 0.0,
            "total_with_reserve_s": 0.0,
            "method": "no_route", "chosen_method": "no_route",
            "comparison": comparison, "skipped": skipped,
            "expanded_forbidden": [{"x": round(x, 4), "y": round(y, 4)}
                                   for x, y in expanded],
            "budget_s": round(max_time, 1),
            "valid": False,
        }

    order_ids = chosen["order"]
    order_targets = [t for tid in order_ids for t in remaining if str(t["target_id"]) == tid]
    return {
        "start": {"x": start[0], "y": start[1]},
        "order": order_ids,
        "targets": order_targets,
        "route": [{"x": round(p[0], 4), "y": round(p[1], 4)} for p in chosen["route"]],
        "length_m": chosen["length_m"],
        "estimated_s": chosen["estimated_s"],
        "reserve_s": chosen["reserve_s"],
        "total_with_reserve_s": chosen["total_with_reserve_s"],
        "method": method,
        "chosen_method": method,
        "comparison": comparison,
        "skipped": skipped,
        "expanded_forbidden": [{"x": round(x, 4), "y": round(y, 4)}
                               for x, y in expanded],
        "budget_s": round(max_time, 1),
        "valid": bool(order_targets),
    }
