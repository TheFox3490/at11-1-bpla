"""Fast regression tests for the pipeline (run with ``pytest``).

These do not open a window: they exercise the validators, the planner, the
merge step and the action registry.  The visual part of the acceptance protocol
(C1-C8 on the real field) lives in ``tests/bench.py`` and is started either from
the application overlay (F9) or with ``python app.py --tests``.
"""
from __future__ import annotations

import math
from datetime import timedelta

import pytest

from cv.detect import Detection
from cv.merge import merge_detections
from cv.mission import apply_subset, build_mission
from cv.planner import plan
from cv.validator import operation_gate, validate_mission
from shared.contracts import checksum_of, new_id, utc_now
from shared.journal import ActionRegistry
from shared.world import (
    FORBIDDEN_EXPANSION, HOME, MAX_RECON_ERROR, NO_FLY,
    expand_polygon, segment_intersects_polygon,
)
from tests.cases import build_bench_mission


def test_units_must_be_metres():
    mission = build_bench_mission([(2.4, 2.0)], units="cm")
    assert "units_must_be_m" in validate_mission(mission)


def test_unknown_calibration_version_is_rejected():
    mission = build_bench_mission([(2.4, 2.0)], version="calib_never_ran")
    errors = validate_mission(mission, known_calibrations={"calib_run_a", "calib_run_b"})
    assert "unknown_calibration_version" in errors


def test_manual_training_version_is_allowed():
    mission = build_bench_mission([(2.4, 2.0)], version="manual_training")
    errors = validate_mission(mission, known_calibrations=set())
    assert "unknown_calibration_version" not in errors


def test_expired_map_is_rejected():
    mission = build_bench_mission(
        [(2.4, 2.0)], ttl_s=60.0,
        created_at=utc_now() - timedelta(minutes=30))
    assert "map_expired" in validate_mission(mission)


def test_checksum_is_recomputed_not_trusted():
    mission = build_bench_mission([(2.4, 2.0)])
    mission["units"] = "mm"          # damage after signing
    errors = validate_mission(mission)
    assert "checksum_mismatch" in errors
    assert "units_must_be_m" in errors


def test_planner_subset_records_reasons():
    targets = [
        {"target_id": f"T{i:02d}", "action_id": f"a{i}", "klass": "A",
         "point": {"x": x, "y": y}, "position_error_m": 0.07, "priority": i}
        for i, (x, y) in enumerate(
            [(1.2, 0.8), (2.0, 0.8), (2.8, 0.8), (1.2, 2.0), (2.0, 2.0), (2.8, 2.0)], 1)
    ]
    result = plan(HOME, targets, max_time=45.0)
    assert result["skipped"], "45 s must not fit all six targets with reserve"
    assert all(item["reason"] for item in result["skipped"])
    assert result["total_with_reserve_s"] <= 45.0
    assert len(result["comparison"]) >= 2, "two order-selection methods are required"


def test_route_never_enters_the_expanded_zone():
    targets = [
        {"target_id": f"T{i:02d}", "action_id": f"a{i}", "klass": "A",
         "point": {"x": x, "y": y}, "position_error_m": 0.07, "priority": i}
        for i, (x, y) in enumerate([(4.2, 2.2), (3.4, 1.2), (2.0, 2.0)], 1)
    ]
    result = plan(HOME, targets)
    expanded = [(p["x"], p["y"]) for p in result["expanded_forbidden"]]
    pts = [(p["x"], p["y"]) for p in result["route"]]
    for a, b in zip(pts, pts[1:]):
        assert not segment_intersects_polygon(a, b, expanded), "leg crosses NFZ+0.23"


def test_target_inside_expanded_zone_is_rejected():
    inside = (4.9, 2.5)
    assert distance_to_zone(*inside) <= FORBIDDEN_EXPANSION
    observation = _obs("A", *inside)
    mission, rejected = build_mission([observation], "run", "calib_x",
                                      last_frame_at=utc_now())
    assert mission["targets"] == []
    reasons = rejected[0]["reasons"]
    assert any("inside_expanded_forbidden_zone" in r for r in reasons)


def test_conflicting_observations_become_u():
    det_a = Detection("A", 100.0, 100.0, 0.5, 0.5, 0.9, frame_id="a", x=2.5, y=1.5)
    det_b = Detection("B", 103.0, 101.0, 0.5, 0.5, 0.9, frame_id="b", x=2.5, y=1.5)
    observations = merge_detections([det_a, det_b])
    assert observations and observations[0].klass == "U"
    mission, rejected = build_mission(observations, "run", "calib_x",
                                      last_frame_at=utc_now())
    assert mission["targets"] == []
    assert "class_not_permitted" in rejected[0]["reasons"]


def test_noisy_observation_is_rejected_by_accuracy_budget():
    observation = _obs("A", 2.6, 1.4, spread_m=0.10)
    mission, rejected = build_mission([observation], "run", "calib_x",
                                      last_frame_at=utc_now(),
                                      recon_error_m=0.04)
    assert mission["targets"] == []
    reasons = rejected[0]["reasons"]
    assert ("position_error_out_of_limit" in reasons
            or "operation_accuracy_budget_exceeded" in reasons)
    assert rejected[0]["position_error_m"] > MAX_RECON_ERROR


def test_subset_mission_drops_skipped_targets():
    points = [(1.2, 0.8), (2.0, 0.8), (2.8, 0.8),
              (1.2, 2.0), (2.0, 2.0), (2.8, 2.0)]
    mission = build_bench_mission(points)
    planning = plan(HOME, mission["targets"], max_time=45.0)
    subset = apply_subset(mission, planning, 45.0)
    dropped = {item["target_id"] for item in planning["skipped"]}
    assert dropped
    assert not dropped & {t["target_id"] for t in subset["targets"]}
    assert subset["partial"] is True
    assert subset["budget_s"] == 45.0
    assert not validate_mission(subset)


@pytest.mark.parametrize("kwargs, expected", [
    ({"stale": True}, "stale_localization"),
    ({"link": False}, "control_link_lost"),
    ({"already": True}, "duplicate_action_blocked"),
    ({"altitude": 0.5}, "altitude_out_of_range"),
])
def test_operation_gate_blocks_unsafe_actions(kwargs, expected):
    permitted, reason = operation_gate(
        map_valid=True, class_a=True,
        position_fresh=not kwargs.get("stale", False),
        link_ok=kwargs.get("link", True),
        altitude_m=kwargs.get("altitude", 1.2),
        stable_inside_area_seconds=1.5,
        position_error_m=0.07,
        already_actioned=kwargs.get("already", False),
    )
    assert permitted is False
    assert reason == expected


def test_registry_uncertain_blocks_repeat(tmp_path):
    path = tmp_path / "registry.json"
    mission = build_bench_mission([(3.3, 2.2)])
    action_id = mission["targets"][0]["action_id"]
    registry = ActionRegistry(path)
    assert not registry.is_done(action_id)
    registry.mark(action_id, "uncertain", reason="crash_after_led_on")
    # A fresh object reads the file: this is what an application restart sees.
    restarted = ActionRegistry(path)
    assert restarted.is_done(action_id)
    assert restarted.get(action_id)["status"] == "uncertain"


# --------------------------------------------------------------------------- #
def _obs(klass: str, x: float, y: float, spread_m: float = 0.0):
    from cv.merge import Observation
    return Observation(oid="O01", klass=klass, x=x, y=y, n_obs=3,
                       votes={klass: 1.0}, frames=["a", "b", "c"],
                       score=0.8, spread_m=spread_m)


def distance_to_zone(x: float, y: float) -> float:
    from shared.world import distance_to_polygon
    return distance_to_polygon(x, y, NO_FLY)
