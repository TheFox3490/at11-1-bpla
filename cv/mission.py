"""Build the observation map and the mission map (A2 -> A3).

The observation map contains every detected object, including B/U and A inside
the forbidden zone.  The mission map contains only the permitted A targets and
is the file handed to the executor.
"""
from __future__ import annotations

import copy
from datetime import datetime, timedelta

from cv.merge import Observation
from cv.validator import Target, Point, Polygon, validate_target
from shared.contracts import checksum_of, iso, new_id, utc_now
from shared.world import (
    FIELD_H, FIELD_W, MAP_TTL_SECONDS, NO_FLY, RECON_ERROR,
)


def observation_map(observations: list[Observation], run_id: str,
                    calibration_version: str, calibration: dict | None = None,
                    created_at: datetime | None = None) -> dict:
    return {
        "run_id": run_id,
        "created_at": iso(created_at),
        "calibration_version": calibration_version,
        "coordinate_frame": "F",
        "units": "m",
        "objects": [o.as_dict() for o in observations],
        "forbidden": [{"zone_id": "NFZ-1",
                       "vertices": [{"x": x, "y": y} for x, y in NO_FLY]}],
        "calibration": calibration or {},
        "warning": "observation map; contains B/U and forbidden-zone objects",
    }


def build_mission(observations: list[Observation], run_id: str,
                  calibration_version: str, last_frame_at: datetime,
                  created_at: datetime | None = None,
                  forbidden: list[tuple[float, float]] | None = None,
                  mission_id: str | None = None,
                  recon_error_m: float | None = None) -> tuple[dict, list[dict]]:
    """Return (mission document, rejected candidates with reasons).

    ``recon_error_m`` is the measured calibration residual (worst independent
    check point).  Each target's declared error is the conservative sum of that
    residual and the spread of its own repeated observations, never below the
    task's 0.07 m bound.  A target whose estimate breaks the accuracy budget is
    rejected instead of being actioned on a doubtful coordinate (task C7).
    """
    created_at = created_at or utc_now()
    forbidden = forbidden or NO_FLY
    mission_id = mission_id or new_id("mission")
    zones = [Polygon("NFZ-1", tuple(Point(x, y) for x, y in forbidden))]
    calibration_residual = max(0.0, float(recon_error_m or 0.0))

    candidates = []
    rejected = []
    for obs in sorted(observations, key=lambda o: (round(o.y, 3), round(o.x, 3))):
        if obs.klass != "A":
            rejected.append({"object_id": obs.oid, "class": obs.klass,
                             "x": obs.x, "y": obs.y,
                             "reasons": ["class_not_permitted"]})
            continue
        error = max(RECON_ERROR, calibration_residual + obs.spread_m)
        # Temporary IDs; final IDs are assigned after filtering.
        target = Target("pending", "pending", obs.klass, Point(obs.x, obs.y), error)
        reasons = validate_target(target, zones)
        if reasons:
            rejected.append({"object_id": obs.oid, "class": obs.klass,
                             "x": obs.x, "y": obs.y,
                             "position_error_m": round(error, 4),
                             "reasons": reasons})
        else:
            candidates.append((obs, error))

    targets = []
    for i, (obs, error) in enumerate(candidates, 1):
        target_id = f"T{i:02d}"
        targets.append({
            "target_id": target_id,
            "action_id": f"{mission_id}-{target_id}",
            "klass": obs.klass,
            "point": {"x": round(obs.x, 4), "y": round(obs.y, 4)},
            "position_error_m": round(error, 4),
            "priority": i,
            "source_object_id": obs.oid,
            "source_frames": obs.frames,
            "observations": obs.n_obs,
            "spread_m": round(obs.spread_m, 4),
        })

    valid_until = last_frame_at + timedelta(seconds=MAP_TTL_SECONDS)
    payload = {
        "mission_id": mission_id,
        "coordinate_frame": "F",
        "units": "m",
        "created_at": iso(created_at),
        "valid_until": iso(valid_until),
        "calibration_version": calibration_version,
        "source_run_id": run_id,
        "field": {"width_m": FIELD_W, "height_m": FIELD_H},
        "targets": targets,
        "forbidden": [{"zone_id": "NFZ-1",
                       "vertices": [{"x": x, "y": y} for x, y in forbidden]}],
        "rejected": rejected,
        "note": "training/simulation mission; checksum is recalculated on write",
    }
    payload["checksum"] = checksum_of(payload)
    return payload, rejected


def apply_subset(mission: dict, plan_result: dict, budget_s: float) -> dict:
    """Return a mission document containing only the feasible subset.

    The planner already dropped targets with a recorded reason; this copies
    that decision into the mission file itself, so the executor never sees
    targets that do not fit the budget (task A4 / scenario C8).
    """
    skipped = [dict(item) for item in (plan_result.get("skipped") or [])]
    out = copy.deepcopy(mission)
    if skipped:
        dropped = {item["target_id"] for item in skipped}
        out["targets"] = [t for t in out["targets"] if t["target_id"] not in dropped]
    out["budget_s"] = round(float(budget_s), 1)
    out["partial"] = bool(skipped)
    out["skipped"] = skipped
    out.pop("checksum", None)
    out["checksum"] = checksum_of(out)
    return out
