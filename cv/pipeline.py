"""End-to-end CV pipeline: recon run -> observation map -> mission file.

The pipeline reads only the run directory produced by the simulator (frames,
local poses, calibration markers).  It never imports ``sim.field`` and never
reads ``ground_truth.json``.  The mission JSON file is the hand-off boundary to
the executor.
"""
from __future__ import annotations

import json
import shutil
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable

import cv2

from cv.calibration import Calibration, CalibrationError, calibrate, pixel_to_local
from cv.detect import detect_plots
from cv.merge import merge_detections
from cv.mission import build_mission, observation_map
from cv.planner import plan
from cv.validator import validate_mission
from shared.contracts import (
    iso, parse_iso, read_json, utc_now, write_json,
)
from shared.frames import apply_similarity
from shared.world import HOME, NO_FLY

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RUNS_DIR = PROJECT_ROOT / "runs"
MISSIONS_DIR = PROJECT_ROOT / "missions"


class PipelineError(RuntimeError):
    pass


@dataclass
class PipelineResult:
    run_id: str
    valid: bool
    errors: list[str]
    mission: dict | None
    mission_path: str | None
    plan: dict | None
    report: dict
    report_path: str
    observations: list = field(default_factory=list)
    calibration: dict | None = None
    rejected: list = field(default_factory=list)
    duration_s: float = 0.0


def _progress(cb: Callable[[str, float], None] | None, stage: str, fraction: float) -> None:
    if cb:
        try:
            cb(stage, fraction)
        except Exception:
            pass


def _frame_meta(run_dir: Path, frame_id: str) -> dict:
    return read_json(run_dir / "frames" / f"{frame_id}.json")


def _image(run_dir: Path, meta: dict):
    image = cv2.imread(str(run_dir / meta["image"]))
    if image is None:
        raise PipelineError(f"cannot read image {meta['image']}")
    return image


def run_pipeline(run_dir: str | Path, progress: Callable[[str, float], None] | None = None,
                 mission_dir: str | Path | None = None) -> PipelineResult:
    started = time.time()
    run_dir = Path(run_dir)
    mission_dir = Path(mission_dir) if mission_dir else MISSIONS_DIR
    mission_dir.mkdir(parents=True, exist_ok=True)

    _progress(progress, "загрузка разведданных", 0.02)
    run = read_json(run_dir / "run.json")
    run_id = str(run["run_id"])
    if run.get("failed"):
        raise PipelineError(f"recon run failed: {run.get('error')}")
    frame_ids = [f for f in run.get("frames", []) if f != "calib"]
    if not frame_ids:
        raise PipelineError("run contains no scan frames")

    calib_meta = _frame_meta(run_dir, "calib")
    calib_img = _image(run_dir, calib_meta)
    _progress(progress, "калибровка кадра", 0.10)
    try:
        calibration: Calibration = calibrate(calib_img, calib_meta["pose"])
    except CalibrationError as exc:
        raise PipelineError(str(exc)) from exc
    calib_version = f"calib_{run_id}"
    write_json(run_dir / "calibration.json", {
        "calibration_version": calib_version,
        "frame_id": "calib",
        **calibration.as_dict(),
    })

    all_detections = []
    detection_rows = []
    total = max(1, len(frame_ids))
    for i, fid in enumerate(frame_ids, 1):
        _progress(progress, f"детекция {fid}", 0.10 + 0.65 * i / total)
        meta = _frame_meta(run_dir, fid)
        img = _image(run_dir, meta)
        dets = detect_plots(img, fid)
        for d in dets:
            lx, ly = pixel_to_local(d.u, d.v, meta["pose"], calibration.f_px)
            d.x, d.y = apply_similarity(calibration.transform, lx, ly)
            all_detections.append(d)
            detection_rows.append({
                "frame_id": fid, "class": d.klass,
                "u": round(d.u, 2), "v": round(d.v, 2),
                "x": round(d.x, 4), "y": round(d.y, 4),
                "score": round(d.score, 4), "details": d.details,
            })

    _progress(progress, "объединение наблюдений", 0.80)
    observations = merge_detections(all_detections)
    write_json(run_dir / "detections.json", {
        "run_id": run_id,
        "calibration_version": calib_version,
        "detections": detection_rows,
    })

    last_capture = max((parse_iso(_frame_meta(run_dir, fid)["captured_at"])
                        for fid in frame_ids), default=utc_now())
    obs_map = observation_map(observations, run_id, calib_version,
                              calibration=calibration.as_dict(),
                              created_at=parse_iso(run.get("started_at", iso())))
    write_json(run_dir / "observation_map.json", obs_map)

    _progress(progress, "построение миссии", 0.88)
    mission, rejected = build_mission(observations, run_id, calib_version,
                                      last_frame_at=last_capture)
    mission_path = mission_dir / f"{mission['mission_id']}.json"
    # Keep an in-run copy for the report even if validation fails.
    write_json(run_dir / "mission_candidate.json", mission)
    write_json(mission_path, mission)

    planning = {"start": {"x": HOME[0], "y": HOME[1]}, "order": [], "targets": [],
                "route": [], "length_m": 0.0, "estimated_s": 0.0,
                "reserve_s": 0.0, "total_with_reserve_s": 0.0,
                "method": "none", "comparison": {}, "skipped": [],
                "expanded_forbidden": [], "valid": False}
    if mission["targets"]:
        planning = plan(HOME, mission["targets"], forbidden=NO_FLY)
        write_json(run_dir / "plan.json", planning)

    errors = validate_mission(mission)
    if not mission["targets"]:
        errors.append("no_targets_in_mission")
    if mission["targets"] and not planning.get("valid"):
        errors.append("planner_failed")
    if planning.get("skipped"):
        errors.append("planner_skipped_targets")
    errors = sorted(set(errors))
    valid = not errors

    latest = mission_dir / "latest_valid.json"
    if valid:
        shutil.copyfile(mission_path, latest)
    elif latest.exists():
        latest.unlink(missing_ok=True)

    counts = {k: sum(1 for o in observations if o.klass == k) for k in ("A", "B", "U")}
    report = {
        "run_id": run_id,
        "finished_at": iso(),
        "valid": valid,
        "validation_errors": errors,
        "calibration_version": calib_version,
        "calibration": calibration.as_dict(),
        "counts": {
            "scan_frames": len(frame_ids),
            "detections": len(detection_rows),
            "objects": len(observations),
            "A": counts["A"], "B": counts["B"], "U": counts["U"],
            "mission_targets": len(mission["targets"]),
            "rejected": len(rejected),
        },
        "observations": [o.as_dict() for o in observations],
        "rejected": rejected,
        "mission_id": mission["mission_id"],
        "mission_path": str(mission_path),
        "plan": {k: planning.get(k) for k in
                 ("order", "route", "length_m", "estimated_s", "reserve_s",
                  "total_with_reserve_s", "method", "comparison", "skipped", "valid")},
    }
    report_path = write_json(run_dir / "report.json", report)
    _progress(progress, "готово", 1.0)

    result = PipelineResult(
        run_id=run_id, valid=valid, errors=errors, mission=mission,
        mission_path=str(mission_path) if valid else None,
        plan=planning, report=report, report_path=str(report_path),
        observations=observations, calibration=calibration.as_dict(),
        rejected=rejected, duration_s=round(time.time() - started, 3),
    )
    return result


def run_pipeline_safe(run_dir: str | Path, progress=None) -> dict:
    """Thread-friendly wrapper returning JSON-ish data and an error string."""
    try:
        result = run_pipeline(run_dir, progress=progress)
        return {
            "ok": True,
            "result": result,
            "error": None,
            "traceback": None,
        }
    except Exception as exc:
        return {
            "ok": False,
            "result": None,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Run the CV pipeline on a recon run")
    parser.add_argument("run_dir")
    args = parser.parse_args()
    result = run_pipeline(args.run_dir, progress=lambda s, f: print(f"[{f:4.0%}] {s}"))
    print(json.dumps(result.report, indent=2, ensure_ascii=False))
    print("valid:", result.valid, "mission:", result.mission_path)


if __name__ == "__main__":
    main()
