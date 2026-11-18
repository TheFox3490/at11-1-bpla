"""Reconnaissance flight: calibrated snake scan with real frame captures.

The controller navigates in the recon drone's local frame ``L_recon``.  Only
local poses are written to the frame metadata.  The simulator uses the true
field frame to render the scene, exactly like a real drone would use its local
navigation solution while the world exists independently.
"""
from __future__ import annotations

import math
from pathlib import Path

from shared.contracts import file_sha256, iso, new_id, utc_now, write_json
from shared.frames import LocalFrame
from shared.world import (
    ALT_MAX, ALT_MIN, ALT_NOMINAL, CALIB_MARKERS, CHECK_MARKERS, FIELD_H, FIELD_W,
    HFOV, IMG_H, IMG_W, RECON_LIMIT_SECONDS, SPEED_LIMIT,
)
from sim.capture import FrameGrabber
from sim.drone import Drone

# The survey grid is defined in the field frame as a design input.  The
# controller receives the same points converted to L_recon and never uses F.
LANES_Y = (1.05, 2.20, 3.20)
XS = tuple(0.9 + 0.6 * i for i in range(8))
CALIB_POINT = (0.8, 0.8)


def scan_path_f() -> list[tuple[float, float]]:
    path: list[tuple[float, float]] = []
    for i, y in enumerate(LANES_Y):
        lane = XS if i % 2 == 0 else tuple(reversed(XS))
        path.extend((x, y) for x in lane)
    return path


class ReconRun:
    def __init__(self, drone: Drone, grabber: FrameGrabber, run_dir: str | Path,
                 frame: LocalFrame | None = None,
                 set_calibration=None):
        self.drone = drone
        self.grabber = grabber
        self.frame = frame or LocalFrame.from_spec("L_recon")
        self.set_calibration = set_calibration or (lambda on: None)
        self.run_dir = Path(run_dir)
        self.frames_dir = self.run_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)

        self.run_id = self.run_dir.name
        self.path_l = [self.frame.to_local(x, y) for x, y in scan_path_f()]
        self.calib_l = self.frame.to_local(*CALIB_POINT)

        self.phase = "to_calib"
        self.index = 0
        self.t = 0.0
        self.started_at = utc_now()
        self.ended_at = None
        self.error: str | None = None
        self.done = False

        self._settle = 0.0
        self._waiting_capture = False
        self._pending: dict | None = None
        self._pending_pose_f: dict | None = None
        self.frame_ids: list[str] = []
        self.capture_count = 0
        self.expected_captures = 1 + len(self.path_l)
        self.set_calibration(False)

        self.drone.reset()
        self.drone.speed_limit = SPEED_LIMIT
        self._goto_calib()

    # ------------------------------------------------------------------ #
    @property
    def status(self) -> str:
        if self.error:
            return f"ошибка: {self.error}"
        if self.done:
            return "разведка завершена"
        return {
            "to_calib": "полёт к калибровочной площадке",
            "capture_calib": "калибровочный кадр",
            "to_waypoint": "облёт поля",
            "capture": "съёмка кадра",
            "to_home": "возврат на точку старта",
            "land": "посадка",
        }.get(self.phase, self.phase)

    def _goto_calib(self) -> None:
        self.drone.goto(self.frame.to_field(*self.calib_l)[0],
                        self.frame.to_field(*self.calib_l)[1], ALT_NOMINAL)

    def _goto_waypoint(self) -> None:
        lx, ly = self.path_l[self.index]
        fx, fy = self.frame.to_field(lx, ly)
        self.drone.goto(fx, fy, ALT_NOMINAL)

    def _arm(self, frame_id: str) -> None:
        pose_f = {"x": self.drone.x, "y": self.drone.y,
                  "alt": self.drone.alt, "yaw": self.drone.yaw}
        pose_l = self.frame.pose_to_local(self.drone.x, self.drone.y, self.drone.yaw)
        path = self.frames_dir / f"{frame_id}.png"
        if not self.grabber.arm(pose_f, path):
            self._fail("камера занята")
            return
        self._pending_pose_f = pose_f
        self._pending = {
            "frame_id": frame_id,
            "run_id": self.run_id,
            "t_s": round(self.t, 3),
            "image": f"frames/{frame_id}.png",
            "pose_frame": self.frame.name,
            "pose": {"x": round(pose_l[0], 4), "y": round(pose_l[1], 4),
                     "alt": round(self.drone.alt, 4), "yaw": round(pose_l[2], 4)},
            "camera": {"hfov_deg": HFOV, "width": IMG_W, "height": IMG_H,
                       "mount": "nadir, fixed, no gimbal"},
            "calib_id": f"calib_{self.run_id}",
            "captured_at": iso(),
        }
        self._waiting_capture = True

    def _finish_capture(self) -> None:
        if self.grabber.error:
            self._fail(self.grabber.error)
            return
        if self._pending is None or self.grabber.last_path is None:
            self._fail("кадр не создан")
            return
        meta = self._pending
        meta["sha256"] = file_sha256(self.grabber.last_path)
        write_json(self.frames_dir / f"{meta['frame_id']}.json", meta)
        self.frame_ids.append(meta["frame_id"])
        self.capture_count += 1
        frame_id = meta["frame_id"]
        self._pending = None
        self._pending_pose_f = None
        self._waiting_capture = False

        if frame_id == "calib":
            self.set_calibration(False)
            self.index = 0
            self.phase = "to_waypoint"
            self._goto_waypoint()
            return
        self.index += 1
        if self.index < len(self.path_l):
            self.phase = "to_waypoint"
            self._goto_waypoint()
        else:
            self.phase = "to_home"
            self.drone.goto(self.frame.to_field(*self.frame.to_local(3.0, 2.0))[0],
                            self.frame.to_field(*self.frame.to_local(3.0, 2.0))[1],
                            ALT_NOMINAL)

    def _fail(self, message: str) -> None:
        self.set_calibration(False)
        self.error = message
        self.phase = "failed"
        self.done = True
        self.ended_at = utc_now()
        self._write_run(failed=True)

    def _write_run(self, failed: bool = False) -> None:
        run = {
            "run_id": self.run_id,
            "scenario": "recon",
            "generator": "sim/recon.py",
            "field": {"width_m": FIELD_W, "height_m": FIELD_H, "frame": "F"},
            "camera": {"hfov_deg": HFOV, "resolution": [IMG_W, IMG_H],
                       "mount": "nadir fixed", "alt_nominal_m": ALT_NOMINAL,
                       "alt_corridor_m": [ALT_MIN, ALT_MAX]},
            "motion": {"speed_m_s": SPEED_LIMIT,
                       "pattern": "snake, 3 lanes x 8 stations",
                       "lane_y_m": list(LANES_Y)},
            "local_frame": self.frame.name,
            "calibration": {
                "calib_id": f"calib_{self.run_id}",
                "markers": {k: {"x": v[0], "y": v[1]} for k, v in CALIB_MARKERS.items()},
                "check_points": {k: {"x": v[0], "y": v[1]} for k, v in CHECK_MARKERS.items()},
                "note": "markers and check points are visible in frames/calib.png",
            },
            "frames": self.frame_ids,
            "started_at": iso(self.started_at),
            "ended_at": iso(self.ended_at or utc_now()),
            "t_end_s": round(self.t, 3),
            "failed": failed,
            "error": self.error,
        }
        write_json(self.run_dir / "run.json", run)

    # ------------------------------------------------------------------ #
    def step(self, dt: float) -> None:
        if self.done:
            return
        self.t += dt
        if self.t > RECON_LIMIT_SECONDS:
            self._fail("превышен лимит времени разведки")
            return

        if self._waiting_capture:
            if self.grabber.busy:
                return
            self._finish_capture()
            return

        if self.phase in {"to_calib", "to_waypoint"}:
            arrived = self.drone.arrived and self.drone.alt_reached
            self.drone.update(dt)
            if arrived:
                self._settle += dt
                if self._settle >= 0.15:
                    self._settle = 0.0
                    if self.phase == "to_calib":
                        self.set_calibration(True)
                        self.phase = "capture_calib"
                        self._arm("calib")
                    else:
                        self.phase = "capture"
                        self._arm(f"f{self.index + 1:03d}")
            return

        if self.phase in {"to_home", "land"}:
            self.drone.update(dt)
            if self.phase == "to_home" and self.drone.arrived and self.drone.alt_reached:
                self.phase = "land"
                home = self.frame.to_local(3.0, 2.0)
                fx, fy = self.frame.to_field(*home)
                self.drone.goto(fx, fy, 0.05, landing=True)
            elif self.phase == "land" and self.drone.arrived and self.drone.alt <= 0.06:
                self.phase = "done"
                self.done = True
                self.ended_at = utc_now()
                self._write_run(failed=False)
            return

        if self.phase == "done":
            self.done = True

    def cancel(self) -> None:
        self.grabber.cancel()
        self.set_calibration(False)
        self.done = True
        self.error = self.error or "отменено пользователем"
        self.phase = "failed"
        self.ended_at = utc_now()
        self._write_run(failed=True)
