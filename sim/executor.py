"""Mission executor: fly the mission file, stabilise, gate, LED, return, land.

The executor only receives a mission JSON file in field frame F.  It converts
the plan to its own navigation frame ``L_exec`` for flight, and re-validates the
file (checksum, TTL, classes, forbidden zones) before acting.  A durable action
registry makes repeated/restarted runs safe: an action confirmed earlier is
never repeated.

Safety model (task A4, scenarios C3/C4/C5/C6):
* the LED can only start after the full gate passed and 1 s of stable hold;
* while the LED is on the gate is re-checked on every step, so losing the
  map, the navigation fix, the link or the target area switches it off at once;
* any unrecoverable condition aborts the mission: the executor stops acting,
  records the reason and either returns home or lands in place (when navigation
  itself is unreliable);
* aborting is a *successful safe ending*, not a crash: every path lands and
  writes a summary.  ``failed`` is reserved for a rejected mission file.
"""
from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

from cv.planner import plan
from cv.validator import operation_gate, validate_mission
from shared.contracts import iso, parse_iso, read_json, utc_now, write_json
from shared.frames import LocalFrame
from shared.journal import ActionRegistry, Journal
from shared.world import (
    ALT_MIN, ALT_MAX, ALT_NOMINAL, EXEC_LIMIT_SECONDS, FORBIDDEN_EXPANSION,
    HOME, OPERATION_SECONDS, SPEED_LIMIT, STABILIZE_SECONDS, TARGET_RADIUS,
    distance_to_polygon, expand_polygon, point_in_poly, segment_intersects_polygon,
)
from sim.drone import Drone

PROJECT_ROOT = Path(__file__).resolve().parent.parent
REGISTRY_PATH = PROJECT_ROOT / "state" / "action_registry.json"
RUNS_DIR = PROJECT_ROOT / "runs"

LOCALIZATION_TIMEOUT_S = 0.3     # task C6: no LED with a fix older than 300 ms
LOCALIZATION_WARN_S = 0.15


def known_calibration_versions(root: Path | None = None) -> set[str]:
    """Calibration versions this machine actually produced (task C2).

    A mission that names a calibration we never ran is not trusted: the map
    could come from a different camera, altitude or field frame.
    """
    root = Path(root or PROJECT_ROOT)
    versions: set[str] = set()
    for path in (root / "runs").glob("*/calibration.json"):
        try:
            version = read_json(path).get("calibration_version")
        except (OSError, ValueError):
            continue
        if version:
            versions.add(str(version))
    return versions


def _is_known_calibration(version: str, known: set[str]) -> bool:
    # Manual training missions never come from the CV pipeline; they are
    # labelled as such and are allowed to run as a training aid only.
    return version in known or version.startswith("manual_")


class ExecutorRun:
    def __init__(self, drone: Drone, mission_path: str | Path, run_dir: str | Path,
                 frame: LocalFrame | None = None,
                 registry_path: str | Path | None = None,
                 journal: Journal | None = None,
                 faults: dict | None = None,
                 known_calibrations: set[str] | None = None):
        self.drone = drone
        self.mission_path = Path(mission_path)
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.frame = frame or LocalFrame.from_spec("L_exec")
        self.registry = ActionRegistry(registry_path or REGISTRY_PATH)
        self.journal = journal or Journal(self.run_dir / "journal.jsonl")
        self._own_journal = journal is None
        # Fault injection for the acceptance bench (task B, C6).  Values are
        # mission seconds after which the corresponding subsystem "fails".
        self.faults = dict(faults or {})
        self.known_calibrations = (known_calibrations
                                   if known_calibrations is not None
                                   else known_calibration_versions())

        self.phase = "load"
        self.t = 0.0
        self.started_at = utc_now()
        self.ended_at = None
        self.done = False
        self.failed = False
        self.error: str | None = None
        self.actions: list[dict] = []
        self.route_index = 0
        self.route_l: list[tuple[float, float]] = []
        self.route_f: list[tuple[float, float]] = []
        self.targets_by_point: dict[tuple[int, int], dict] = {}
        self.current_target: dict | None = None
        self.current_action: dict | None = None
        self.stable = 0.0
        self.led_time = 0.0
        self._arrival_handled = False

        self.mission: dict | None = None
        self.plan: dict | None = None
        self.validation_errors: list[str] = []
        self.mission_expiry: datetime | None = None
        self.budget_s: float | None = None
        self.reserve_s: float = 0.0

        # navigation / link model
        self.last_fix_t = 0.0
        self.localization_age_s = 0.0
        self.localization_age_max = 0.0
        self.localization_lost_at: float | None = None
        self.link_alive = True
        self.link_lost_at: float | None = None

        # abort bookkeeping
        self.aborted = False
        self.abort_reason: str | None = None
        self.abort_mode: str | None = None
        self.abort_point: tuple[float, float] | None = None
        self.led_interruptions: list[dict] = []
        self.uncertain_actions: list[str] = []

        self._load()
        if not self.failed:
            self.drone.reset()
            self.drone.speed_limit = SPEED_LIMIT
            self._goto_route()

    # ------------------------------------------------------------------ #
    def _load(self) -> None:
        try:
            self.mission = read_json(self.mission_path)
        except (OSError, ValueError) as exc:
            self._fail(f"не удалось прочитать миссию: {exc}")
            return
        self.validation_errors = validate_mission(
            self.mission, known_calibrations=self.known_calibrations)
        if self.validation_errors:
            self._fail("миссия не прошла проверку: " + ", ".join(self.validation_errors))
            return
        self.mission_expiry = parse_iso(self.mission["valid_until"])
        if self.mission.get("budget_s") is not None:
            self.budget_s = float(self.mission["budget_s"])
        try:
            self.plan = plan(HOME, self.mission["targets"], forbidden=None)
        except Exception as exc:
            self._fail(f"планировщик: {exc}")
            return
        if not self.plan or not self.plan.get("valid") or not self.plan.get("targets"):
            self._fail("планировщик не построил маршрут")
            return
        self.reserve_s = float(self.plan.get("reserve_s") or 0.0)
        self.route_f = [(float(p["x"]), float(p["y"])) for p in self.plan["route"]]
        if not self._route_is_safe():
            self._fail("маршрут пересекает расширенную запретную зону")
            return
        self.route_l = [self.frame.to_local(x, y) for x, y in self.route_f]
        for target in self.plan["targets"]:
            key = self._key(target["point"]["x"], target["point"]["y"])
            self.targets_by_point[key] = target
        # C5: an action left "uncertain" by a crash must be surfaced before
        # the flight, not silently retried and not silently skipped.
        for target in self.plan["targets"]:
            entry = self.registry.get(target["action_id"]) or {}
            if entry.get("status") == "uncertain":
                self.uncertain_actions.append(target["action_id"])
        self.journal.log("mission_loaded", mission_id=self.mission.get("mission_id"),
                         mission_path=str(self.mission_path), targets=len(self.plan["targets"]),
                         route_points=len(self.route_f), local_frame=self.frame.name,
                         budget_s=self.budget_s, reserve_s=round(self.reserve_s, 1),
                         uncertain_actions=self.uncertain_actions)

    def _route_is_safe(self) -> bool:
        """Defence in depth: the file may have been edited after planning."""
        expanded = expand_polygon(_zone_vertices(self.mission), FORBIDDEN_EXPANSION)
        for a, b in zip(self.route_f, self.route_f[1:]):
            if segment_intersects_polygon(a, b, expanded):
                return False
        return True

    @staticmethod
    def _key(x: float, y: float) -> tuple[int, int]:
        return int(round(float(x) * 1000)), int(round(float(y) * 1000))

    def _fail(self, message: str) -> None:
        """Mission file rejected before or instead of flight (no take-off)."""
        self.error = message
        self.failed = True
        self.done = True
        self.phase = "failed"
        self.ended_at = utc_now()
        try:
            self.journal.log("mission_rejected", error=message,
                             errors=self.validation_errors)
        except Exception:
            pass
        self._write_summary()

    # ------------------------------------------------------------------ #
    def _budget_deadline(self) -> float | None:
        if self.budget_s is None:
            return None
        return max(1.0, self.budget_s - self.reserve_s)

    def _map_valid(self) -> bool:
        if self.mission is None:
            return False
        if self.mission_expiry is not None and utc_now() > self.mission_expiry:
            return False
        return not validate_mission(self.mission, check_checksum=False)

    def _localization_fresh(self) -> bool:
        return self.localization_age_s <= LOCALIZATION_TIMEOUT_S

    def _update_navigation(self) -> None:
        """Simulated navigation solution and control link (task C6)."""
        lost_after = self.faults.get("localization_lost_after")
        if lost_after is not None and self.t >= float(lost_after):
            if self.localization_lost_at is None:
                self.localization_lost_at = self.t
                self.journal.log("localization_lost", t_s=round(self.t, 3),
                                 reason="injected_navigation_failure")
        else:
            self.last_fix_t = self.t
        self.localization_age_s = max(0.0, self.t - self.last_fix_t)
        self.localization_age_max = max(self.localization_age_max, self.localization_age_s)

        lost_at = self.faults.get("link_lost_after")
        link_ok = not (lost_at is not None and self.t >= float(lost_at))
        if not link_ok and self.link_alive:
            self.link_alive = False
            self.link_lost_at = self.t
            self.journal.log("link_lost", t_s=round(self.t, 3),
                             reason="injected_control_link_failure")

    # ------------------------------------------------------------------ #
    def _goto_route(self) -> None:
        if self.route_index >= len(self.route_l):
            self.phase = "to_home"
            fx, fy = HOME
            self.drone.goto(fx, fy, ALT_NOMINAL)
            self._arrival_handled = False
            return
        fx, fy = self.route_f[self.route_index]
        self.drone.goto(fx, fy, ALT_NOMINAL)
        self.phase = "cruise"
        self._arrival_handled = False

    def _target_at_current_point(self) -> dict | None:
        if self.route_index >= len(self.route_f):
            return None
        x, y = self.route_f[self.route_index]
        return self.targets_by_point.get(self._key(x, y))

    def _start_land(self) -> None:
        self.phase = "land"
        fx, fy = HOME
        self.drone.goto(fx, fy, 0.05, landing=True)
        self.journal.log("landing_started", t_s=round(self.t, 2),
                         x=round(self.drone.x, 3), y=round(self.drone.y, 3))

    # ------------------------------------------------------------------ #
    def _begin_stabilize(self, target: dict) -> None:
        self.current_target = target
        self.current_action = {
            "target_id": target["target_id"],
            "action_id": target["action_id"],
            "class": target.get("klass", "A"),
            "point_field": dict(target["point"]),
            "point_local": dict(zip(("x", "y"), self.frame.to_local(
                target["point"]["x"], target["point"]["y"]))),
            "status": "stabilizing",
        }
        entry = self.registry.get(target["action_id"]) or {}
        if self.registry.is_done(target["action_id"]):
            previous = entry.get("status", "confirmed")
            self.current_action["status"] = "duplicate_blocked"
            self.current_action["reason"] = (
                "uncertain_pending_verification" if previous == "uncertain"
                else "already_actioned")
            self.journal.log("action_blocked", target_id=target["target_id"],
                             action_id=target["action_id"],
                             reason=self.current_action["reason"],
                             previous_status=previous,
                             position_age_s=round(self.localization_age_s, 3))
        else:
            self.journal.log("stabilize_start", target_id=target["target_id"],
                             action_id=target["action_id"],
                             point=target["point"], t_s=round(self.t, 3),
                             position_age_s=round(self.localization_age_s, 3),
                             position_error_m=float(target.get("position_error_m", 0.0)))
        self.stable = 0.0
        self.phase = "stabilize"

    def _gate(self, target: dict, distance: float) -> tuple[bool, str]:
        tx, ty = target["point"]["x"], target["point"]["y"]
        target_forbidden = distance_to_polygon(tx, ty, _zone_vertices(self.mission)) <= FORBIDDEN_EXPANSION
        drone_forbidden = distance_to_polygon(self.drone.x, self.drone.y,
                                              _zone_vertices(self.mission)) <= FORBIDDEN_EXPANSION
        return operation_gate(
            map_valid=bool(self._map_valid()),
            class_a=target.get("klass") == "A",
            position_fresh=self._localization_fresh(),
            link_ok=self.link_alive,
            altitude_m=self.drone.alt,
            stable_inside_area_seconds=self.stable,
            position_error_m=float(target.get("position_error_m", 1.0)),
            already_actioned=self.registry.is_done(target["action_id"]),
            distance_to_target_m=distance,
            target_in_forbidden=target_forbidden,
            drone_in_forbidden=drone_forbidden,
        )

    def _led_guard(self) -> tuple[bool, str]:
        """Conditions that must hold during the whole 2 s operation (A4)."""
        if not self._localization_fresh():
            return False, "stale_localization"
        if not self.link_alive:
            return False, "control_link_lost"
        if not self._map_valid():
            return False, "invalid_map"
        deadline = self._budget_deadline()
        if deadline is not None and self.t > deadline:
            return False, "budget_exhausted_with_reserve"
        target = self.current_target
        if target is None:
            return False, "no_target"
        tx, ty = target["point"]["x"], target["point"]["y"]
        if math.hypot(self.drone.x - tx, self.drone.y - ty) > TARGET_RADIUS:
            return False, "outside_target_radius"
        if not (ALT_MIN <= self.drone.alt <= ALT_MAX):
            return False, "altitude_out_of_range"
        if point_in_poly(self.drone.x, self.drone.y, _zone_vertices(self.mission)):
            return False, "forbidden_zone"
        return True, "ok"

    # ------------------------------------------------------------------ #
    def _abort(self, reason: str, mode: str) -> None:
        """Stop acting and end safely: return home or land in place (C3/C6)."""
        if self.done or self.aborted:
            return
        self.aborted = True
        self.abort_reason = reason
        self.abort_mode = mode
        self.abort_point = (self.drone.x, self.drone.y)
        self.drone.rig.set_led(False)
        self.led_time = 0.0
        action = self.current_action
        if action is not None:
            action["status"] = "aborted"
            action["reason"] = reason
            action["position_age_s"] = round(self.localization_age_s, 3)
            if action.get("action_id"):
                self.registry.mark(action["action_id"], "aborted", reason=reason)
            self.actions.append(action)
        self.current_action = None
        self.current_target = None
        self.journal.log("mission_aborted", reason=reason, mode=mode,
                         t_s=round(self.t, 3), x=round(self.drone.x, 3),
                         y=round(self.drone.y, 3), alt=round(self.drone.alt, 3),
                         position_age_s=round(self.localization_age_s, 3),
                         budget_s=self.budget_s,
                         completed=sum(1 for a in self.actions
                                       if a.get("status") == "confirmed"))
        if mode == "land":
            self.phase = "emergency_land"
            self.drone.goto(self.drone.x, self.drone.y, 0.05, landing=True)
            self.journal.log("emergency_land_started", t_s=round(self.t, 3),
                             x=round(self.drone.x, 3), y=round(self.drone.y, 3))
        else:
            self.phase = "abort_home"
            self.drone.goto(*HOME, ALT_NOMINAL)
        self._arrival_handled = False

    def _finish_abort(self) -> None:
        self.phase = "aborted"
        self.done = True
        self.ended_at = utc_now()
        self.journal.log("mission_safely_ended", reason=self.abort_reason,
                         mode=self.abort_mode, t_s=round(self.t, 3),
                         x=round(self.drone.x, 3), y=round(self.drone.y, 3),
                         alt=round(self.drone.alt, 3),
                         completed=sum(1 for a in self.actions
                                       if a.get("status") == "confirmed"))
        self._write_summary()

    def _step_abort(self, dt: float) -> None:
        if self.phase in {"abort_home", "emergency_land"}:
            self.drone.update(dt)
            if self.phase == "abort_home":
                if self.drone.arrived and self.drone.alt_reached and not self._arrival_handled:
                    self._arrival_handled = True
                    self.journal.log("abort_home_arrived", t_s=round(self.t, 3),
                                     x=round(self.drone.x, 3), y=round(self.drone.y, 3))
                    self.phase = "land"
                    self.drone.goto(HOME[0], HOME[1], 0.05, landing=True)
                return
            if self.drone.alt <= 0.06:
                self._finish_abort()
            return
        if self.phase == "land":
            self.drone.update(dt)
            if self.drone.arrived and self.drone.alt <= 0.06:
                self._finish_abort()

    # ------------------------------------------------------------------ #
    def _complete_action(self) -> None:
        target = self.current_target
        action = self.current_action
        if target is None or action is None:
            return
        self.registry.mark(target["action_id"], "confirmed",
                           target_id=target["target_id"],
                           point=target["point"], at=iso(),
                           position_error_m=float(target.get("position_error_m", 0.0)))
        action["status"] = "confirmed"
        action["operation_seconds"] = OPERATION_SECONDS
        action["position_age_s"] = round(self.localization_age_s, 3)
        self.actions.append(action)
        self.journal.log("led_confirmed", target_id=target["target_id"],
                         action_id=target["action_id"], point=target["point"],
                         t_s=round(self.t, 3),
                         position_age_s=round(self.localization_age_s, 3))
        self.current_target = None
        self.current_action = None
        self.route_index += 1
        self._goto_route()

    # ------------------------------------------------------------------ #
    def step(self, dt: float) -> None:
        if self.done:
            return
        self.t += dt
        self._update_navigation()

        if self.aborted:
            self._step_abort(dt)
            return

        if self.t > EXEC_LIMIT_SECONDS:
            self._abort("executor_time_limit", "home")
            self._step_abort(dt)
            return
        deadline = self._budget_deadline()
        if deadline is not None and self.t > deadline:
            self._abort("budget_exhausted_with_reserve", "home")
            self._step_abort(dt)
            return
        if not self.link_alive:
            self._abort("control_link_lost", "home")
            self._step_abort(dt)
            return
        if not self._localization_fresh():
            self._abort("stale_localization", "land")
            self._step_abort(dt)
            return
        if not self._map_valid():
            self._abort("map_expired_or_invalid", "home")
            self._step_abort(dt)
            return

        if self.phase in {"cruise", "stabilize", "led", "to_home"}:
            if point_in_poly(self.drone.x, self.drone.y, _zone_vertices(self.mission)):
                self._abort("forbidden_zone_violation", "land")
                self._step_abort(dt)
                return

        if self.phase == "led":
            ok, reason = self._led_guard()
            if not ok:
                self.drone.rig.set_led(False)
                self.led_interruptions.append({"reason": reason,
                                               "t_s": round(self.t, 3)})
                self.journal.log("led_interrupted", reason=reason,
                                 t_s=round(self.t, 3),
                                 position_age_s=round(self.localization_age_s, 3),
                                 led_seconds=round(self.led_time, 3))
                if reason in {"stale_localization", "forbidden_zone"}:
                    self._abort(reason, "land")
                elif reason in {"invalid_map", "budget_exhausted_with_reserve"}:
                    self._abort(reason, "home")
                elif reason == "control_link_lost":
                    self._abort(reason, "home")
                else:
                    self._skip_current(f"operation_interrupted:{reason}")
                self.led_time = 0.0
                if self.aborted:
                    self._step_abort(dt)
                return
            self.led_time += dt
            self.drone.update(dt)
            if self.led_time >= OPERATION_SECONDS:
                self.drone.rig.set_led(False)
                self._complete_action()
            return

        if self.phase in {"cruise", "to_home"}:
            self.drone.update(dt)
            if self.drone.arrived and self.drone.alt_reached and not self._arrival_handled:
                self._arrival_handled = True
                self.journal.log("waypoint", t_s=round(self.t, 3),
                                 index=self.route_index,
                                 x=round(self.drone.x, 3), y=round(self.drone.y, 3))
                if self.phase == "to_home":
                    self._start_land()
                    return
                target = self._target_at_current_point()
                if target is None:
                    self.route_index += 1
                    self._goto_route()
                else:
                    self._begin_stabilize(target)
            return

        if self.phase == "stabilize":
            self.drone.update(dt)
            target = self.current_target
            if target is None:
                self._fail("internal: no current target")
                return
            tx, ty = target["point"]["x"], target["point"]["y"]
            d = math.hypot(self.drone.x - tx, self.drone.y - ty)
            inside = d <= TARGET_RADIUS and ALT_MIN <= self.drone.alt <= ALT_MAX
            self.stable = self.stable + dt if inside else 0.0
            if self.stable < STABILIZE_SECONDS:
                return
            permitted, reason = self._gate(target, d)
            if not permitted:
                if reason == "duplicate_action_blocked":
                    entry = self.registry.get(target["action_id"]) or {}
                    if entry.get("status") == "uncertain":
                        # C5: a crashed operation is not silently repeated and
                        # not silently dropped: it is flagged for verification.
                        reason = "uncertain_pending_verification"
                self.journal.log("action_blocked", target_id=target["target_id"],
                                 action_id=target["action_id"], reason=reason,
                                 distance_m=round(d, 4), t_s=round(self.t, 3),
                                 position_age_s=round(self.localization_age_s, 3))
                action = self.current_action or {}
                action.update({"status": "blocked", "reason": reason,
                               "distance_m": round(d, 4),
                               "position_age_s": round(self.localization_age_s, 3)})
                self.actions.append(action)
                if reason not in {"duplicate_action_blocked",
                                  "uncertain_pending_verification"}:
                    self.registry.mark(target["action_id"], "skipped", reason=reason)
                self.current_target = None
                self.current_action = None
                self.route_index += 1
                self._goto_route()
                return
            self.phase = "led"
            self.led_time = 0.0
            self.drone.rig.set_led(True)
            # The registry entry becomes "uncertain" for the duration of the
            # operation: a crash right now must not lead to a repeat.
            self.registry.mark(target["action_id"], "uncertain",
                               target_id=target["target_id"], reason="led_in_progress")
            if self.current_action:
                self.current_action["status"] = "led_on"
                self.current_action["distance_m"] = round(d, 4)
                self.current_action["stable_seconds"] = round(self.stable, 3)
                self.current_action["position_age_s"] = round(self.localization_age_s, 3)
            self.journal.log("led_command", target_id=target["target_id"],
                             action_id=target["action_id"],
                             point=target["point"], distance_m=round(d, 4),
                             t_s=round(self.t, 3),
                             position_age_s=round(self.localization_age_s, 3),
                             position_error_m=float(target.get("position_error_m", 0.0)))
            return

        if self.phase == "land":
            self.drone.update(dt)
            if self.drone.arrived and self.drone.alt <= 0.06:
                self.phase = "done"
                self.done = True
                self.ended_at = utc_now()
                self.journal.log("mission_done", t_s=round(self.t, 3),
                                 actions=len(self.actions))
                self._write_summary()
            return

    def _skip_current(self, reason: str) -> None:
        target = self.current_target
        action = self.current_action or {}
        if target is not None:
            self.journal.log("action_skipped", target_id=target["target_id"],
                             action_id=target["action_id"], reason=reason,
                             t_s=round(self.t, 3))
            self.registry.mark(target["action_id"], "interrupted", reason=reason)
        action.update({"status": "interrupted", "reason": reason})
        self.actions.append(action)
        self.current_target = None
        self.current_action = None
        self.stable = 0.0
        self.route_index += 1
        self._goto_route()

    # ------------------------------------------------------------------ #
    def _status_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for action in self.actions:
            key = str(action.get("status", "unknown"))
            counts[key] = counts.get(key, 0) + 1
        return counts

    def _write_summary(self) -> None:
        summary = {
            "run_id": self.run_dir.name,
            "mission_id": (self.mission or {}).get("mission_id"),
            "mission_path": str(self.mission_path),
            "started_at": iso(self.started_at),
            "ended_at": iso(self.ended_at or utc_now()),
            "t_end_s": round(self.t, 3),
            "status": ("failed" if self.failed
                       else "aborted" if self.aborted
                       else "done" if self.done else "running"),
            "error": self.error,
            "aborted": self.aborted,
            "abort_reason": self.abort_reason,
            "abort_mode": self.abort_mode,
            "abort_point": ({"x": round(self.abort_point[0], 4),
                             "y": round(self.abort_point[1], 4)}
                            if self.abort_point else None),
            "budget_s": self.budget_s,
            "reserve_s": round(self.reserve_s, 2),
            "localization_age_max_s": round(self.localization_age_max, 3),
            "link_lost_at_s": (round(self.link_lost_at, 3)
                               if self.link_lost_at is not None else None),
            "led_interruptions": self.led_interruptions,
            "uncertain_actions": self.uncertain_actions,
            "status_counts": self._status_counts(),
            "validation_errors": self.validation_errors,
            "local_frame": self.frame.name,
            "route": [{"x": round(x, 4), "y": round(y, 4)} for x, y in self.route_f],
            "actions": self.actions,
            "journal": str(self.run_dir / "journal.jsonl"),
        }
        write_json(self.run_dir / "summary.json", summary)

    def cancel(self) -> None:
        if self.done:
            return
        self._abort("cancelled_by_operator", "land")
        self._finish_abort()


def _zone_vertices(mission: dict | None) -> list[tuple[float, float]]:
    if mission:
        zones = mission.get("forbidden") or []
        if zones:
            return [(float(v["x"]), float(v["y"])) for v in zones[0]["vertices"]]
    from shared.world import NO_FLY
    return NO_FLY
