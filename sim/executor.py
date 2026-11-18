"""Mission executor: fly the mission file, stabilise, gate, LED, return, land.

The executor only receives a mission JSON file in field frame F.  It converts
the plan to its own navigation frame ``L_exec`` for flight, and re-validates the
file (checksum, TTL, classes, forbidden zones) before acting.  A durable action
registry makes repeated/restarted runs safe: an action confirmed earlier is
never repeated.
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
    distance_to_polygon,
)
from sim.drone import Drone

PROJECT_ROOT = Path(__file__).resolve().parent.parent
REGISTRY_PATH = PROJECT_ROOT / "state" / "action_registry.json"


class ExecutorRun:
    def __init__(self, drone: Drone, mission_path: str | Path, run_dir: str | Path,
                 frame: LocalFrame | None = None,
                 registry_path: str | Path | None = None,
                 journal: Journal | None = None):
        self.drone = drone
        self.mission_path = Path(mission_path)
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.frame = frame or LocalFrame.from_spec("L_exec")
        self.registry = ActionRegistry(registry_path or REGISTRY_PATH)
        self.journal = journal or Journal(self.run_dir / "journal.jsonl")
        self._own_journal = journal is None

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
        self.validation_errors = validate_mission(self.mission)
        if self.validation_errors:
            self._fail("миссия не прошла проверку: " + ", ".join(self.validation_errors))
            return
        self.mission_expiry = parse_iso(self.mission["valid_until"])
        try:
            self.plan = plan(HOME, self.mission["targets"], forbidden=None)
        except Exception as exc:
            self._fail(f"планировщик: {exc}")
            return
        if not self.plan or not self.plan.get("valid") or not self.plan.get("targets"):
            self._fail("планировщик не построил маршрут")
            return
        self.route_f = [(float(p["x"]), float(p["y"])) for p in self.plan["route"]]
        self.route_l = [self.frame.to_local(x, y) for x, y in self.route_f]
        for target in self.plan["targets"]:
            key = self._key(target["point"]["x"], target["point"]["y"])
            self.targets_by_point[key] = target
        self.journal.log("mission_loaded", mission_id=self.mission.get("mission_id"),
                         mission_path=str(self.mission_path), targets=len(self.plan["targets"]),
                         route_points=len(self.route_f), local_frame=self.frame.name)

    @staticmethod
    def _key(x: float, y: float) -> tuple[int, int]:
        return int(round(float(x) * 1000)), int(round(float(y) * 1000))

    def _fail(self, message: str) -> None:
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
        if self.registry.is_done(target["action_id"]):
            self.current_action["status"] = "duplicate_blocked"
            self.journal.log("action_blocked", target_id=target["target_id"],
                             action_id=target["action_id"], reason="already_actioned")
        else:
            self.journal.log("stabilize_start", target_id=target["target_id"],
                             action_id=target["action_id"],
                             point=target["point"], t_s=round(self.t, 3))
        self.stable = 0.0
        self.phase = "stabilize"

    def _gate(self, target: dict, distance: float) -> tuple[bool, str]:
        now = utc_now()
        map_valid = (not validate_mission(self.mission) if self.mission else False)
        if self.mission_expiry is not None and now > self.mission_expiry:
            map_valid = False
        tx, ty = target["point"]["x"], target["point"]["y"]
        target_forbidden = distance_to_polygon(tx, ty, _zone_vertices(self.mission)) <= FORBIDDEN_EXPANSION
        drone_forbidden = distance_to_polygon(self.drone.x, self.drone.y,
                                              _zone_vertices(self.mission)) <= FORBIDDEN_EXPANSION
        return operation_gate(
            map_valid=bool(map_valid),
            class_a=target.get("klass") == "A",
            position_fresh=True,
            altitude_m=self.drone.alt,
            stable_inside_area_seconds=self.stable,
            position_error_m=float(target.get("position_error_m", 1.0)),
            already_actioned=self.registry.is_done(target["action_id"]),
            distance_to_target_m=distance,
            target_in_forbidden=target_forbidden,
            drone_in_forbidden=drone_forbidden,
        )

    def _complete_action(self) -> None:
        target = self.current_target
        action = self.current_action
        if target is None or action is None:
            return
        self.registry.mark(target["action_id"], "confirmed",
                           target_id=target["target_id"],
                           point=target["point"], at=iso())
        action["status"] = "confirmed"
        action["operation_seconds"] = OPERATION_SECONDS
        self.actions.append(action)
        self.journal.log("action_confirmed", target_id=target["target_id"],
                         action_id=target["action_id"], point=target["point"],
                         t_s=round(self.t, 3))
        self.current_target = None
        self.current_action = None
        self.route_index += 1
        self._goto_route()

    # ------------------------------------------------------------------ #
    def step(self, dt: float) -> None:
        if self.done:
            return
        self.t += dt
        if self.t > EXEC_LIMIT_SECONDS:
            self._fail("превышен лимит времени исполнителя")
            return

        if self.phase == "led":
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
                self.journal.log("action_blocked", target_id=target["target_id"],
                                 action_id=target["action_id"], reason=reason,
                                 distance_m=round(d, 4), t_s=round(self.t, 3))
                action = self.current_action or {}
                action.update({"status": "blocked", "reason": reason,
                               "distance_m": round(d, 4)})
                self.actions.append(action)
                if reason != "duplicate_action_blocked":
                    self.registry.mark(target["action_id"], "skipped", reason=reason)
                self.current_target = None
                self.current_action = None
                self.route_index += 1
                self._goto_route()
                return
            self.phase = "led"
            self.led_time = 0.0
            self.drone.rig.set_led(True)
            self.registry.mark(target["action_id"], "uncertain",
                               target_id=target["target_id"], reason="led_in_progress")
            if self.current_action:
                self.current_action["status"] = "led_on"
                self.current_action["distance_m"] = round(d, 4)
                self.current_action["stable_seconds"] = round(self.stable, 3)
            self.journal.log("operation_led", target_id=target["target_id"],
                             action_id=target["action_id"],
                             point=target["point"], distance_m=round(d, 4),
                             t_s=round(self.t, 3))
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

    # ------------------------------------------------------------------ #
    def _write_summary(self) -> None:
        summary = {
            "run_id": self.run_dir.name,
            "mission_id": (self.mission or {}).get("mission_id"),
            "mission_path": str(self.mission_path),
            "started_at": iso(self.started_at),
            "ended_at": iso(self.ended_at or utc_now()),
            "t_end_s": round(self.t, 3),
            "status": "failed" if self.failed else ("done" if self.done else "running"),
            "error": self.error,
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
        self.drone.rig.set_led(False)
        self.journal.log("mission_cancelled", t_s=round(self.t, 3))
        self._fail("отменено пользователем")


def _zone_vertices(mission: dict | None) -> list[tuple[float, float]]:
    if mission:
        zones = mission.get("forbidden") or []
        if zones:
            return [(float(v["x"]), float(v["y"])) for v in zones[0]["vertices"]]
    from shared.world import NO_FLY
    return NO_FLY
