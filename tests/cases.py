"""Acceptance scenarios C1-C8 for the AT11-2 bench.

Each case is a generator: it yields ``None`` whenever it needs another rendered
frame and returns a :class:`CaseResult`.  The same code drives the in-app
overlay (live 3D view) and the ``--tests`` CLI, so what the jury sees and what
the protocol records are produced by one implementation.
"""
from __future__ import annotations

import math
import time
import traceback
from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path

from cv.calibration import MAX_CHECK_ERROR_MM, CalibrationError, calibrate
from cv.detect import Detection
from cv.merge import Observation, merge_detections
from cv.mission import apply_subset, build_mission
from cv.planner import plan
from cv.validator import validate_mission
from shared.contracts import checksum_of, iso, new_id, utc_now, write_json
from shared.world import (
    HOME, NO_FLY, expand_polygon, segment_intersects_polygon,
)
from sim.executor import ExecutorRun

from .report import MapCanvas, collect_events, draw_timeline

PASS = "PASS"
FAIL = "FAIL"


class CaseFailure(RuntimeError):
    pass


@dataclass
class CaseResult:
    cid: str
    title: str
    expected: str
    received: str
    passed: bool
    duration_s: float
    details: list[str] = field(default_factory=list)
    proof: dict[str, str] = field(default_factory=dict)


@dataclass
class BenchCase:
    cid: str
    title: str
    expected: str
    run: object
    flights: bool = False


# --------------------------------------------------------------------------- #
# context
# --------------------------------------------------------------------------- #
def build_bench_mission(points, *, version: str = "calib_bench",
                        units: str = "m", ttl_s: float = 600.0,
                        budget_s: float | None = None,
                        mission_id: str | None = None,
                        created_at=None) -> dict:
    """A mission document with the same shape the CV pipeline produces."""
    created = created_at or utc_now()
    mission_id = mission_id or new_id("bench_mission")
    targets = []
    for index, (x, y) in enumerate(points, 1):
        target_id = f"T{index:02d}"
        targets.append({
            "target_id": target_id,
            "action_id": f"{mission_id}-{target_id}",
            "klass": "A",
            "point": {"x": float(x), "y": float(y)},
            "position_error_m": 0.07,
            "priority": index,
        })
    payload = {
        "mission_id": mission_id,
        "coordinate_frame": "F",
        "units": units,
        "created_at": iso(created),
        "valid_until": iso(created + timedelta(seconds=ttl_s)),
        "calibration_version": version,
        "targets": targets,
        "forbidden": [{"zone_id": "NFZ-1",
                       "vertices": [{"x": x, "y": y} for x, y in NO_FLY]}],
        "rejected": [],
        "note": "acceptance bench mission",
    }
    if budget_s is not None:
        payload["budget_s"] = float(budget_s)
        payload["partial"] = False
        payload["skipped"] = []
    payload["checksum"] = checksum_of(payload)
    return payload


class BenchContext:
    def __init__(self, app, bench_dir: str | Path):
        self.app = app
        self.dir = Path(bench_dir)
        (self.dir / "shots").mkdir(parents=True, exist_ok=True)
        (self.dir / "exec").mkdir(parents=True, exist_ok=True)
        self.registry = self.dir / "registry.json"
        self.calibration_version = "calib_bench"
        self.saved_layout = None
        self._mission: dict | None = None
        self._mission_path: str | None = None
        self._report: dict | None = None
        self._work = None
        self.c1_run: ExecutorRun | None = None
        self._write_bench_calibration()
        # The protocol must be reproducible: run on the task's base layout and
        # put the operator's field back afterwards.
        from sim import scenario
        self.saved_layout = scenario.copy_objects(self.app.layout)
        self.saved_starts = dict(self.app.starts)
        self.app.apply_layout(scenario.default_objects(), persist=False,
                              starts=scenario.default_starts())

    def close(self) -> None:
        if self.saved_layout is not None:
            self.app.apply_layout(self.saved_layout, persist=False, starts=self.saved_starts)
            self.saved_layout = None

    # -- infrastructure -------------------------------------------------- #
    def _write_bench_calibration(self) -> None:
        path = Path(self.app.registry_path).parent.parent / "runs" / "bench_calibration" / "calibration.json"
        if not path.exists():
            write_json(path, {
                "calibration_version": self.calibration_version,
                "note": "synthetic calibration used by acceptance scenarios",
                "f_px": 457.0,
                "worst_check_mm": 0.1,
            })

    def log(self, text: str) -> None:
        print(f"[bench] {text}")

    def frames(self, count: int = 1):
        for _ in range(count):
            yield

    def wait_until(self, predicate, timeout_s: float = 240.0, label: str = ""):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if predicate():
                return True
            yield
        raise CaseFailure(f"таймаут ожидания: {label or 'условие'}")

    def reset_drone(self, role: str = "executor") -> None:
        self.app.reset_drone(role)
        self.app.scene.clear_trail()

    def shot(self, name: str) -> str:
        path = self.dir / "shots" / f"{name}.png"
        self.app.screenshot(path)
        return str(path)

    # -- missions -------------------------------------------------------- #
    def make_mission(self, points, *, version: str | None = None,
                     units: str = "m", ttl_s: float = 600.0,
                     budget_s: float | None = None,
                     mission_id: str | None = None,
                     created_at=None) -> dict:
        return build_bench_mission(
            points, version=version or self.calibration_version, units=units,
            ttl_s=ttl_s, budget_s=budget_s, mission_id=mission_id,
            created_at=created_at)

    def write_mission(self, mission: dict, name: str) -> str:
        path = self.dir / f"{name}.json"
        write_json(path, mission)
        return str(path)

    def ensure_mission(self):
        """Run the real recon + CV once and reuse the mission."""
        if self._mission is not None:
            return self._mission
        app = self.app
        app.set_mode("work")
        work = app.modes["work"]
        work.reset_work()
        app.set_speed(5.0)
        self.reset_drone("recon")
        work.start_recon()
        yield from self.wait_until(
            lambda: work.phase in {"ready", "mission_invalid", "cv_failed", "recon_failed"},
            timeout_s=300.0, label="разведка и CV")
        if not (work.phase == "ready" and work.mission_valid and work.mission):
            raise CaseFailure(
                f"CV не дал валидную миссию: {work.phase} "
                f"{work.cv_error or (work.recon.error if work.recon else '')}")
        self._mission = work.mission
        self._mission_path = work.mission_path
        self._report = work.result.report
        self._work = work
        return work.mission

    def fly(self, mission_path: str, *, faults: dict | None = None,
            registry: str | Path | None = None):
        app = self.app
        app.set_mode("work")
        self.reset_drone("executor")
        self.app.scene.clear_route()
        app.start_executor(str(mission_path), source=None,
                           registry_path=registry or self.registry,
                           faults=faults)
        run = app.execution
        if run is not None and run.plan and run.plan.get("route"):
            self.app.scene.show_route(run.plan["route"],
                                      run.plan.get("targets") or [],
                                      run.plan.get("expanded_forbidden") or [])
        yield from self.wait_until(lambda: app.execution is None,
                                   timeout_s=300.0, label="полёт исполнителя")
        run = app.last_execution
        if run is not None:
            if run.abort_point:
                self.app.scene.show_abort_marker(*run.abort_point)
            for action in run.actions:
                if action.get("status") == "confirmed":
                    point = action.get("point_field")
                    if point and "x" in point and "y" in point:
                        self.app.scene.show_led_point(point["x"], point["y"])
        return run

    def fast_executor(self, mission_path: str, *,
                      registry: str | Path | None = None) -> ExecutorRun:
        """Step an executor without rendering: used for pure regression checks."""
        self.app.set_mode("work")
        self.reset_drone("executor")
        run = ExecutorRun(
            self.app.drone, mission_path,
            self.dir / "exec" / new_id("bench_exec"),
            frame=self.app.local_frame("executor"), start=self.app.starts["executor"],
            registry_path=registry or self.registry)
        guard = 0
        while not run.done and guard < 20000:
            run.step(0.05)
            guard += 1
        return run

    # -- proof ----------------------------------------------------------- #
    def map_for(self, cid: str, *, mission: dict | None = None,
                run: ExecutorRun | None = None,
                extra: str = "", aborted: bool = False) -> str:
        canvas = MapCanvas(self.app.layout, f"{cid} · поле {int(time.time()) % 1000:03d}"
                           if not extra else f"{cid} · {extra}")
        plan_data = (run.plan if run is not None else None) or (mission or {}).get("_plan")
        expanded = None
        if plan_data:
            expanded = plan_data.get("expanded_forbidden")
        elif mission and mission.get("forbidden"):
            expanded = [{"x": x, "y": y} for x, y in
                        expand_polygon([(v["x"], v["y"]) for v in mission["forbidden"][0]["vertices"]],
                                       0.23)]
        canvas.plots().nfz(expanded)
        if plan_data and plan_data.get("route"):
            canvas.route(plan_data["route"])
            for index, target in enumerate(plan_data.get("targets") or [], 1):
                point = target["point"]
                canvas.markers([(point["x"], point["y"], str(index), "target")])
        if mission:
            targets = mission.get("targets") or []
            if not plan_data:
                canvas.markers([(t["point"]["x"], t["point"]["y"], str(i), "target")
                                for i, t in enumerate(targets, 1)])
            if mission.get("skipped"):
                for item in mission["skipped"]:
                    point = item.get("point") or {}
                    if point:
                        canvas.skipped(point.get("x", 0), point.get("y", 0),
                                       f"{item.get('target_id')} P={item.get('priority')}")
        if run is not None and run.abort_point:
            canvas.cross(run.abort_point[0], run.abort_point[1],
                         run.abort_reason or "abort")
        canvas.legend([("A", (206, 62, 62)), ("B", (56, 104, 206)),
                       ("U", (176, 176, 176)), ("маршрут", (80, 230, 255)),
                       ("NFZ+0.23", (250, 170, 60)), ("прерывание", (238, 100, 100))])
        return str(canvas.save(self.dir / f"map_{cid}.png"))

    def proof_for(self, cid: str, *, mission: dict | None = None,
                  run: ExecutorRun | None = None, extra: str = "") -> dict[str, str]:
        proof = {"карта": self.map_for(cid, mission=mission, run=run, extra=extra)}
        if run is not None:
            events = collect_events(run.journal.path)
            proof["таймлайн"] = str(draw_timeline(
                events, self.dir / f"timeline_{cid}.png",
                max(run.t, 0.1), f"{cid} · {run.phase}"))
            proof["журнал"] = str(run.journal.path)
            proof["сводка"] = str(run.run_dir / "summary.json")
        return proof


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _result(cid: str, case: BenchCase, started: float, passed: bool,
            received: str, details: list[str], proof: dict[str, str]) -> CaseResult:
    return CaseResult(cid=cid, title=case.title, expected=case.expected,
                      received=received, passed=bool(passed),
                      duration_s=time.time() - started,
                      details=list(details), proof=dict(proof))


def _confirmed(run: ExecutorRun) -> int:
    return sum(1 for a in run.actions if a.get("status") == "confirmed")


def _route_is_safe(route: list[dict], expanded: list[dict]) -> bool:
    pts = [(p["x"], p["y"]) for p in route]
    poly = [(p["x"], p["y"]) for p in expanded]
    return not any(segment_intersects_polygon(a, b, poly)
                   for a, b in zip(pts, pts[1:]))


# --------------------------------------------------------------------------- #
# C1
# --------------------------------------------------------------------------- #
def case_c1(ctx: BenchContext):
    started = time.time()
    case = CASES["C1"]
    details = []
    lr = ctx.app.local_frame("recon")
    le = ctx.app.local_frame("executor")
    distinct_origins = math.dist(lr.origin, le.origin) >= 0.01
    details.append(f"L_recon: старт в F {lr.origin}, курс {lr.yaw_deg}°")
    details.append(f"L_exec: старт в F {le.origin}, курс {le.yaw_deg}°")

    mission = yield from ctx.ensure_mission()
    recon_landed = math.dist((ctx.app.drone.x, ctx.app.drone.y), lr.origin) < 0.01
    report = ctx._report or {}
    calib = report.get("calibration", {})
    transform = calib.get("local_to_field", {})
    yaw_ok = abs(float(transform.get("yaw_deg", 0.0)) - lr.yaw_deg) < 1.0
    tx_ok = abs(float(transform.get("tx", 99.0)) - lr.origin[0]) < 0.03
    ty_ok = abs(float(transform.get("ty", 99.0)) - lr.origin[1]) < 0.03
    scale_ok = abs(float(transform.get("scale", 0.0)) - 1.0) < 0.02
    details.append(
        f"оценка L_recon→F: масштаб {transform.get('scale', 0):.4f}, "
        f"курс {transform.get('yaw_deg', 0):.2f}°, "
        f"сдвиг {transform.get('tx', 0):.3f}/{transform.get('ty', 0):.3f}")

    targets = mission.get("targets", [])
    classes_ok = bool(targets) and all(t.get("klass") == "A" for t in targets)
    frame_ok = mission.get("coordinate_frame") == "F" and mission.get("units") == "m"
    details.append(f"миссия: {len(targets)} целей, только A={classes_ok}, кадр F={frame_ok}")
    details.append(f"контрольные точки: худшая {calib.get('worst_check_mm', 0):.2f} мм")

    run = yield from ctx.fly(ctx._mission_path)
    confirmed = _confirmed(run)
    landed = (run.drone.alt <= 0.07
              and math.dist((run.drone.x, run.drone.y), le.origin) < 0.01)
    route_ok = bool(run.route_f) and run.route_f[0] == run.route_f[-1] == le.origin
    details.append(f"возврат разведчика на свой старт={recon_landed}, "
                   f"маршрут исполнителя замкнут на его старте={route_ok}")
    details.append(f"исполнитель: подтверждено {confirmed}/{len(targets)}, "
                   f"посадка {run.drone.alt:.3f} м")
    ctx.c1_run = run
    proof = ctx.proof_for("C1", mission=mission, run=run,
                          extra="оба локальных кадра → F")
    proof["снимок"] = ctx.shot("C1_after_flight")

    passed = (distinct_origins and recon_landed and route_ok and yaw_ok and tx_ok and ty_ok and scale_ok
              and classes_ok and frame_ok and confirmed == len(targets) and landed)
    received = (f"начала разные; перевод в F курс {transform.get('yaw_deg', 0):.2f}°; "
                f"подтверждено {confirmed}/{len(targets)}; посадка {run.drone.alt:.2f} м")
    return _result("C1", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
# C2
# --------------------------------------------------------------------------- #
def case_c2(ctx: BenchContext):
    started = time.time()
    case = CASES["C2"]
    details = []
    checks: list[bool] = []

    cm_mission = ctx.make_mission([(2.4, 2.0)], units="cm")
    cm_path = ctx.write_mission(cm_mission, "c2_units_cm")
    ctx.reset_drone()
    cm_run = ctx.app.start_executor(cm_path, registry_path=ctx.registry)
    yield from ctx.frames(2)
    units_rejected = bool(cm_run.failed and "units_must_be_m" in (cm_run.error or ""))
    checks.append(units_rejected)
    details.append(f"см вместо м: refused={cm_run.failed}, "
                   f"ошибка «{cm_run.error}», дрон на HOME={math.dist((ctx.app.drone.x, ctx.app.drone.y), HOME) < 0.01}")

    bad_cal = ctx.make_mission([(2.4, 2.0)], version="calib_unknown_bench")
    bad_path = ctx.write_mission(bad_cal, "c2_unknown_calibration")
    ctx.reset_drone()
    bad_run = ctx.app.start_executor(bad_path, registry_path=ctx.registry)
    yield from ctx.frames(2)
    calib_rejected = bool(bad_run.failed
                          and "unknown_calibration_version" in (bad_run.error or ""))
    checks.append(calib_rejected)
    details.append(f"неизвестная калибровка: refused={bad_run.failed}, "
                   f"ошибка «{bad_run.error}»")

    no_takeoff = (math.dist((ctx.app.drone.x, ctx.app.drone.y), HOME) < 1e-6
                  and abs(ctx.app.drone.alt - 1.2) < 1e-3)
    checks.append(no_takeoff)
    details.append(f"взлёта не было: {no_takeoff}")

    # No auto-correction: the file on disk must stay untouched.
    import json
    raw = json.loads(Path(cm_path).read_text(encoding="utf-8"))
    untouched = raw.get("units") == "cm"
    checks.append(untouched)
    details.append(f"файл не исправлен: units={raw.get('units')}")

    proof = ctx.proof_for("C2", mission=cm_mission, extra="отказ до взлёта")
    proof["снимок"] = ctx.shot("C2_refused")
    passed = all(checks)
    received = ("units_must_be_m + unknown_calibration_version, отказ до взлёта, "
                "файл не изменён" if passed else
                f"units={units_rejected}, calib={calib_rejected}, "
                f"no_takeoff={no_takeoff}, untouched={untouched}")
    return _result("C2", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
# C3
# --------------------------------------------------------------------------- #
def case_c3(ctx: BenchContext):
    started = time.time()
    case = CASES["C3"]
    details = []
    checks: list[bool] = []
    yield from ctx.frames(1)

    expired = ctx.make_mission([(2.4, 2.0)], ttl_s=30.0,
                               created_at=utc_now() - timedelta(minutes=30))
    expired_path = ctx.write_mission(expired, "c3_expired")
    ctx.reset_drone()
    pre = ctx.app.start_executor(expired_path, registry_path=ctx.registry)
    yield from ctx.frames(2)
    before_ok = bool(pre.failed and "map_expired" in (pre.error or ""))
    checks.append(before_ok)
    details.append(f"до старта: map_expired={before_ok}, «{pre.error}»")

    # TTL is wall-clock based, so at x5 speed this short window expires while
    # the drone is still on the route.
    mid = ctx.make_mission([(3.6, 2.2), (4.2, 0.7), (1.4, 1.0)], ttl_s=1.5)
    mid_path = ctx.write_mission(mid, "c3_mid_flight")
    run = yield from ctx.fly(mid_path)
    total_targets = len(run.plan.get("targets", [])) if run else 0
    after_ok = (bool(run) and run.aborted
                and run.abort_reason == "map_expired_or_invalid"
                and run.phase == "aborted")
    checks.append(after_ok)
    not_all_done = _confirmed(run) < total_targets
    checks.append(not_all_done)
    landed = bool(run) and run.drone.alt <= 0.07
    checks.append(landed)
    details.append(f"в полёте: aborted={run.aborted}, причина={run.abort_reason}, "
                   f"статус={run.phase}, посадка {run.drone.alt:.3f} м")
    details.append(f"время до прерывания: {run.t:.1f} с, "
                   f"подтверждено {_confirmed(run)} из {total_targets}")

    proof = ctx.proof_for("C3", mission=mid, run=run, extra="TTL истёк в полёте")
    proof["снимок"] = ctx.shot("C3_aborted")
    passed = all(checks)
    received = (f"до старта отказ; в полёте через {run.t:.1f} с прервано "
                f"({run.abort_reason}), посадка {run.drone.alt:.2f} м")
    return _result("C3", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
# C4
# --------------------------------------------------------------------------- #
def case_c4(ctx: BenchContext):
    started = time.time()
    case = CASES["C4"]
    details = []
    checks: list[bool] = []

    mission = yield from ctx.ensure_mission()
    report = ctx._report or {}
    rejected = report.get("rejected") or []
    zone_hits = [r for r in rejected
                 if any("inside_expanded_forbidden_zone" in reason
                        for reason in (r.get("reasons") or []))]
    classes = [r for r in rejected
               if "class_not_permitted" in (r.get("reasons") or [])]
    checks.append(len(zone_hits) == 2)
    checks.append(len(classes) == 2)
    details.append(f"отклонено в расширенной NFZ: {len(zone_hits)}, по классу: {len(classes)}")

    plan_data = (ctx._work.plan if ctx._work is not None else None) or {}
    route = plan_data.get("route") or []
    expanded = plan_data.get("expanded_forbidden") or []
    safe_plan = bool(route) and bool(expanded) and _route_is_safe(route, expanded)
    checks.append(safe_plan)
    details.append(f"маршрут вне NFZ+0.23: {safe_plan} "
                   f"({len(route)} точек, длина {plan_data.get('length_m', 0):.2f} м)")

    zones = [(v["x"], v["y"]) for v in mission["forbidden"][0]["vertices"]]
    from shared.world import FORBIDDEN_EXPANSION, distance_to_polygon
    inside = [t for t in mission.get("targets", [])
              if distance_to_polygon(t["point"]["x"], t["point"]["y"], zones)
              <= FORBIDDEN_EXPANSION]
    checks.append(not inside)
    details.append(f"целей внутри запретной зоны в миссии: {len(inside)}")

    proof = ctx.proof_for("C4", mission=mission, extra="обход NFZ")
    proof["снимок"] = ctx.shot("C4_route")
    passed = all(checks)
    received = (f"2 A в NFZ исключены, маршрут вне контура, "
                f"зона не пересекается" if passed else
                f"zone_hits={len(zone_hits)}, classes={len(classes)}, safe={safe_plan}")
    return _result("C4", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
# C5
# --------------------------------------------------------------------------- #
def case_c5(ctx: BenchContext):
    started = time.time()
    case = CASES["C5"]
    details = []
    checks: list[bool] = []
    registry = ctx.dir / "registry_c5.json"
    registry.unlink(missing_ok=True)

    mission = ctx.make_mission([(3.3, 2.2)], mission_id=new_id("c5_mission"))
    path = ctx.write_mission(mission, "c5_first")
    run = yield from ctx.fly(path, registry=registry)
    first_ok = _confirmed(run) == 1
    checks.append(first_ok)
    details.append(f"первый запуск: подтверждено {_confirmed(run)}/1")

    # A fresh registry object reads the file from disk: that is what a restart
    # of the application does before the next mission.
    second = ctx.fast_executor(path, registry=registry)
    blocked = [a for a in second.actions
               if a.get("reason") == "duplicate_action_blocked"]
    led_cmds = [e for e in collect_events(second.journal.path)
                if e.get("event") == "led_command"]
    repeat_ok = (not second.failed and _confirmed(second) == 0
                 and len(blocked) == 1 and not led_cmds)
    checks.append(repeat_ok)
    details.append(f"повтор файла: подтверждено {_confirmed(second)}, "
                   f"заблокировано {len(blocked)}, команд LED {len(led_cmds)}")

    # Lost confirmation: an action left "uncertain" by a crash must not repeat.
    uncertain_mission = ctx.make_mission([(3.3, 2.2)], mission_id=new_id("c5_uncertain"))
    uncertain_path = ctx.write_mission(uncertain_mission, "c5_uncertain")
    action_id = uncertain_mission["targets"][0]["action_id"]
    from shared.journal import ActionRegistry
    ActionRegistry(registry).mark(action_id, "uncertain", reason="crash_after_led_on")
    third = ctx.fast_executor(uncertain_path, registry=registry)
    reasons = {a.get("reason") for a in third.actions}
    uncertain_ok = (_confirmed(third) == 0
                    and "uncertain_pending_verification" in reasons)
    checks.append(uncertain_ok)
    details.append(f"UNCERTAIN: причины {sorted(r for r in reasons if r)}")

    proof = ctx.proof_for("C5", mission=mission, run=run,
                          extra="повтор и UNCERTAIN")
    proof["снимок"] = ctx.shot("C5_repeat")
    passed = all(checks)
    received = ("повтор заблокирован, LED не включался, UNCERTAIN без повтора"
                if passed else
                f"first={first_ok}, repeat={repeat_ok}, uncertain={uncertain_ok}")
    return _result("C5", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
# C6
# --------------------------------------------------------------------------- #
def case_c6(ctx: BenchContext):
    started = time.time()
    case = CASES["C6"]
    details = []
    checks: list[bool] = []

    stale_mission = ctx.make_mission([(3.3, 2.2)], mission_id=new_id("c6_nav"))
    stale_path = ctx.write_mission(stale_mission, "c6_navigation")
    run = yield from ctx.fly(stale_path, faults={"localization_lost_after": 3.0})
    nav_ok = (bool(run) and run.aborted
              and run.abort_reason == "stale_localization"
              and run.abort_mode == "land")
    checks.append(nav_ok)
    checks.append(bool(run) and run.drone.alt <= 0.07)
    checks.append(bool(run) and run.localization_age_max > 0.3)
    details.append(f"потеря локализации: причина={run.abort_reason}, "
                   f"режим={run.abort_mode}, возраст {run.localization_age_max:.2f} с, "
                   f"посадка {run.drone.alt:.3f} м")

    link_mission = ctx.make_mission([(3.3, 2.2)], mission_id=new_id("c6_link"))
    link_path = ctx.write_mission(link_mission, "c6_link")
    link_run = yield from ctx.fly(link_path, faults={"link_lost_after": 2.0})
    link_ok = (bool(link_run) and link_run.aborted
               and link_run.abort_reason == "control_link_lost"
               and link_run.abort_mode == "home")
    checks.append(link_ok)
    home_distance = math.dist((link_run.drone.x, link_run.drone.y), HOME)
    checks.append(home_distance < 0.15 and link_run.drone.alt <= 0.07)
    details.append(f"потеря связи: причина={link_run.abort_reason}, "
                   f"режим={link_run.abort_mode}, до HOME {home_distance:.2f} м, "
                   f"посадка {link_run.drone.alt:.3f} м")

    proof = ctx.proof_for("C6", mission=stale_mission, run=run,
                          extra="отказ навигации → аварийная посадка")
    proof["таймлайн связи"] = str(draw_timeline(
        collect_events(link_run.journal.path), ctx.dir / "timeline_C6_link.png",
        max(link_run.t, 0.1), "C6 · потеря связи → возврат"))
    proof["снимок"] = ctx.shot("C6_landed")
    passed = all(checks)
    received = ("LED off, аварийная посадка при потере локализации; "
                "возврат и посадка при потере связи" if passed else
                f"nav={nav_ok}, link={link_ok}, home_ok={home_distance < 0.15}")
    return _result("C6", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
# C7
# --------------------------------------------------------------------------- #
def case_c7(ctx: BenchContext):
    started = time.time()
    case = CASES["C7"]
    details = []
    checks: list[bool] = []
    yield from ctx.frames(1)

    # 1. Contradictory observations of the same zone must become U.
    det_a = Detection("A", 100.0, 100.0, 0.5, 0.5, 0.9, frame_id="bench_a", x=2.5, y=1.5)
    det_b = Detection("B", 104.0, 101.0, 0.5, 0.5, 0.9, frame_id="bench_b", x=2.5, y=1.5)
    observations = merge_detections([det_a, det_b])
    contradiction = bool(observations) and observations[0].klass == "U"
    checks.append(contradiction)
    mission, rejected = build_mission(observations, "bench_c7", ctx.calibration_version,
                                      last_frame_at=utc_now())
    rejected_u = any("class_not_permitted" in (r.get("reasons") or [])
                     for r in rejected)
    checks.append(rejected_u)
    details.append(f"противоречие A/B → {observations[0].klass if observations else '—'}, "
                   f"отклонено миссией={rejected_u}")

    # 2. An observation whose spread breaks the accuracy budget is rejected.
    noisy = Observation(oid="O01", klass="A", x=2.6, y=1.4, n_obs=3,
                        votes={"A": 1.0}, frames=["a", "b", "c"], score=0.8,
                        spread_m=0.10)
    _, noisy_rejected = build_mission([noisy], "bench_c7b", ctx.calibration_version,
                                      last_frame_at=utc_now(), recon_error_m=0.04)
    noisy_reasons = noisy_rejected[0]["reasons"] if noisy_rejected else []
    noisy_ok = ("position_error_out_of_limit" in noisy_reasons
                or "operation_accuracy_budget_exceeded" in noisy_reasons)
    checks.append(noisy_ok)
    details.append(f"разброс 100 мм + калибровка 40 мм → ошибка "
                   f"{noisy_rejected[0].get('position_error_m', 0) if noisy_rejected else 0:.3f} м, "
                   f"причины {noisy_reasons}")

    # 3. A calibration that disagrees with the check points must be refused.
    calib_ok = True
    source = None
    for candidate in sorted((Path(ctx.app.registry_path).parent.parent / "runs").glob("*/frames/calib.png"),
                            key=lambda p: p.stat().st_mtime, reverse=True):
        if (candidate.parent / "calib.json").exists():
            source = candidate
            break
    import cv.calibration as calib_module
    import json
    if source is not None:
        meta = json.loads((source.parent / "calib.json").read_text(encoding="utf-8"))
        import cv2
        image = cv2.imread(str(source))
        original = dict(calib_module.CHECK_MARKERS)
        try:
            shifted = dict(original)
            name, (x, y, rgb) = next(iter(shifted.items()))
            shifted[name] = (x + 0.06, y + 0.06, rgb)
            calib_module.CHECK_MARKERS = shifted
            try:
                calibrate(image, meta["pose"])
                calib_ok = False
            except CalibrationError as exc:
                details.append(f"калибровка сдвинута на 60 мм → отказ: {exc}")
        finally:
            calib_module.CHECK_MARKERS = original
        checks.append(calib_ok)
    else:
        details.append("кадр калибровки недоступен: проверка порога пропущена")
        checks.append(True)

    details.append(f"порог контрольных точек: {MAX_CHECK_ERROR_MM:.0f} мм")
    proof = ctx.proof_for("C7", extra="противоречия и точность")
    proof["снимок"] = ctx.shot("C7_check")
    passed = all(checks)
    received = ("противоречие → U и пропуск; ошибка 0.14 м → пропуск; "
                "калибровка вне допуска отклонена" if passed else
                f"contradiction={contradiction}, noisy={noisy_ok}, calib={calib_ok}")
    return _result("C7", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
# C8
# --------------------------------------------------------------------------- #
def case_c8(ctx: BenchContext):
    started = time.time()
    case = CASES["C8"]
    details = []
    checks: list[bool] = []

    points = [(1.2, 0.8), (2.0, 0.8), (2.8, 0.8), (1.2, 2.0), (2.0, 2.0), (2.8, 2.0)]
    full = ctx.make_mission(points, mission_id=new_id("c8_full"))
    budget = 45.0
    planning = plan(HOME, full["targets"], max_time=budget)
    subset_ok = (bool(planning.get("skipped"))
                 and planning["total_with_reserve_s"] <= budget
                 and all(item.get("reason") for item in planning["skipped"]))
    checks.append(subset_ok)
    subset = apply_subset(full, planning, budget)
    checks.append(bool(subset.get("partial")) and len(subset["targets"]) < len(full["targets"]))
    details.append(f"бюджет {budget:.0f} с: пропущено "
                   f"{[(s['target_id'], s['priority']) for s in planning.get('skipped', [])]}, "
                   f"оценка с резервом {planning.get('total_with_reserve_s', 0):.1f} с")

    # The executor honours the budget stored in the file: a far target cannot
    # be reached inside it, so the mission is aborted and the drone returns.
    tight = ctx.make_mission([(4.2, 0.7)], mission_id=new_id("c8_tight"),
                             budget_s=25.0, ttl_s=600.0)
    tight_path = ctx.write_mission(tight, "c8_tight")
    run = yield from ctx.fly(tight_path)
    abort_ok = (bool(run) and run.aborted
                and run.abort_reason == "budget_exhausted_with_reserve"
                and run.abort_mode == "home")
    checks.append(abort_ok)
    checks.append(bool(run) and run.drone.alt <= 0.07)
    details.append(f"исполнитель: бюджет 25 с → прерывание через {run.t:.1f} с, "
                   f"посадка {run.drone.alt:.3f} м")

    proof = ctx.proof_for("C8", mission=tight, run=run,
                          extra="нехватка времени")
    proof["карта подмножества"] = ctx.map_for(
        "C8_subset", mission=subset, extra="подмножество по приоритету")
    proof["снимок"] = ctx.shot("C8_subset")
    passed = all(checks)
    received = (f"подмножество {len(subset['targets'])}/{len(full['targets'])}, "
                f"причины записаны; исполнитель прервался и вернулся"
                if passed else
                f"subset={subset_ok}, abort={abort_ok}")
    return _result("C8", case, started, passed, received, details, proof)


# --------------------------------------------------------------------------- #
CASES: dict[str, BenchCase] = {
    "C1": BenchCase("C1", "Два локальных начала", "Корректный перевод в F, исполнение только A", case_c1, True),
    "C2": BenchCase("C2", "Единицы и калибровка", "Отказ до взлёта без автокоррекции", case_c2),
    "C3": BenchCase("C3", "Просрочка карты", "Запрет операций и безопасное завершение", case_c3, True),
    "C4": BenchCase("C4", "Запретная зона", "Исключение цели или обход, полёт вне зоны", case_c4),
    "C5": BenchCase("C5", "Повтор и UNCERTAIN", "Нет повторного воздействия", case_c5, True),
    "C6": BenchCase("C6", "Потеря локализации/связи", "LED off, штатное аварийное завершение", case_c6, True),
    "C7": BenchCase("C7", "Ошибка и противоречия", "Пропуск сомнительной цели", case_c7),
    "C8": BenchCase("C8", "Нехватка времени", "Подмножество, причины, возврат", case_c8, True),
}

ORDER = ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8"]


def run_case(ctx: BenchContext, cid: str, tick=None):
    """Drive a case to completion; returns the CaseResult.

    ``tick`` is called after every yield, so the runner decides whether that
    means "render another frame" (the overlay) or "just continue" (pytest).
    """
    case = CASES[cid]
    generator = case.run(ctx)
    while True:
        try:
            next(generator)
        except StopIteration as stop:
            if stop.value is None:
                raise CaseFailure("сценарий завершился без результата")
            return stop.value
        except CaseFailure:
            raise
        except Exception as exc:
            raise CaseFailure(f"{type(exc).__name__}: {exc}\n{traceback.format_exc(limit=3)}")
        if tick is not None:
            tick()


# --------------------------------------------------------------------------- #
# B2 metrics
# --------------------------------------------------------------------------- #
def compute_metrics(ctx: BenchContext) -> dict:
    report = ctx._report or {}
    mission = ctx._mission or {}
    run = ctx.c1_run
    layout = ctx.app.layout
    metrics: dict[str, str] = {}

    counts = report.get("counts", {})
    allowed = sum(1 for o in layout if o.get("klass") == "A"
                  and not _in_nfz(o["x"], o["y"]))
    found = len(mission.get("targets", []))
    metrics["Полнота обнаружения допустимых A"] = f"{found}/{allowed}"
    metrics["Объектов в карте наблюдений"] = str(counts.get("objects", "—"))

    if run is not None:
        confirmed = _confirmed(run)
        total = len(run.plan.get("targets", [])) if run.plan else 0
        metrics["Доля подтверждённых выполнений"] = f"{confirmed}/{total}"
        wrong = 0
        for action in run.actions:
            if action.get("status") != "confirmed":
                continue
            point = action.get("point_field") or {}
            nearest = min((math.dist((point.get("x", 9), point.get("y", 9)),
                                     (o["x"], o["y"])) for o in layout
                           if o.get("klass") == "A"), default=99.0)
            if nearest > 0.25:
                wrong += 1
        metrics["Ошибочные назначения операции"] = str(wrong)

    # Independent check points (evaluation of the calibration, not the fit).
    calib = report.get("calibration", {})
    check_mm = [v for v in (calib.get("check_errors_mm") or {}).values()]
    if check_mm:
        metrics["Ошибка на контрольных точках, средняя"] = f"{sum(check_mm) / len(check_mm):.2f} мм"
        metrics["Ошибка на контрольных точках, максимальная"] = f"{max(check_mm):.2f} мм"

    # Target error against the applied layout (ground truth, evaluation only).
    errors = []
    for target in mission.get("targets", []):
        point = target["point"]
        candidates = [math.dist((point["x"], point["y"]), (o["x"], o["y"]))
                      for o in layout if o.get("klass") == "A"]
        if candidates:
            errors.append(min(candidates))
    if errors:
        metrics["Ошибка координат целей, средняя"] = f"{sum(errors) / len(errors) * 1000:.1f} мм"
        metrics["Ошибка координат целей, максимальная"] = f"{max(errors) * 1000:.1f} мм"

    # Mission time including processing and the role hand-over.
    recon_s = 0.0
    cv_s = float(report.get("duration_s") or 0.0)
    exec_s = float(run.t) if run is not None else 0.0
    if report.get("run_id"):
        run_json = Path(ctx.app.registry_path).parent.parent / "runs" / str(report["run_id"]) / "run.json"
        if run_json.exists():
            import json
            recon_s = float(json.loads(run_json.read_text(encoding="utf-8")).get("t_end_s", 0.0))
    metrics["Время разведки"] = f"{recon_s:.1f} с"
    metrics["Время обработки карты (CV)"] = f"{cv_s:.2f} с"
    metrics["Время исполнителя"] = f"{exec_s:.1f} с"
    metrics["Время миссии суммарно"] = f"{recon_s + cv_s + exec_s:.1f} с"

    if run is not None:
        led_points = [a for a in run.actions if a.get("status") == "confirmed"]
        metrics["Фактических включений LED"] = str(len(led_points))
        if led_points:
            deviations = []
            for action in led_points:
                point = action.get("point_field") or {}
                candidates = [math.dist((point.get("x", 9), point.get("y", 9)),
                                        (o["x"], o["y"])) for o in layout
                              if o.get("klass") == "A"]
                if candidates:
                    deviations.append(min(candidates))
            if deviations:
                metrics["Максимальное отклонение включения от участка"] = \
                    f"{max(deviations) * 1000:.0f} мм"
    return metrics


def _in_nfz(x: float, y: float) -> bool:
    from shared.world import distance_to_polygon, FORBIDDEN_EXPANSION
    return distance_to_polygon(x, y, NO_FLY) <= FORBIDDEN_EXPANSION
