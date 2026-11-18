"""Application shell: three modes, shared drone, speed control, capture loop."""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from panda3d.core import Filename, PNMImage  # noqa: E402
from ursina import (  # noqa: E402
    Sky, Ursina, camera, color, held_keys, scene, time as ursina_time, window,
)

from shared.contracts import new_id  # noqa: E402
from shared.frames import LocalFrame  # noqa: E402
from shared.world import EXEC_LIMIT_SECONDS  # noqa: E402
from sim import scenario, ui  # noqa: E402
from sim.capture import FrameGrabber  # noqa: E402
from sim.drone import Drone  # noqa: E402
from sim.executor import ExecutorRun  # noqa: E402
from sim.modes.free import FreeMode  # noqa: E402
from sim.modes.manual import ManualMode  # noqa: E402
from sim.modes.work import WorkMode  # noqa: E402
from sim.scene import WorldScene  # noqa: E402
from tests.bench import BenchOverlay, run_cli  # noqa: E402

RUNS_DIR = ROOT / "runs"
APP = None


class Application:
    def __init__(self):
        ui.configure_fonts()
        self.ursina = Ursina(
            title="BPLA AT11-2 — 3 режима",
            size=(1280, 720),
            borderless=False,
            development_mode=False,
            editor_ui_enabled=False,
            show_ursina_splash=False,
        )
        window.color = color.rgb32(128, 176, 222)
        Sky()
        self.layout = scenario.load_layout()
        self.scene = WorldScene(layout=self.layout)
        self.grabber = FrameGrabber(self.scene.capture_hidden_entities)
        self.drone = Drone(self.scene.rig)
        self.speed = 1.0
        self.mode = None
        self.mode_name = ""
        self.execution: ExecutorRun | None = None
        self.execution_source = None
        self.last_execution: ExecutorRun | None = None
        self.last_mission: dict | None = None
        self.last_mission_path: str | None = None
        self.registry_path = ROOT / "state" / "action_registry.json"
        self._build_top_bar()
        self.modes = {
            "free": FreeMode(self),
            "manual": ManualMode(self),
            "work": WorkMode(self),
        }
        self.bench = BenchOverlay(self)
        self.set_speed(1.0)
        self.set_mode("free")

    # ------------------------------------------------------------------ #
    def _build_top_bar(self) -> None:
        self.add_top = ui.panel(position=(0.0, 0.455), scale=(1.72, 0.06),
                                tint=color.rgba32(8, 12, 20, 235), name="mode_bar")
        labels = [("free", "1 · СВОБОДНЫЙ ПОЛЁТ"),
                  ("manual", "2 · КОНСТРУКТОР ПОЛЯ"),
                  ("work", "3 · РАЗВЕДКА → МИССИЯ")]
        self.mode_buttons = {}
        for i, (name, text) in enumerate(labels):
            button = ui.button(None, text, (-0.70 + i * 0.35, 0.455),
                               scale=(0.33, 0.046), text_size=0.56,
                               on_click=lambda n=name: self.set_mode(n),
                               name=f"mode_{name}")
            self.mode_buttons[name] = button
        ui.label(None, "ТЕМП", (0.40, 0.445), scale=0.46, tint=ui.MUTED)
        self.speed_buttons = {}
        for i, factor in enumerate((1.0, 2.0, 5.0)):
            button = ui.button(None, f"×{factor:g}", (0.56 + i * 0.105, 0.455),
                               scale=(0.095, 0.042), text_size=0.54,
                               on_click=lambda f=factor: self.set_speed(f),
                               name=f"speed_{factor:g}")
            self.speed_buttons[factor] = button

    # ------------------------------------------------------------------ #
    def set_mode(self, name: str) -> None:
        if name == self.mode_name and self.mode is not None:
            return
        if name not in self.modes:
            return
        bench = getattr(self, "bench", None)
        if bench is not None and bench.running and name != self.mode_name:
            return
        if self.mode is not None:
            self.mode.exit()
        self.mode_name = name
        self.mode = self.modes[name]
        self.mode.enter()
        if bench is not None and bench.visible:
            self.mode.set_ui_visible(False)
        for key, button in self.mode_buttons.items():
            ui.highlight_button(button, key == name)

    def set_speed(self, factor: float) -> None:
        self.speed = float(factor)
        for f, button in self.speed_buttons.items():
            ui.highlight_button(button, abs(f - self.speed) < 1e-9)

    def reset_drone(self, role: str = "neutral", reset_led: bool = True) -> None:
        self.drone.reset()
        self.drone.rig.set_role(role)
        if reset_led:
            self.scene.rig.set_led(False)
        self.scene.sync_visuals(self.drone.alt)

    def apply_layout(self, objects: list[dict], persist: bool = True) -> list[str]:
        """Make an edited layout the physical field; returns validation errors.

        ``persist=False`` is used by the selftest so a diagnostic run never
        overwrites the layout the operator saved.
        """
        errors = scenario.validate(objects)
        if errors:
            return errors
        self.scene.set_layout(objects)
        if persist:
            scenario.save_layout(objects)
        self.layout = scenario.copy_objects(objects)
        work = getattr(self, "modes", {}).get("work")
        if work is not None:
            work.on_layout_applied()
        manual = getattr(self, "modes", {}).get("manual")
        if manual is not None:
            manual.sync_applied(self.layout)
        return []

    # ------------------------------------------------------------------ #
    def start_executor(self, mission_path: str, source=None,
                       registry_path: str | Path | None = None,
                       faults: dict | None = None) -> ExecutorRun:
        run_dir = RUNS_DIR / new_id("exec")
        run = ExecutorRun(self.drone, mission_path, run_dir,
                          frame=LocalFrame.from_spec("L_exec"),
                          registry_path=registry_path or self.registry_path,
                          faults=faults)
        self.execution = run
        self.execution_source = source
        if run.done:
            self._finish_execution()
        return run

    def _finish_execution(self) -> None:
        run = self.execution
        if run is None:
            return
        self.execution = None
        self.last_execution = run
        source = self.execution_source
        self.execution_source = None
        if source is not None and hasattr(source, "on_execution_done"):
            source.on_execution_done(run)

    def cancel_execution(self) -> None:
        if self.execution is None:
            return
        self.execution.cancel()
        self._finish_execution()

    def set_last_mission(self, path: str, payload: dict, errors: list[str],
                         source: str) -> None:
        self.last_mission_path = path
        self.last_mission = payload

    def clear_last_mission(self) -> None:
        self.last_mission_path = None
        self.last_mission = None

    # ------------------------------------------------------------------ #
    def update(self) -> None:
        self.grabber.update()

        real_dt = min(ursina_time.dt, 0.10)
        total = max(0.0, real_dt * self.speed)
        steps = max(1, int(math.ceil(total / 0.02))) if total > 0 else 1
        dt_step = total / steps if total > 0 else 0.0

        for _ in range(steps):
            if self.grabber.busy:
                break
            self.mode.step(dt_step)
            if self.execution is not None:
                if not self.execution.done:
                    self.execution.step(dt_step)
                if self.execution is not None and self.execution.done:
                    break
            if self.grabber.busy:
                break

        if self.execution is not None and self.execution.done:
            self._finish_execution()

        self.bench.update()
        self.scene.sync_visuals(self.drone.alt)
        if self.mode is not None:
            self.mode.refresh()
            if self.bench.visible:
                # The bench owns the screen: the mode's own panels and the
                # route banner must not reappear behind it.
                self.mode.set_ui_visible(False)

    def input(self, key: str) -> None:
        if key == "f9":
            self.bench.toggle()
            return
        if key == "f1":
            self.set_mode("free")
            return
        if key == "f2":
            self.set_mode("manual")
            return
        if key == "f3":
            self.set_mode("work")
            return
        if self.bench.visible:
            return
        if key == "escape" and self.mode is not None:
            self.mode.input(key)
            return
        if self.mode is not None:
            self.mode.input(key)

    # ------------------------------------------------------------------ #
    def screenshot(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        pnm = PNMImage()
        base.win.getScreenshot(pnm)
        pnm.write(Filename.fromOsSpecific(str(path.resolve())))
        print("saved:", path)


# --------------------------------------------------------------------------- #
# module-level hooks used by Ursina
# --------------------------------------------------------------------------- #
def update() -> None:
    if APP is not None:
        APP.update()


def input(key: str) -> None:  # noqa: A001 - name required by Ursina
    if APP is not None:
        APP.input(key)


def run_selftest(app: Application) -> None:
    failures: list[str] = []
    step = app.ursina.step
    saved_layout = scenario.copy_objects(app.layout)

    def check(name: str, condition: bool, detail: str = "") -> None:
        print(f"  {'PASS' if condition else 'FAIL'}  {name} {detail}")
        if not condition:
            failures.append(name)

    def frames(n: int) -> None:
        for _ in range(n):
            step()

    print("1) режимы и свободный полёт")
    app.set_mode("free")
    app.set_speed(5.0)
    app.reset_drone("free")
    frames(3)
    y0 = app.drone.y
    held_keys["t"] = 1
    frames(30)
    held_keys["t"] = 0
    frames(2)
    check("T moves +y in free mode", app.drone.y - y0 > 0.05,
          f"({app.drone.y - y0:+.3f} m)")
    app.set_mode("manual")
    check("manual mode camera is top-down", abs(camera.rotation_x - 90) < 1e-6)

    try:
        print("2) конструктор поля: проверка и применение")
        work = app.modes["work"]
        custom = [
            {"id": "P01", "klass": "A", "x": 1.0, "y": 0.8, "priority": 1},
            {"id": "P02", "klass": "A", "x": 1.8, "y": 0.8, "priority": 2},
            {"id": "P03", "klass": "A", "x": 2.6, "y": 0.8, "priority": 3},
            {"id": "P04", "klass": "B", "x": 1.0, "y": 1.6, "priority": 4},
        ]
        errors = app.apply_layout(custom, persist=False)
        check("custom field applies", not errors, f"errors={errors}")
        check("scene rebuilt from the layout",
              set(app.scene.plot_entities) == {o["id"] for o in custom})
        check("layout change drops the previous mission",
              work.phase == "idle" and work.mission is None)
        bad = [{"id": "P01", "klass": "A", "x": 0.02, "y": 0.02, "priority": 1}]
        check("validation rejects a plot outside the margins",
              bool(app.apply_layout(bad, persist=False)))
        check("rejected layout is not applied",
              set(app.scene.plot_entities) == {o["id"] for o in custom})
        check("default field restored for recon",
              not app.apply_layout(scenario.default_objects(), persist=False))
        check("recon field has the 10 task plots",
              len(app.scene.plot_entities) == 10,
              f"got {len(app.scene.plot_entities)}")

        print("3) ручная карта и миссия")
        manual = app.modes["manual"]
        manual.clear_map()
        manual.set_class("A")
        for x, y in [(2.1, 0.7), (3.0, 1.4), (4.0, 0.7),
                     (4.2, 2.2), (1.6, 2.3), (2.6, 3.2)]:
            manual.add_object(x, y, "A")
        manual.add_object(5.1, 2.1, "A")   # inside NFZ -> rejected
        manual.add_object(3.6, 2.4, "B")
        manual.add_object(3.2, 3.5, "U")
        manual.generate_mission()
        check("manual mission valid", manual.mission_valid,
              f"errors={manual.mission_errors}")
        check("manual mission has 6 A targets",
              bool(manual.mission) and len(manual.mission.get("targets", [])) == 6)
        check("manual map rejects 3 objects",
              bool(manual.mission) and len(manual.mission.get("rejected", [])) == 3)
        if manual.mission:
            from cv.validator import validate_mission
            check("manual mission checksum", not validate_mission(manual.mission))

        print("4) режим 3: разведка -> CV -> исполнитель")
        app.set_mode("work")
        work.reset_work()
        registry = ROOT / "state" / "selftest_registry.json"
        registry.parent.mkdir(parents=True, exist_ok=True)
        registry.unlink(missing_ok=True)
        app.registry_path = registry
        app.set_speed(5.0)
        check("calibration markers hidden before recon",
              not any(e.visible for e in app.scene.calib_entities))
        work.start_recon()
        deadline = time.time() + 240.0
        last_note = 0
        calib_seen_visible = False
        calib_visible_wrongly = False
        while time.time() < deadline:
            if work.recon is not None:
                visible = any(e.visible for e in app.scene.calib_entities)
                if work.recon.phase == "capture_calib":
                    calib_seen_visible = calib_seen_visible or visible
                elif visible:
                    calib_visible_wrongly = True
            step()
            if work.phase in {"ready", "mission_invalid", "cv_failed", "recon_failed"}:
                break
            if work.recon is not None and work.recon.capture_count >= last_note + 5:
                last_note = work.recon.capture_count
                print(f"    кадров {work.recon.capture_count}/"
                      f"{work.recon.expected_captures}, t={work.recon.t:.1f}s")
        check("recon completed", work.recon is not None and work.recon.done and not work.recon.error,
              work.recon.error if work.recon else "no recon")
        check("calibration markers shown only for the calibration frame",
              calib_seen_visible and not calib_visible_wrongly)
        check("calibration markers hidden after recon",
              not any(e.visible for e in app.scene.calib_entities))
        check("CV produced a valid mission", work.phase == "ready" and work.mission_valid,
              f"phase={work.phase} err={work.cv_error or ''}")
        if work.result is not None:
            counts = work.result.report.get("counts", {})
            print(f"    объектов {counts.get('objects')}, A={counts.get('A')}, "
                  f"B={counts.get('B')}, U={counts.get('U')}, "
                  f"целей {counts.get('mission_targets')}, отклонено {counts.get('rejected')}")
            check("10 objects detected", counts.get("objects") == 10,
                  f"got {counts.get('objects')}")
            check("mission has 6 targets", counts.get("mission_targets") == 6,
                  f"got {counts.get('mission_targets')}")
            check("all 4 forbidden/other objects rejected",
                  counts.get("rejected") == 4, f"got {counts.get('rejected')}")
        if work.plan:
            comparison = work.plan.get("comparison") or []
            methods = [m.get("method") for m in comparison]
            check("planner compared two order methods", len(methods) >= 2,
                  f"{methods}")
            feasible = [m for m in comparison if m.get("feasible")]
            shortest = min(feasible, key=lambda m: m.get("length_m", 1e9)) if feasible else None
            check("chosen order is the shortest feasible one",
                  shortest is not None and work.plan.get("method") == shortest["method"],
                  f"chosen={work.plan.get('method')}")
            check("route preview is displayed",
                  bool(app.scene.route_entities) and work.route_title.text == "МАРШРУТ ПОСТРОЕН",
                  f"entities={len(app.scene.route_entities)}")
        else:
            check("planner produced a route", False, "no plan")

        print("5) бюджет времени: подмножество и причины пропуска")
        work.set_budget(60.0)
        skipped = (work.plan or {}).get("skipped") or []
        check("60 s budget selects a subset", bool(skipped), f"{skipped}")
        check("skipped targets carry a reason",
              all(s.get("reason") for s in skipped))
        check("mission file is marked partial",
              bool(work.mission) and work.mission.get("partial") is True)
        if skipped and work.mission:
            dropped = {s["target_id"] for s in skipped}
            present = {t["target_id"] for t in work.mission["targets"]}
            check("subset mission drops exactly the skipped targets",
                  not (dropped & present), f"{sorted(dropped)}")
            check("subset still fits the budget",
                  work.plan.get("total_with_reserve_s", 1e9) <= 60.0,
                  f"{work.plan.get('total_with_reserve_s')} s")
        work.set_budget(EXEC_LIMIT_SECONDS)
        check("full budget restores all 6 targets",
              bool(work.mission) and len(work.mission.get("targets", [])) == 6)

        if work.mission_valid:
            work.start_executor()
            deadline = time.time() + 180.0
            while time.time() < deadline:
                step()
                if app.execution is None and work.phase in {"exec_done", "exec_failed"}:
                    break
            run = app.last_execution
            check("executor finished", run is not None and run.done and not run.failed,
                  run.error if run else "no execution")
            if run is not None:
                confirmed = sum(1 for a in run.actions if a.get("status") == "confirmed")
                check("all targets actioned once", confirmed == len(run.plan.get("targets", [])),
                      f"{confirmed}/{len(run.plan.get('targets', []))}")
                check("drone landed", run.drone.alt <= 0.07,
                      f"alt={run.drone.alt:.3f}")

            print("6) реестр действий: повторный запуск")
            work.start_executor()
            deadline = time.time() + 180.0
            while time.time() < deadline:
                step()
                if app.execution is None and work.phase in {"exec_done", "exec_failed"}:
                    break
            run2 = app.last_execution
            check("second run finished", run2 is not None and run2.done and not run2.failed,
                  run2.error if run2 else "no execution")
            if run2 is not None:
                total2 = len(run2.plan.get("targets", [])) if run2.plan else 0
                confirmed2 = sum(1 for a in run2.actions if a.get("status") == "confirmed")
                blocked2 = sum(1 for a in run2.actions if a.get("status") == "blocked")
                check("registry blocks repeated actions",
                      confirmed2 == 0 and blocked2 == total2,
                      f"confirmed={confirmed2} blocked={blocked2}/{total2}")
    finally:
        app.set_mode("manual")
        app.apply_layout(saved_layout, persist=False)

    print("\nselftest:", "OK" if not failures else f"FAILED {failures}")


def _parse_mode(value: str) -> str:
    return {"1": "free", "2": "manual", "3": "work",
            "free": "free", "manual": "manual", "work": "work"}[value]


def main() -> None:
    global APP
    parser = argparse.ArgumentParser(description="BPLA AT11-2 three-mode application")
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--shot", type=str, default=None)
    parser.add_argument("--mode", type=str, default="1")
    parser.add_argument("--map-demo", action="store_true",
                        help="with --shot: fill the manual map with a demo layout")
    parser.add_argument("--tests", action="store_true",
                        help="run the C1-C8 acceptance scenarios and write a protocol")
    parser.add_argument("--only", type=str, default=None,
                        help="with --tests: comma-separated scenario ids, e.g. C3,C6")
    parser.add_argument("--bench", action="store_true",
                        help="open the acceptance bench overlay (with --shot)")
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()

    APP = Application()
    APP.set_speed(args.speed)

    if args.tests:
        only = None
        if args.only:
            only = [part.strip().upper() for part in args.only.split(",") if part.strip()]
        code = run_cli(APP, only)
        sys.stdout.flush()
        base.userExit()
        os._exit(code)

    if args.selftest:
        run_selftest(APP)
        base.userExit()
        return

    if args.shot:
        mode = _parse_mode(args.mode)
        APP.set_mode(mode)
        if args.map_demo and mode == "manual":
            APP.modes["manual"].load_demo_map()
        if args.bench:
            APP.bench.open()
        for _ in range(6):
            APP.ursina.step()
        APP.screenshot(args.shot)
        base.userExit()
        return

    print(__doc__)
    APP.ursina.run()


if __name__ == "__main__":
    main()
