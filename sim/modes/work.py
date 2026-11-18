"""Mode 3: recon -> CV -> mission -> executor, the official workflow."""
from __future__ import annotations

import copy
import math
import threading
import textwrap
from pathlib import Path

from ursina import camera, mouse

from cv.mission import apply_subset
from cv.pipeline import run_pipeline_safe
from cv.planner import plan
from cv.validator import validate_mission
from shared.contracts import new_id, write_json
from shared.frames import LocalFrame
from shared.world import EXEC_LIMIT_SECONDS, HOME, NO_FLY
from sim import ui
from sim.modes.base import Mode
from sim.recon import ReconRun

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
RUNS_DIR = PROJECT_ROOT / "runs"
MISSIONS_DIR = PROJECT_ROOT / "missions"

PHASE_LABEL = {
    "idle": "ожидание",
    "recon": "разведка",
    "cv": "обработка CV",
    "ready": "миссия готова",
    "mission_invalid": "миссия отклонена",
    "cv_failed": "ошибка CV",
    "exec": "исполнитель",
    "exec_done": "исполнитель завершил",
    "exec_failed": "ошибка исполнителя",
    "recon_failed": "ошибка разведки",
}

BUDGETS = (360.0, 120.0, 60.0)
METHOD_LABEL = {
    "exhaustive": "полный перебор",
    "nearest_neighbour": "ближайший сосед",
    "nearest_neighbour_2opt": "сосед + 2-opt",
}
METHOD_SHORT = {
    "exhaustive": "перебор",
    "nearest_neighbour": "сосед",
    "nearest_neighbour_2opt": "сосед+2-opt",
}


class WorkMode(Mode):
    name = "work"
    title = "3 · РАЗВЕДКА → МИССИЯ"
    role = "neutral"

    def __init__(self, app):
        super().__init__(app)
        self.phase = "idle"
        self.recon: ReconRun | None = None
        self.run_dir: Path | None = None
        self.result = None
        self.base_mission: dict | None = None
        self.plan: dict | None = None
        self.cv_payload = None
        self.cv_error: str | None = None
        self.cv_progress = {"stage": "—", "fraction": 0.0}
        self.cv_event = threading.Event()
        self.cv_thread: threading.Thread | None = None
        self.cv_token = 0
        self.mission_path: str | None = None
        self.mission: dict | None = None
        self.mission_valid = False
        self.mission_errors: list[str] = []
        self.budget_s = EXEC_LIMIT_SECONDS
        self.priority_index = 0
        self.exec_run = None
        self._report_cache = ""
        self._build_ui()

    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        self.add_ui(ui.panel(position=(-0.645, -0.02), scale=(0.43, 0.84),
                             tint=ui.PANEL, name="work_panel"))
        self.add_ui(ui.label(None, "РАЗВЕДКА → МИССИЯ", (-0.84, 0.375),
                             scale=0.72, tint=ui.ACCENT))
        self.add_ui(ui.label(None, "ОФИЦИАЛЬНЫЙ СЦЕНАРИЙ\nбез ручных координат",
                             (-0.84, 0.315), scale=0.50, tint=ui.GOOD))

        self.recon_btn = self.add_ui(ui.button(
            None, "1 · РАЗВЕДКА", (-0.645, 0.215), scale=(0.37, 0.058),
            text_size=0.66, on_click=self.start_recon, name="work_recon_btn"))
        self.exec_btn = self.add_ui(ui.button(
            None, "2 · ИСПОЛНИТЕЛЬ", (-0.645, 0.135), scale=(0.37, 0.058),
            text_size=0.66, on_click=self.start_executor, name="work_exec_btn"))
        self.reset_btn = self.add_ui(ui.button(
            None, "Сброс сценария", (-0.645, 0.065), scale=(0.37, 0.040),
            text_size=0.56, danger=True, on_click=self.reset_work, name="work_reset_btn"))

        self.status = self.add_ui(ui.label(None, "", (-0.84, 0.005), scale=0.54,
                                           tint=ui.INK))

        self.add_ui(ui.panel(position=(0.63, -0.02), scale=(0.46, 0.84),
                             tint=ui.PANEL_SOFT, name="work_report_panel"))
        self.add_ui(ui.label(None, "ОТЧЁТ", (0.42, 0.375), scale=0.76,
                             tint=ui.ACCENT))
        self.report_text = self.add_ui(ui.label(None, "", (0.42, 0.325),
                                                scale=0.445, tint=ui.INK))

        # route preview banner (centre of the view, above the field)
        self.route_panel = self.add_ui(ui.panel(
            position=(0.0, 0.298), scale=(0.36, 0.090), tint=ui.PANEL,
            name="work_route_banner"))
        self.route_title = self.add_ui(ui.label(
            None, "", (0.0, 0.318), origin=(0.0, 0.0), scale=0.80,
            tint=ui.GOOD))
        self.route_stats = self.add_ui(ui.label(
            None, "", (0.0, 0.286), origin=(0.0, 0.0), scale=0.54,
            tint=ui.INK))

        # bottom strip: budget, priority and re-planning
        self.add_ui(ui.panel(position=(0.0, -0.402), scale=(0.79, 0.120),
                             tint=ui.PANEL, name="work_route_panel"))
        self.add_ui(ui.label(None, "БЮДЖЕТ, с", (-0.385, -0.362), scale=0.40,
                             tint=ui.MUTED))
        self.budget_buttons = {}
        for i, value in enumerate(BUDGETS):
            button = self.add_ui(ui.button(
                None, f"{value:g}", (-0.305 + i * 0.070, -0.362),
                scale=(0.062, 0.030), text_size=0.44,
                on_click=lambda v=value: self.set_budget(v),
                name=f"work_budget_{value:g}"))
            self.budget_buttons[value] = button
        self.add_ui(ui.label(None, "ЦЕЛЬ", (-0.100, -0.362), scale=0.40,
                             tint=ui.MUTED))
        self.priority_text = self.add_ui(ui.label(None, "—", (-0.042, -0.362),
                                                  scale=0.42, tint=ui.INK))
        self.add_ui(ui.button(None, "←", (0.058, -0.362), scale=(0.032, 0.030),
                              text_size=0.42, on_click=lambda: self.cycle_target(-1)))
        self.add_ui(ui.button(None, "P−", (0.098, -0.362), scale=(0.036, 0.030),
                              text_size=0.42, on_click=lambda: self.bump_priority(-1)))
        self.add_ui(ui.button(None, "P+", (0.138, -0.362), scale=(0.036, 0.030),
                              text_size=0.42, on_click=lambda: self.bump_priority(+1)))
        self.add_ui(ui.button(None, "→", (0.178, -0.362), scale=(0.032, 0.030),
                              text_size=0.42, on_click=lambda: self.cycle_target(+1)))
        self.replan_btn = self.add_ui(ui.button(
            None, "Пересчитать", (0.310, -0.362), scale=(0.120, 0.030),
            text_size=0.42, on_click=self.replan, name="work_replan_btn"))
        self.order_text = self.add_ui(ui.label(None, "", (-0.385, -0.404),
                                               scale=0.42, tint=ui.INK))
        self.compare_text = self.add_ui(ui.label(None, "", (-0.385, -0.438),
                                                 scale=0.40, tint=ui.MUTED))

    # ------------------------------------------------------------------ #
    def _observer_camera(self) -> None:
        camera.position = (8.6, 5.2, -3.2)
        target = (3.0, -0.6, 2.0)
        dx, dy, dz = target[0] - 8.6, target[1] - 5.2, target[2] + 3.2
        yaw = math.degrees(math.atan2(dx, dz))
        pitch = -math.degrees(math.atan2(dy, math.hypot(dx, dz)))
        camera.rotation = (pitch, yaw, 0)
        camera.fov = 90.0

    def on_enter(self) -> None:
        self._observer_camera()
        mouse.locked = False
        self.app.scene.set_overlays(True)
        if self.phase not in {"recon", "cv", "exec"}:
            self.app.drone.reset()
            self.app.drone.rig.set_role("neutral")
            self.app.scene.clear_trail()
        self._refresh_route_view()
        self.refresh()

    def on_exit(self) -> None:
        if self.phase == "recon" and self.recon is not None:
            self.recon.cancel()
            self.recon = None
            self.phase = "idle"
            self._report_cache = "разведка отменена при смене режима"
        if self.app.execution is not None and self.app.execution_source is self:
            self.app.cancel_execution()
            if self.phase == "exec":
                self.phase = "idle"

    # ------------------------------------------------------------------ #
    def _set_mission(self, result) -> None:
        self.result = result
        if result is not None and result.valid and result.mission_path:
            self.mission_path = result.mission_path
            self.mission = result.mission
            self.mission_valid = True
            self.phase = "ready"
            self.base_mission = copy.deepcopy(result.full_mission or result.mission)
            self.plan = result.plan
            self.priority_index = 0
            self.app.set_last_mission(result.mission_path, result.mission,
                                      result.errors, "cv")
            self._refresh_route_view()
        else:
            self.mission_path = None
            self.mission = None
            self.mission_valid = False
            self.base_mission = None
            self.plan = None
            self.phase = "mission_invalid"
            self.app.clear_last_mission()
            self._refresh_route_view()

    def start_recon(self) -> None:
        if self.app.execution is not None:
            return
        if self.phase in {"recon", "cv"}:
            return
        self.cv_payload = None
        self.cv_error = None
        self.cv_progress = {"stage": "—", "fraction": 0.0}
        self.cv_token += 1
        self.cv_event.clear()
        self.result = None
        self.base_mission = None
        self.plan = None
        self.exec_run = None
        self.mission_path = None
        self.mission = None
        self.mission_valid = False
        self.app.clear_last_mission()
        self._report_cache = ""
        self.app.scene.clear_route()

        run_id = new_id("recon")
        self.run_dir = RUNS_DIR / run_id
        self.app.drone.rig.set_role("recon")
        self.app.scene.clear_trail()
        self.recon = ReconRun(self.app.drone, self.app.grabber, self.run_dir,
                              frame=LocalFrame.from_spec("L_recon"),
                              set_calibration=self.app.scene.set_calibration_visible)
        self.phase = "recon"
        self._observer_camera()
        self.app.scene.set_overlays(True)

    def _start_cv(self) -> None:
        if self.run_dir is None:
            self.phase = "cv_failed"
            self.cv_error = "нет каталога разведки"
            return
        self.phase = "cv"
        self.cv_event.clear()
        self.cv_payload = None
        self.cv_error = None
        self.cv_token += 1
        token = self.cv_token
        event = self.cv_event
        budget = self.budget_s

        def progress(stage: str, fraction: float) -> None:
            if token == self.cv_token:
                self.cv_progress = {"stage": stage, "fraction": float(fraction)}

        def worker() -> None:
            payload = None
            try:
                payload = run_pipeline_safe(self.run_dir, progress=progress,
                                            budget_s=budget)
            except Exception as exc:  # run_pipeline_safe should not raise
                payload = {"ok": False, "result": None,
                           "error": f"{type(exc).__name__}: {exc}",
                           "traceback": ""}
            if token == self.cv_token:
                self.cv_payload = payload
                event.set()

        self.cv_thread = threading.Thread(target=worker, name="cv-pipeline", daemon=True)
        self.cv_thread.start()

    def _poll_cv(self) -> None:
        if not self.cv_event.is_set():
            return
        payload = self.cv_payload or {}
        if not payload.get("ok"):
            self.phase = "cv_failed"
            self.cv_error = payload.get("error") or "неизвестная ошибка CV"
            return
        result = payload["result"]
        self.result = result
        self._set_mission(result)

    def start_executor(self) -> None:
        if not self.mission_valid or not self.mission_path or self.app.execution is not None:
            return
        self.phase = "exec"
        self.exec_run = None
        self.app.drone.rig.set_role("executor")
        self.app.scene.clear_trail()
        self.app.start_executor(self.mission_path, source=self)
        self._refresh_route_view()

    def reset_work(self) -> None:
        self.cv_token += 1
        self.cv_event.clear()
        if self.recon is not None and self.phase == "recon":
            self.recon.cancel()
        self.recon = None
        if self.app.execution is not None and self.app.execution_source is self:
            self.app.cancel_execution()
        self.phase = "idle"
        self.result = None
        self.base_mission = None
        self.plan = None
        self.exec_run = None
        self.mission_path = None
        self.mission = None
        self.mission_valid = False
        self.cv_payload = None
        self.cv_error = None
        self.cv_progress = {"stage": "—", "fraction": 0.0}
        self._report_cache = ""
        self.app.clear_last_mission()
        self.app.scene.clear_route()
        self.app.reset_drone("neutral")
        self._observer_camera()
        self.app.scene.set_overlays(True)

    def on_layout_applied(self) -> None:
        """A new field invalidates the mission built for the previous one."""
        self.cv_token += 1
        self.cv_event.clear()
        self.result = None
        self.base_mission = None
        self.plan = None
        self.mission_path = None
        self.mission = None
        self.mission_valid = False
        self.mission_errors = []
        self.phase = "idle"
        self._report_cache = ""
        self.app.clear_last_mission()
        self.app.scene.clear_route()
        self._refresh_route_view()

    # ------------------------------------------------------------------ #
    def set_budget(self, value: float) -> None:
        self.budget_s = float(value)
        for budget, button in self.budget_buttons.items():
            ui.highlight_button(button, abs(budget - self.budget_s) < 1e-9)
        if self.base_mission is not None:
            self.replan()
        else:
            self._report_cache = ""

    def cycle_target(self, delta: int) -> None:
        targets = self._targets()
        if not targets:
            return
        self.priority_index = (self.priority_index + delta) % len(targets)
        self._report_cache = ""
        self.refresh()

    def bump_priority(self, delta: int) -> None:
        targets = self._targets()
        if not targets:
            return
        self.priority_index %= len(targets)
        selected = targets[self.priority_index]
        order = sorted(targets, key=lambda t: (int(t.get("priority", 0)), t["target_id"]))
        index = order.index(selected)
        target = max(0, min(len(order) - 1, index + delta))
        order.insert(target, order.pop(index))
        for i, item in enumerate(order, 1):
            item["priority"] = i
        self.priority_index = order.index(selected)
        self.replan()

    def _targets(self) -> list[dict]:
        if self.base_mission is None:
            return []
        return self.base_mission.get("targets", [])

    def replan(self) -> None:
        """Recompute the route for the current budget and priorities."""
        if self.base_mission is None or self.mission_path is None:
            return
        targets = copy.deepcopy(self._targets())
        if not targets:
            return
        planning = plan(HOME, targets, forbidden=NO_FLY, max_time=self.budget_s)
        mission = apply_subset(self.base_mission, planning, self.budget_s)
        errors = validate_mission(mission)
        self.plan = planning
        self.mission = mission
        self.mission_errors = errors
        self.mission_valid = not errors and bool(mission.get("targets"))
        if self.mission_valid:
            self.app.set_last_mission(self.mission_path, mission, errors, "cv")
        else:
            self.app.clear_last_mission()
        write_json(self.mission_path, mission)
        if self.run_dir is not None:
            write_json(self.run_dir / "mission_candidate.json", mission)
            write_json(self.run_dir / "plan.json", planning)
        latest = MISSIONS_DIR / "latest_valid.json"
        if self.mission_valid:
            write_json(latest, mission)
        elif latest.exists():
            latest.unlink(missing_ok=True)
        if self.result is not None:
            self.result.report["partial"] = bool(planning.get("skipped"))
            self.result.report["budget_s"] = round(self.budget_s, 1)
            self.result.report["plan"] = {k: planning.get(k) for k in
                                          ("order", "route", "length_m", "estimated_s",
                                           "reserve_s", "total_with_reserve_s", "method",
                                           "chosen_method", "comparison", "skipped",
                                           "valid", "budget_s")}
            self.result.report["counts"]["mission_targets"] = len(mission.get("targets", []))
            self.result.report["counts"]["skipped"] = len(planning.get("skipped") or [])
        self._report_cache = ""
        self._refresh_route_view()
        self.refresh()

    def step(self, dt: float) -> None:
        if self.phase == "recon" and self.recon is not None:
            self.recon.step(dt)
            if self.recon.done:
                if self.recon.error:
                    self.phase = "recon_failed"
                else:
                    self._start_cv()
        elif self.phase == "cv":
            self._poll_cv()

    def input(self, key: str) -> None:
        if key == "1":
            self.start_recon()
        elif key == "2":
            self.start_executor()
        elif key == "backspace":
            self.reset_work()

    def on_execution_done(self, run) -> None:
        self.exec_run = run
        if run.failed:
            self.phase = "exec_failed"
        else:
            self.phase = "exec_done"
        self._refresh_route_view()
        self.refresh()

    # ------------------------------------------------------------------ #
    def _refresh_route_view(self) -> None:
        plan_data = self.plan or {}
        route = plan_data.get("route") or []
        show = (self.phase in {"ready", "exec", "exec_done"}
                and bool(route) and self.mission_valid)
        if not show:
            self.app.scene.clear_route()
            self.route_panel.enabled = False
            self.route_title.text = ""
            self.route_stats.text = ""
            self.order_text.text = ""
            self.compare_text.text = ""
            self._update_priority_text()
            return
        self.route_panel.enabled = True
        if self.phase == "exec":
            title = "МАРШРУТ ВЫПОЛНЯЕТСЯ"
        elif self.phase == "exec_done":
            title = "МАРШРУТ ВЫПОЛНЕН"
        else:
            title = "МАРШРУТ ПОСТРОЕН"
        if self.phase == "ready":
            # The recon trail is historical; keep the field clean for the briefing.
            self.app.scene.clear_trail()
        if getattr(self.app.scene, "route_entities", None):
            # Rebuild only when the plan actually changed; keep the line stable.
            self.app.scene.clear_route()
        self.app.scene.show_route(route, plan_data.get("targets") or [],
                                  plan_data.get("expanded_forbidden") or [])
        methods = plan_data.get("comparison") or []
        chosen = plan_data.get("method", "—")
        alternative = next((m for m in methods if m.get("method") != chosen), None)
        stats = (f"{plan_data.get('length_m', 0):.2f} м · "
                 f"{plan_data.get('estimated_s', 0):.0f} с + резерв "
                 f"{plan_data.get('reserve_s', 0):.0f} с")
        if plan_data.get("skipped"):
            stats += f" · пропущено по бюджету: {len(plan_data['skipped'])}"
        self.route_title.text = title
        self.route_stats.text = stats
        order = plan_data.get("order") or []
        self.order_text.text = ("порядок: " + " → ".join(order)) if order else "порядок: —"
        if alternative is not None:
            self.compare_text.text = (
                f"сравнение: {METHOD_SHORT.get(chosen, chosen)} "
                f"{plan_data.get('length_m', 0):.2f} м vs "
                f"{alternative.get('length_m', 0):.2f} м "
                f"({METHOD_SHORT.get(alternative.get('method'), alternative.get('method'))})")
        else:
            self.compare_text.text = "сравнение: одна методика"
        self._update_priority_text()

    def _update_priority_text(self) -> None:
        targets = self._targets()
        if not targets:
            self.priority_text.text = "—"
            return
        self.priority_index %= len(targets)
        target = targets[self.priority_index]
        self.priority_text.text = (f"{target['target_id']}  P={int(target.get('priority', 0))}"
                                   f"  ({self.priority_index + 1}/{len(targets)})")

    # ------------------------------------------------------------------ #
    def _lines(self) -> list[str]:
        lines: list[str] = []
        lines.append(f"ФАЗА: {PHASE_LABEL.get(self.phase, self.phase)}")
        if self.recon is not None and self.phase in {"recon", "cv"}:
            lines.append(f"кадров: {self.recon.capture_count}/{self.recon.expected_captures}")
            lines.append(f"время разведки: {self.recon.t:.1f} с")
            lines.append(self.recon.status)
        if self.phase == "cv":
            pct = int(round(self.cv_progress.get("fraction", 0.0) * 100))
            lines.append(f"CV: {pct}%  {self.cv_progress.get('stage', '')}")
        if self.cv_error:
            lines += ["", "ОШИБКА CV"]
            lines += textwrap.wrap(self.cv_error, 52)[:5]
        if self.result is not None:
            report = self.result.report
            cal = report.get("calibration", {})
            counts = report.get("counts", {})
            lines += ["", "КАЛИБРОВКА"]
            lines.append(f"проверка: {cal.get('worst_check_mm', float('nan')):.1f} мм")
            lines.append(f"f: {cal.get('f_px', 0):.1f} px")
            lines += ["", "ОБЪЕКТЫ"]
            lines.append(f"кадров: {counts.get('scan_frames', 0)}")
            lines.append(f"объектов: {counts.get('objects', 0)}")
            lines.append(f"A={counts.get('A', 0)}  B={counts.get('B', 0)}  U={counts.get('U', 0)}")
            lines.append(f"целей миссии: {counts.get('mission_targets', 0)}")
            lines.append(f"отклонено: {counts.get('rejected', 0)}")
            if counts.get("skipped"):
                lines.append(f"пропущено по бюджету: {counts['skipped']}")
            if report.get("partial"):
                lines.append("МИССИЯ: ЧАСТИЧНАЯ (бюджет)")
            if report.get("validation_errors"):
                lines += ["", "ВАЛИДАТОР"]
                for err in report["validation_errors"][:5]:
                    lines += textwrap.wrap(err, 52)[:2]
            plan_data = self.plan or report.get("plan") or {}
            if plan_data.get("route"):
                lines += ["", "МАРШРУТ"]
                lines.append(f"длина: {plan_data.get('length_m', 0):.2f} м")
                lines.append(f"оценка: {plan_data.get('estimated_s', 0):.0f} с + "
                             f"резерв {plan_data.get('reserve_s', 0):.0f} с")
                lines.append(f"бюджет: {plan_data.get('budget_s', self.budget_s):.0f} с")
                lines.append(f"метод: {METHOD_LABEL.get(plan_data.get('method'), plan_data.get('method', '—'))}")
                for method in plan_data.get("comparison") or []:
                    mark = "✓" if method.get("method") == plan_data.get("chosen_method",
                                                                         plan_data.get("method")) else " "
                    name = METHOD_LABEL.get(method.get("method"), method.get("method"))
                    lines.append(f"{mark} {name}: {method.get('length_m', 0):.2f} м, "
                                 f"{method.get('total_with_reserve_s', 0):.0f} с")
                for item in (plan_data.get("skipped") or [])[:4]:
                    lines.append(f"− {item['target_id']} (P={item.get('priority', 0)}): "
                                 f"не хватает времени")
            lines += ["", "МИССИЯ"]
            if self.mission_valid and self.mission_path:
                lines.append("СТАТУС: ГОТОВА")
                lines.append(Path(self.mission_path).name[:42])
            else:
                lines.append("СТАТУС: НЕ ГОТОВА")
                for err in self.mission_errors[:3]:
                    lines += textwrap.wrap(err, 52)[:2]
        if self.exec_run is not None:
            run = self.exec_run
            lines += ["", "ИСПОЛНИТЕЛЬ"]
            confirmed = sum(1 for a in run.actions if a.get("status") == "confirmed")
            blocked = sum(1 for a in run.actions if a.get("status") == "blocked")
            total = len(run.plan.get("targets", [])) if run.plan else 0
            lines.append(f"подтверждено: {confirmed}/{total}")
            lines.append(f"заблокировано: {blocked}")
            lines.append(f"время: {run.t:.1f} с")
            if run.error:
                lines += textwrap.wrap(run.error, 52)[:4]
            lines.append(f"журнал: {run.run_dir.name}/journal.jsonl"[:50])
        if not lines:
            lines = ["Нажмите «1 · РАЗВЕДКА».",
                     "После посадки CV построит миссию.",
                     "",
                     "Кнопка «2 · ИСПОЛНИТЕЛЬ»",
                     "станет активной только для",
                     "валидной миссии."]
        elif self.phase == "idle" and self.result is None:
            lines += ["", "ПОРЯДОК РАБОТЫ",
                      "1. «1 · РАЗВЕДКА» — облёт поля.",
                      "2. Посадка, затем автоматический",
                      "   разбор кадров и построение миссии.",
                      "3. Маршрут показывается на поле.",
                      "4. «2 · ИСПОЛНИТЕЛЬ» — полёт по миссии.",
                      "",
                      "Ручные координаты не используются."]
        return lines

    def refresh(self) -> None:
        busy = self.phase in {"recon", "cv", "exec"} or self.app.execution is not None
        ui.set_button(self.recon_btn, not busy)
        ui.set_button(self.exec_btn, bool(self.mission_valid and not busy))
        ui.set_button(self.reset_btn, self.app.execution is None)
        ready = bool(self.mission_valid and self.plan)
        ui.set_button(self.replan_btn, ready and not busy)
        for value, button in self.budget_buttons.items():
            if busy:
                ui.set_button(button, False)
            else:
                button.disabled = False
                ui.highlight_button(button, abs(value - self.budget_s) < 1e-9)
        if self.recon is not None and self.phase in {"recon", "cv"}:
            status = [
                f"разведка: {self.recon.capture_count}/{self.recon.expected_captures}",
                self.recon.status,
                f"t = {self.recon.t:.1f} с",
                f"кадр {self.recon.phase}",
            ]
        elif self.phase == "cv":
            pct = int(round(self.cv_progress.get("fraction", 0.0) * 100))
            status = [f"CV: {pct}%", self.cv_progress.get("stage", ""),
                      "ожидайте завершения"]
        elif self.phase in {"ready", "exec", "exec_done"}:
            status = [f"фаза: {PHASE_LABEL.get(self.phase, self.phase)}",
                      f"миссия: {Path(self.mission_path).name if self.mission_path else '—'}"[:40],
                      f"бюджет: {self.budget_s:.0f} с",
                      "маршрут показан на поле"]
        elif self.phase in {"mission_invalid", "cv_failed", "recon_failed", "exec_failed"}:
            status = [f"фаза: {PHASE_LABEL.get(self.phase, self.phase)}",
                      self.cv_error or (self.recon.error if self.recon else "") or
                      (self.exec_run.error if self.exec_run else ""),
                      "см. отчёт справа"]
        else:
            status = ["готов к запуску", "1 — разведка", "2 — исполнитель"]
        self.status.text = "\n".join(textwrap.wrap(line, 42) if len(line) > 42 else line
                                     for line in status if line is not None)
        text = "\n".join(self._lines())
        if text != self._report_cache:
            self.report_text.text = text
            self._report_cache = text
