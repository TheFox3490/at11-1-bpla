"""Mode 3: recon -> CV -> mission -> executor, the official workflow."""
from __future__ import annotations

import math
import threading
import textwrap
import time
from pathlib import Path

from ursina import camera, mouse

from cv.pipeline import run_pipeline_safe
from shared.contracts import new_id
from shared.frames import LocalFrame
from sim import ui
from sim.modes.base import Mode
from sim.recon import ReconRun

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
RUNS_DIR = PROJECT_ROOT / "runs"

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
        self.cv_payload = None
        self.cv_error: str | None = None
        self.cv_progress = {"stage": "—", "fraction": 0.0}
        self.cv_event = threading.Event()
        self.cv_thread: threading.Thread | None = None
        self.cv_token = 0
        self.mission_path: str | None = None
        self.mission: dict | None = None
        self.mission_valid = False
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
                                                scale=0.475, tint=ui.INK))

    # ------------------------------------------------------------------ #
    def _observer_camera(self) -> None:
        camera.position = (8.6, 5.2, -3.2)
        target = (3.0, 0.6, 2.0)
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
            self.app.set_last_mission(result.mission_path, result.mission,
                                      result.errors, "cv")
        else:
            self.mission_path = None
            self.mission = None
            self.mission_valid = False
            self.phase = "mission_invalid"
            self.app.clear_last_mission()

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
        self.exec_run = None
        self.mission_path = None
        self.mission = None
        self.mission_valid = False
        self.app.clear_last_mission()
        self._report_cache = ""

        run_id = new_id("recon")
        self.run_dir = RUNS_DIR / run_id
        self.app.drone.rig.set_role("recon")
        self.app.scene.clear_trail()
        self.recon = ReconRun(self.app.drone, self.app.grabber, self.run_dir,
                              frame=LocalFrame.from_spec("L_recon"))
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

        def progress(stage: str, fraction: float) -> None:
            if token == self.cv_token:
                self.cv_progress = {"stage": stage, "fraction": float(fraction)}

        def worker() -> None:
            payload = None
            try:
                payload = run_pipeline_safe(self.run_dir, progress=progress)
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
        self.exec_run = None
        self.mission_path = None
        self.mission = None
        self.mission_valid = False
        self.cv_payload = None
        self.cv_error = None
        self.cv_progress = {"stage": "—", "fraction": 0.0}
        self._report_cache = ""
        self.app.clear_last_mission()
        self.app.reset_drone("neutral")
        self._observer_camera()
        self.app.scene.set_overlays(True)

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
        self.refresh()

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
            lines += textwrap.wrap(self.cv_error, 56)[:5]
        if self.result is not None:
            report = self.result.report
            cal = report.get("calibration", {})
            counts = report.get("counts", {})
            lines += ["", "КАЛИБРОВКА"]
            lines.append(f"проверка: {cal.get('worst_check_mm', float('nan')):.1f} мм")
            lines.append(f"f: {cal.get('f_px', 0):.1f} px")
            lines += ["", "ОБЪЕКТЫ"]
            lines.append(f"кадров: {counts.get('scan_frames', 0)}")
            lines.append(f"детекций: {counts.get('detections', 0)}")
            lines.append(f"объектов: {counts.get('objects', 0)}")
            lines.append(f"A={counts.get('A', 0)}  B={counts.get('B', 0)}  U={counts.get('U', 0)}")
            lines.append(f"целей миссии: {counts.get('mission_targets', 0)}")
            lines.append(f"отклонено: {counts.get('rejected', 0)}")
            if report.get("validation_errors"):
                lines += ["", "ВАЛИДАТОР"]
                for err in report["validation_errors"][:6]:
                    lines += textwrap.wrap(err, 56)[:2]
            plan = report.get("plan") or {}
            if plan.get("route"):
                lines += ["", "МАРШРУТ"]
                lines.append(f"длина: {plan.get('length_m', 0):.2f} м")
                lines.append(f"оценка: {plan.get('estimated_s', 0):.0f} с + "
                             f"резерв {plan.get('reserve_s', 0):.0f} с")
                lines.append(f"метод: {plan.get('method', '—')}")
            lines += ["", "МИССИЯ"]
            if self.mission_valid and self.mission_path:
                lines.append("СТАТУС: ГОТОВА")
                lines.append(Path(self.mission_path).name[:44])
            else:
                lines.append("СТАТУС: НЕ ГОТОВА")
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
                lines += textwrap.wrap(run.error, 56)[:4]
            lines.append(f"журнал: {run.run_dir.name}/journal.jsonl"[:52])
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
                      "3. «2 · ИСПОЛНИТЕЛЬ» — полёт по миссии.",
                      "",
                      "Ручные координаты не используются."]
        return lines

    def refresh(self) -> None:
        busy = self.phase in {"recon", "cv", "exec"} or self.app.execution is not None
        ui.set_button(self.recon_btn, not busy)
        ui.set_button(self.exec_btn, bool(self.mission_valid and not busy))
        ui.set_button(self.reset_btn, self.app.execution is None)
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
                      "кнопка 2 активна"]
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
