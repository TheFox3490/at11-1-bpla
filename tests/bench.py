"""In-app acceptance bench (F9): C1-C8, live field view, protocol artifacts.

The bench is deliberately *not* a fourth mode: it is a service overlay that
runs the same three-mode application through the scenarios of task B.  The
field, the drone and the route on screen are the real ones, so the run is a
demonstration, not a mock.
"""
from __future__ import annotations

import textwrap
import time
from pathlib import Path

from ursina import Entity, Texture, color

from shared.contracts import new_id
from sim import ui

from . import cases as case_module  # noqa: F401  (kept for discoverability)
from .cases import (
    CASES, ORDER, BenchContext, CaseFailure, CaseResult, compute_metrics,
)
from .report import write_protocol

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RUNS_DIR = PROJECT_ROOT / "runs"

SHORT_TITLE = {
    "C1": "два начала",
    "C2": "единицы/калибр",
    "C3": "просрочка",
    "C4": "NFZ и обход",
    "C5": "повтор/UNCERT",
    "C6": "локализация",
    "C7": "ошибки",
    "C8": "бюджет",
}


class BenchOverlay:
    def __init__(self, app):
        self.app = app
        self.visible = False
        self.running = False
        self.finished = False
        self.current_cid: str | None = None
        self.results: dict[str, CaseResult] = {}
        self.order: list[str] = []
        self.error: str | None = None
        self.dir: Path | None = None
        self.ctx: BenchContext | None = None
        self._queue: list[str] = []
        self._gen = None
        self._case_started = 0.0
        self._speed_before = 1.0
        self._entities: list = []
        self._rows: dict[str, dict] = {}
        self._map_entity: Entity | None = None
        self._map_texture: Texture | None = None
        self._pending_shot: tuple[str, Path] | None = None
        self._build()

    # ------------------------------------------------------------------ #
    def _build(self) -> None:
        def add(entity):
            self._entities.append(entity)
            return entity

        add(ui.panel(position=(-0.715, -0.015), scale=(0.30, 0.88),
                     tint=color.rgba32(8, 13, 22, 244), name="bench_left"))
        add(ui.panel(position=(0.715, -0.015), scale=(0.30, 0.88),
                     tint=color.rgba32(8, 13, 22, 244), name="bench_right"))
        add(ui.panel(position=(0.0, 0.400), scale=(0.78, 0.072),
                     tint=color.rgba32(8, 13, 22, 240), name="bench_top"))
        add(ui.panel(position=(0.0, -0.412), scale=(0.78, 0.080),
                     tint=color.rgba32(8, 13, 22, 240), name="bench_bottom"))

        add(ui.label(None, "ИСПЫТАНИЯ C1–C8", (-0.855, 0.388), scale=0.58,
                     tint=ui.ACCENT))
        self.counter = add(ui.label(None, "готово 0/8", (-0.855, 0.352),
                                    scale=0.44, tint=ui.MUTED))
        self.run_all_btn = add(ui.button(None, "Прогнать все", (-0.760, 0.305),
                                         scale=(0.165, 0.036), text_size=0.44,
                                         on_click=self.start_all,
                                         name="bench_run_all"))
        self.cancel_btn = add(ui.button(None, "Стоп", (-0.632, 0.305),
                                        scale=(0.075, 0.036), text_size=0.44,
                                        danger=True, on_click=self.stop,
                                        name="bench_stop"))

        row_y = 0.252
        for cid in ORDER:
            case = CASES[cid]
            add(ui.label(None, cid, (-0.852, row_y + 0.008), scale=0.46,
                         tint=ui.INK))
            add(ui.label(None, SHORT_TITLE.get(cid, case.title),
                         (-0.812, row_y + 0.008), scale=0.38, tint=ui.MUTED,
                         max_width=0.155))
            state = add(ui.label(None, "—", (-0.648, row_y + 0.008), scale=0.40,
                                 tint=ui.MUTED, max_width=0.032))
            button = add(ui.button(None, ">", (-0.592, row_y + 0.006),
                                   scale=(0.040, 0.028), text_size=0.44,
                                   on_click=lambda c=cid: self.start_one(c),
                                   name=f"bench_run_{cid}"))
            self._rows[cid] = {"state": state, "button": button, "y": row_y}
            row_y -= 0.060

        add(ui.label(
            None, "F9 — закрыть · протокол в runs/tests/\nсценарии идут на реальном поле",
            (-0.855, -0.398), scale=0.36, tint=ui.MUTED, max_width=0.28))

        # top banner: what is running now
        self.now_title = add(ui.label(None, "стенд готов", (0.0, 0.416),
                                      origin=(0.0, 0.0), scale=0.58,
                                      tint=ui.INK, max_width=0.75))
        self.now_text = add(ui.label(None, "выберите сценарий или «Прогнать все»",
                                     (0.0, 0.386), origin=(0.0, 0.0),
                                     scale=0.42, tint=ui.MUTED, max_width=0.75))

        # right card
        add(ui.label(None, "СЦЕНАРИЙ", (0.578, 0.388), scale=0.40, tint=ui.MUTED))
        self.case_title = add(ui.label(None, "—", (0.582, 0.348), scale=0.48,
                                       tint=ui.ACCENT, max_width=0.265))
        self.case_expected = add(ui.label(
            None, "слева выберите сценарий\nили «Прогнать все».\n\n"
                  "каждый сценарий идёт на\nреальном поле: маршрут,\n"
                  "дрон и журнал — настоящие.",
            (0.582, 0.314), scale=0.35, tint=ui.MUTED, max_width=0.265, max_height=0.06))
        self.case_received = add(ui.label(None, "", (0.582, 0.246), scale=0.35,
                                          tint=ui.INK, max_width=0.265, max_height=0.085))
        self.case_details = add(ui.label(None, "", (0.582, 0.152), scale=0.32,
                                         tint=ui.MUTED, max_width=0.265, max_height=0.23))
        add(ui.label(None, "КАРТА", (0.582, -0.092), scale=0.36, tint=ui.MUTED))
        self._map_entity = add(ui.panel(position=(0.715, -0.196), z=-0.03,
                                        scale=(0.27, 0.172), tint=color.white,
                                        name="bench_map"))
        self._map_entity.visible = False
        self.case_proof = add(ui.label(None, "", (0.582, -0.300), scale=0.30,
                                       tint=ui.MUTED, max_width=0.265, max_height=0.145))

        # bottom: protocol path
        self.status = add(ui.label(None, "", (0.0, -0.400), origin=(0.0, 0.0),
                                   scale=0.40, tint=ui.INK, max_width=0.75))

        for entity in self._entities:
            entity.enabled = False

    # ------------------------------------------------------------------ #
    def toggle(self) -> None:
        if self.visible:
            self.close()
        else:
            self.open()

    def open(self) -> None:
        if self.visible:
            return
        self.visible = True
        for entity in self._entities:
            entity.enabled = True
        self._refresh_all()
        self.app.set_mode("work")
        self.app.mode.set_ui_visible(False)

    def close(self) -> None:
        if not self.visible:
            return
        if self.running:
            self.stop()
        self.visible = False
        for entity in self._entities:
            entity.enabled = False
        self.app.scene.clear_test_markers()
        if self.app.mode is not None:
            self.app.mode.set_ui_visible(True)

    # ------------------------------------------------------------------ #
    def start_all(self, only: list[str] | None = None) -> None:
        if self.running:
            return
        self._reset_run()
        self._queue = list(only) if only else list(ORDER)
        self.running = True
        self.finished = False
        self.dir = RUNS_DIR / "tests" / new_id("bench")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.ctx = BenchContext(self.app, self.dir)
        self._speed_before = self.app.speed
        self.app.set_speed(5.0)
        self.app.set_mode("work")
        self.app.mode.set_ui_visible(False)
        self.status.text = f"прогон: {self.dir}"
        self._refresh_all()

    def start_one(self, cid: str) -> None:
        if self.running:
            return
        self._reset_run()
        self._queue = [cid]
        self.running = True
        self.finished = False
        self.dir = RUNS_DIR / "tests" / new_id("bench")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.ctx = BenchContext(self.app, self.dir)
        self._speed_before = self.app.speed
        self.app.set_speed(5.0)
        self.app.set_mode("work")
        self.app.mode.set_ui_visible(False)
        self.status.text = f"прогон: {self.dir}"
        self._refresh_all()

    def stop(self) -> None:
        if not self.running:
            return
        self.running = False
        self._gen = None
        self._queue = []
        self.current_cid = None
        self.now_title.text = "прогон остановлен"
        self.now_text.text = "результаты сохранены не полностью"
        self._write_report()
        self.app.set_speed(self._speed_before)
        if self.ctx is not None:
            self.ctx.close()
        self._refresh_all()

    def _reset_run(self) -> None:
        self.results.clear()
        self.order.clear()
        self.finished = False
        self.error = None
        self.current_cid = None
        for cid in ORDER:
            self._rows[cid]["state"].text = "—"
            self._rows[cid]["state"].color = ui.MUTED
        if self._map_entity is not None:
            self._map_entity.visible = False
        self.case_title.text = "—"
        self.case_expected.text = ("слева выберите сценарий\nили «Прогнать все».\n\n"
                                   "каждый сценарий идёт на\nреальном поле: маршрут,\n"
                                   "дрон и журнал — настоящие.")
        self.case_received.text = ""
        self.case_details.text = ""
        self.case_proof.text = ""
        self._refresh_all()

    # ------------------------------------------------------------------ #
    def update(self) -> None:
        if self._pending_shot is not None:
            cid, path = self._pending_shot
            self._pending_shot = None
            try:
                self.app.screenshot(path)
                result = self.results.get(cid)
                if result is not None:
                    result.proof["итог"] = str(path)
                    self._write_report()
            except Exception:
                pass
        if not self.running:
            return
        if self._gen is None:
            if not self._queue:
                self._finish()
                return
            self._begin(self._queue.pop(0))
            return
        try:
            next(self._gen)
        except StopIteration as stop:
            self._complete(stop.value)
        except CaseFailure as exc:
            self._fail_current(exc)
        except Exception as exc:  # defensive: never kill the app from the bench
            self._fail_current(CaseFailure(f"{type(exc).__name__}: {exc}"))
        if self.running and self.current_cid and time.time() - self._case_started > 0.4:
            self.now_text.text = (f"{self.current_cid}: {CASES[self.current_cid].title} · "
                                  f"{time.time() - self._case_started:.0f} с")

    def _begin(self, cid: str) -> None:
        self.current_cid = cid
        self._case_started = time.time()
        self.app.scene.clear_route()
        self.app.scene.clear_test_markers()
        self.app.scene.clear_trail()
        self._rows[cid]["state"].text = "…"
        self._rows[cid]["state"].color = ui.WARN
        case = CASES[cid]
        self.now_title.text = f"{cid} · {case.title}"
        self.now_text.text = "идёт сценарий"
        self.case_title.text = f"{cid} · {case.title}"
        self.case_expected.text = self._wrap(f"ожидалось: {case.expected}", 44)
        self.case_received.text = "получено: …"
        self.case_details.text = ""
        self.case_proof.text = ""
        self._refresh_buttons()
        self._gen = case.run(self.ctx)

    def _complete(self, result: CaseResult) -> None:
        if result is None:
            self._fail_current(CaseFailure("сценарий не вернул результат"))
            return
        self.results[result.cid] = result
        self.order.append(result.cid)
        row = self._rows[result.cid]
        row["state"].text = "PASS" if result.passed else "FAIL"
        row["state"].color = ui.GOOD if result.passed else ui.BAD
        self._show_result(result)
        self._pending_shot = (result.cid,
                              self.dir / "shots" / f"{result.cid}_result.png")
        self._gen = None
        self.current_cid = None
        self._refresh_all()
        self._write_report()

    def _fail_current(self, exc: Exception) -> None:
        cid = self.current_cid or "?"
        case = CASES.get(cid)
        result = CaseResult(
            cid=cid, title=case.title if case else "?",
            expected=case.expected if case else "",
            received=f"ошибка стенда: {exc}"[:400], passed=False,
            duration_s=time.time() - self._case_started,
            details=[str(exc)[:260]], proof={})
        self._complete(result)
        self.error = str(exc)

    def _finish(self) -> None:
        self.running = False
        self.finished = True
        self.current_cid = None
        passed = sum(1 for r in self.results.values() if r.passed)
        self._write_report()
        self.app.set_speed(self._speed_before)
        if self.ctx is not None:
            self.ctx.close()
        self.now_title.text = f"прогон завершён: {passed}/{len(self.results)} PASS"
        self.now_text.text = "F9 — закрыть, протокол сохранён"
        self._refresh_all()

    # ------------------------------------------------------------------ #
    def _refresh_buttons(self) -> None:
        busy = self.running
        ui.set_button(self.run_all_btn, not busy)
        ui.set_button(self.cancel_btn, busy)
        for cid in ORDER:
            ui.set_button(self._rows[cid]["button"], not busy)

    def _refresh_all(self) -> None:
        total = len(self.results)
        passed = sum(1 for r in self.results.values() if r.passed)
        self.counter.text = f"готово {total}/8 · PASS {passed}"
        self._refresh_buttons()

    def _show_result(self, result: CaseResult) -> None:
        self.case_title.text = f"{result.cid} · {result.title}"
        self.case_expected.text = self._wrap(f"ожидалось: {result.expected}", 44)
        self.case_received.text = self._wrap(
            f"получено: {result.received} · {result.duration_s:.1f} с", 44)
        lines: list[str] = []
        for item in result.details[:8]:
            lines.extend(textwrap.wrap(item, 50) or [""])
        self.case_details.text = "\n".join(lines[:9])
        proof_lines = [f"{name}: {Path(path).name}"
                       for name, path in result.proof.items() if path]
        self.case_proof.text = self._wrap(
            "доказательства:\n" + "\n".join(proof_lines), 46)
        map_path = result.proof.get("карта")
        if map_path and Path(map_path).exists():
            self._map_texture = Texture(str(map_path))
            self._map_entity.texture = self._map_texture
            self._map_entity.visible = True
        self.now_title.text = (f"{result.cid} · {'PASS' if result.passed else 'FAIL'}")
        self.now_text.text = result.received[:90]

    @staticmethod
    def _wrap(text: str, width: int) -> str:
        out: list[str] = []
        for line in text.splitlines():
            out.extend(textwrap.wrap(line, width) or [""])
        return "\n".join(out)

    # ------------------------------------------------------------------ #
    def _write_report(self) -> None:
        if self.dir is None:
            return
        results = [self.results[cid] for cid in ORDER if cid in self.results]
        metrics = {}
        if self.ctx is not None and self.ctx._mission is not None:
            try:
                metrics = compute_metrics(self.ctx)
            except Exception as exc:
                metrics = {"Метрики": f"не рассчитаны: {exc}"}
        try:
            path = write_protocol(self.dir, results, metrics)
            try:
                shown = path.relative_to(PROJECT_ROOT)
            except ValueError:
                shown = path
            self.status.text = f"протокол: {shown}"
        except Exception as exc:
            self.status.text = f"протокол не записан: {exc}"

# --------------------------------------------------------------------------- #
def run_cli(app, only: list[str] | None = None) -> int:
    """``app.py --tests``: run every scenario, print a table, exit with status."""
    bench = app.bench
    bench.open()
    bench.start_all(only)
    deadline = time.time() + 1800.0
    while bench.running and time.time() < deadline:
        app.ursina.step()
    if bench.running:
        bench.stop()
    results = [bench.results[cid] for cid in ORDER if cid in bench.results]
    print()
    print(f"{'№':<3} {'сценарий':<32} {'результат':<9} {'время':>7}")
    for result in results:
        print(f"{result.cid:<3} {result.title[:32]:<32} "
              f"{'PASS' if result.passed else 'FAIL':<9} {result.duration_s:7.1f}с")
    passed = sum(1 for r in results if r.passed)
    print(f"\nитог: {passed}/{len(results)} PASS")
    if bench.dir is not None:
        print(f"протокол: {bench.dir / 'protocol.md'}")
    if only:
        return 0 if passed == len(results) else 1
    return 0 if passed == len(ORDER) else 1
