"""Mode 2: field layout editor (the polygon the drones will see).

The operator draws the plots that the simulator renders on the ground.  The
edits are a draft until "Применить к полю" is pressed; applying changes the
physical field and drops any mission built for the previous layout.

This is not a way to hand coordinates to the executor: mode 3 still derives
the mission only from the recon frames (task 1.2).  The manual mission button
stays as a training aid and is marked as such.
"""
from __future__ import annotations

import math
import textwrap
from datetime import timedelta
from pathlib import Path

from ursina import Entity, camera, color, destroy, mouse

from cv.validator import Point, Polygon, Target, validate_mission, validate_target
from shared.contracts import checksum_of, iso, new_id, utc_now, write_json
from shared.world import (
    FIELD_H, FIELD_W, MAP_TTL_SECONDS, NO_FLY, RECON_ERROR,
)
from sim import scenario, ui
from sim.modes.base import Mode

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
MANUAL_MISSIONS = PROJECT_ROOT / "missions" / "manual"

CLASS_COLORS = {"A": color.rgba32(226, 62, 62, 190),
                "B": color.rgba32(56, 104, 226, 190),
                "U": color.rgba32(190, 190, 190, 190)}
CLASS_NAMES = {"A": "A · треугольник", "B": "B · круг", "U": "U · не определено"}
FIELD_PAD = 0.27


class ManualMarker:
    """Small translucent handle drawn over the plot texture."""

    def __init__(self, root: Entity, obj: dict):
        self.obj = obj
        self.halo = Entity(parent=root, model="sphere",
                           scale=(0.50, 0.006, 0.50),
                           position=(obj["x"], 0.008, obj["y"]),
                           color=color.rgba32(248, 248, 248, 110), visible=False)
        self.disc = Entity(parent=root, model="sphere",
                           scale=(0.11, 0.010, 0.11),
                           position=(obj["x"], 0.012, obj["y"]),
                           color=CLASS_COLORS[obj["klass"]])
        self.core = Entity(parent=root, model="sphere", scale=0.035,
                           position=(obj["x"], 0.024, obj["y"]),
                           color=color.rgb32(20, 20, 20))

    def sync(self, selected: bool) -> None:
        x, y = self.obj["x"], self.obj["y"]
        self.halo.position = (x, 0.008, y)
        self.disc.position = (x, 0.012, y)
        self.core.position = (x, 0.024, y)
        self.disc.color = CLASS_COLORS.get(self.obj["klass"], color.white)
        self.halo.visible = selected
        self.halo.scale = (0.54, 0.006, 0.54) if selected else (0.50, 0.006, 0.50)

    def destroy(self) -> None:
        destroy(self.halo)
        destroy(self.disc)
        destroy(self.core)


class ManualMode(Mode):
    name = "manual"
    title = "2 · КОНСТРУКТОР ПОЛЯ"
    role = "neutral"

    def __init__(self, app):
        super().__init__(app)
        self.objects: list[dict] = scenario.copy_objects(app.layout)
        self.applied: list[dict] = scenario.copy_objects(app.layout)
        self.markers: dict[str, ManualMarker] = {}
        self.selected: dict | None = None
        self.current_class = "A"
        self.drag: dict | None = None
        self._was_left = False
        self._was_right = False
        self._hover_point: tuple[float, float] | None = None
        self._active = False
        self.dirty = False
        self.layout_errors: list[str] = []
        self.mission_path: str | None = None
        self.mission: dict | None = None
        self.mission_valid = False
        self.mission_errors: list[str] = []
        self.last_exec: str = ""
        self.validation_rows: list[dict] = []
        self.marker_root = Entity(name="manual_markers")
        self.marker_root.enabled = False
        self._build_ui()
        self._rebuild_markers()

    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        self.add_ui(ui.panel(position=(-0.645, -0.02), scale=(0.43, 0.84),
                             tint=ui.PANEL, name="manual_panel"))
        self.add_ui(ui.label(None, "КОНСТРУКТОР ПОЛЯ", (-0.84, 0.375),
                             scale=0.74, tint=ui.ACCENT))
        self.add_ui(ui.label(None, "правки меняют полигон\nдля режима 3 · координаты\nдаёт только CV",
                             (-0.84, 0.315), scale=0.48, tint=ui.WARN))
        self.add_ui(ui.label(None, "КЛАСС ОБЪЕКТА", (-0.84, 0.205),
                             scale=0.53, tint=ui.MUTED))

        self.class_buttons = {}
        for i, klass in enumerate("ABU"):
            b = ui.button(None, klass, (-0.80 + i * 0.155, 0.155),
                          scale=(0.14, 0.042), text_size=0.60,
                          on_click=lambda k=klass: self.set_class(k),
                          name=f"class_{klass}")
            self.class_buttons[klass] = self.add_ui(b)

        self.add_ui(ui.label(None, "ПРИОРИТЕТ / УДАЛЕНИЕ", (-0.84, 0.095),
                             scale=0.48, tint=ui.MUTED))
        self.add_ui(ui.button(None, "P −", (-0.80, 0.052), scale=(0.13, 0.038),
                              text_size=0.55, on_click=lambda: self.bump_priority(-1)))
        self.add_ui(ui.button(None, "P +", (-0.655, 0.052), scale=(0.13, 0.038),
                              text_size=0.55, on_click=lambda: self.bump_priority(+1)))
        self.add_ui(ui.button(None, "Удалить", (-0.50, 0.052), scale=(0.15, 0.038),
                              text_size=0.50, danger=True, on_click=self.delete_selected))

        self.add_ui(ui.label(None, "ПОЛИГОН ДЛЯ РЕЖИМА 3", (-0.84, 0.005),
                             scale=0.48, tint=ui.MUTED))
        self.apply_btn = self.add_ui(ui.button(
            None, "Применить к полю", (-0.645, -0.038),
            scale=(0.36, 0.045), text_size=0.58, tint=ui.BUTTON_ACTIVE,
            on_click=self.apply_to_field, name="manual_apply_btn"))
        self.add_ui(ui.button(None, "Базовая", (-0.795, -0.088),
                              scale=(0.13, 0.036), text_size=0.47,
                              on_click=self.load_demo_map))
        self.add_ui(ui.button(None, "С поля", (-0.645, -0.088),
                              scale=(0.13, 0.036), text_size=0.47,
                              on_click=self.revert_to_applied))
        self.add_ui(ui.button(None, "Очистить", (-0.495, -0.088),
                              scale=(0.145, 0.036), text_size=0.45,
                              danger=True, on_click=self.clear_map))

        self.add_ui(ui.button(None, "Проверить карту", (-0.645, -0.132),
                              scale=(0.36, 0.040), on_click=self.validate_map_ui))
        self.add_ui(ui.button(None, "Учебная миссия", (-0.645, -0.178),
                              scale=(0.36, 0.038), on_click=self.generate_mission))
        self.launch_btn = self.add_ui(ui.button(
            None, "Запустить исполнитель", (-0.645, -0.222),
            scale=(0.36, 0.038), on_click=self.launch_executor))
        self.status = self.add_ui(ui.label(None, "", (-0.84, -0.262), scale=0.50,
                                           tint=ui.INK))

        self.add_ui(ui.panel(position=(0.63, -0.02), scale=(0.46, 0.84),
                             tint=ui.PANEL_SOFT, name="manual_check_panel"))
        self.add_ui(ui.label(None, "ПРОВЕРКА КАРТЫ", (0.42, 0.375), scale=0.68,
                             tint=ui.ACCENT))
        self.check_text = self.add_ui(ui.label(None, "", (0.42, 0.325), scale=0.475,
                                               tint=ui.INK))

        self.add_ui(ui.label(
            None, "ЛКМ — поставить / выбрать и перетащить · ПКМ — удалить · "
            "1/2/3 — класс · синяя рамка — зона калибровки",
            (0.0, -0.467), origin=(-0.5, 0.5), scale=0.55, tint=ui.MUTED))

    # ------------------------------------------------------------------ #
    def on_enter(self) -> None:
        camera.position = (FIELD_W / 2, 7.2, FIELD_H / 2)
        camera.rotation = (90, 0, 0)
        camera.fov = 86.0
        mouse.locked = False
        self._active = True
        self.app.scene.set_overlays(True)
        self.app.scene.set_calibration_zone_visible(True)
        self.marker_root.enabled = True
        self.app.drone.reset()
        self.app.drone.rig.set_role("neutral")
        self.app.scene.clear_trail()
        self._preview()
        self._sync_markers()
        self.validate_map_ui()

    def on_exit(self) -> None:
        self._active = False
        self.marker_root.enabled = False
        self.app.scene.set_calibration_zone_visible(False)
        self.drag = None
        self._was_left = False
        # The draft is only a preview; the physical field is what was applied.
        self.app.scene.set_layout(self.applied)
        if self.app.execution is not None and self.app.execution_source is self:
            self.app.cancel_execution()

    def _preview(self) -> None:
        if self._active:
            self.app.scene.set_layout(self.objects)

    # ------------------------------------------------------------------ #
    def set_class(self, klass: str) -> None:
        self.current_class = klass
        for k, button in self.class_buttons.items():
            ui.highlight_button(button, k == klass)

    def add_object(self, x: float, y: float, klass: str | None = None) -> dict:
        klass = klass or self.current_class
        x = max(FIELD_PAD, min(FIELD_W - FIELD_PAD, x))
        y = max(FIELD_PAD, min(FIELD_H - FIELD_PAD, y))
        obj = {
            "id": scenario.next_id(self.objects),
            "klass": klass,
            "x": round(x, 3),
            "y": round(y, 3),
            "priority": max((o["priority"] for o in self.objects), default=0) + 1,
        }
        self.objects.append(obj)
        self.markers[obj["id"]] = ManualMarker(self.marker_root, obj)
        self.selected = obj
        self._touch()
        self._sync_markers()
        return obj

    def remove_object(self, obj: dict) -> None:
        marker = self.markers.pop(obj["id"], None)
        if marker:
            marker.destroy()
        self.objects = [o for o in self.objects if o is not obj]
        if self.selected is obj:
            self.selected = None
        self._touch()
        self._sync_markers()

    def clear_map(self) -> None:
        self._replace_draft([])
        self.last_exec = ""
        self.validate_map_ui()

    def load_demo_map(self) -> None:
        """Load the task's base layout as a draft (kept for --map-demo)."""
        self._replace_draft(scenario.default_objects())
        self.selected = None
        self.last_exec = ""
        self.validate_map_ui()

    def revert_to_applied(self) -> None:
        self._replace_draft(self.applied)
        self.selected = None
        self.last_exec = ""
        self.validate_map_ui()

    def _replace_draft(self, objects: list[dict]) -> None:
        for marker in self.markers.values():
            marker.destroy()
        self.markers.clear()
        self.objects = scenario.copy_objects(objects)
        for obj in self.objects:
            self.markers[obj["id"]] = ManualMarker(self.marker_root, obj)
        self.selected = None
        self._touch()

    def _same_as_applied(self) -> bool:
        def key(obj):
            return (str(obj["id"]), str(obj["klass"]), round(float(obj["x"]), 3),
                    round(float(obj["y"]), 3), int(obj.get("priority", 0)))
        return sorted(map(key, self.objects)) == sorted(map(key, self.applied))

    def _touch(self) -> None:
        self.dirty = not self._same_as_applied()
        self.layout_errors = []
        self.invalidate_mission()
        self._preview()

    def _rebuild_markers(self) -> None:
        for marker in self.markers.values():
            marker.destroy()
        self.markers.clear()
        for obj in self.objects:
            self.markers[obj["id"]] = ManualMarker(self.marker_root, obj)

    def _sync_markers(self) -> None:
        for marker in self.markers.values():
            marker.sync(self.selected is marker.obj)

    def nearest(self, point: tuple[float, float], radius: float = 0.24) -> dict | None:
        best, best_d = None, radius
        for obj in self.objects:
            d = math.hypot(obj["x"] - point[0], obj["y"] - point[1])
            if d < best_d:
                best, best_d = obj, d
        return best

    def _valid_point(self, point) -> bool:
        return 0.0 <= point[0] <= FIELD_W and 0.0 <= point[1] <= FIELD_H

    def step(self, dt: float) -> None:
        point = None if ui.pointer_over_ui() else ui.field_point_under_mouse()
        self._hover_point = point if point and self._valid_point(point) else None

        if mouse.left and not self._was_left:
            if self._hover_point is not None:
                hit = self.nearest(self._hover_point, 0.26)
                if hit is not None:
                    self.selected = hit
                    self.drag = hit
                else:
                    self.drag = self.add_object(*self._hover_point)
                self._sync_markers()
                self.validate_map_ui()
        elif mouse.left and self.drag is not None and self._hover_point is not None:
            self.drag["x"] = round(max(FIELD_PAD, min(FIELD_W - FIELD_PAD,
                                                    self._hover_point[0])), 3)
            self.drag["y"] = round(max(FIELD_PAD, min(FIELD_H - FIELD_PAD,
                                                    self._hover_point[1])), 3)
            self._sync_markers()
            self.validate_map_ui()

        if not mouse.left and self._was_left:
            if self.drag is not None:
                self._preview()
            self.drag = None

        if mouse.right and not self._was_right and self._hover_point is not None:
            hit = self.nearest(self._hover_point, 0.26)
            if hit is not None:
                self.remove_object(hit)
                self.validate_map_ui()

        self._was_left = bool(mouse.left)
        self._was_right = bool(mouse.right)

    def input(self, key: str) -> None:
        if key in ("1", "2", "3"):
            self.set_class({"1": "A", "2": "B", "3": "U"}[key])
        elif key == "delete":
            self.delete_selected()
        elif key == "v":
            self.validate_map_ui()
        elif key == "escape":
            self.selected = None
            self.drag = None
            self._sync_markers()

    # ------------------------------------------------------------------ #
    def _field_busy(self) -> str | None:
        work = self.app.modes.get("work")
        if self.app.execution is not None:
            return "идёт полёт: менять поле нельзя"
        if work is not None and work.phase in {"recon", "cv"}:
            return "идёт разведка: менять поле нельзя"
        return None

    def apply_to_field(self) -> None:
        busy = self._field_busy()
        if busy:
            self.layout_errors = [busy]
            self.validate_map_ui()
            return
        errors = self.app.apply_layout(self.objects)
        if errors:
            self.layout_errors = errors
            self.last_exec = ""
            self.validate_map_ui()
            return
        self.applied = scenario.copy_objects(self.objects)
        self.dirty = False
        self.layout_errors = []
        self.last_exec = ("поле обновлено: режим 3 полетит по этой карте"
                          if not scenario.is_default(self.objects)
                          else "восстановлена базовая раскладка")
        self.validate_map_ui()

    def _evaluate(self) -> tuple[list[dict], list[dict]]:
        zones = [Polygon("NFZ-1", tuple(Point(x, y) for x, y in NO_FLY))]
        accepted, rows = [], []
        for obj in sorted(self.objects, key=lambda o: (o["priority"], o["id"])):
            reasons: list[str]
            if obj["klass"] != "A":
                reasons = ["class_not_A_in_official_mission"]
            else:
                target = Target(obj["id"], obj["id"], obj["klass"],
                                Point(obj["x"], obj["y"]), RECON_ERROR)
                reasons = validate_target(target, zones)
            row = {"id": obj["id"], "klass": obj["klass"],
                   "x": obj["x"], "y": obj["y"], "reasons": reasons}
            rows.append(row)
            if not reasons:
                accepted.append(obj)
        self.validation_rows = rows
        return accepted, rows

    def validate_map_ui(self) -> None:
        accepted, rows = self._evaluate()
        counts = {k: sum(1 for o in self.objects if o["klass"] == k) for k in "ABU"}
        lines = [
            f"объектов: {len(self.objects)}",
            f"A={counts['A']}  B={counts['B']}  U={counts['U']}",
            f"допустимых A: {len(accepted)}",
            "",
        ]
        if self.selected:
            o = self.selected
            lines += [f"выбран {o['id']}  класс {o['klass']}",
                      f"x={o['x']:.3f}  y={o['y']:.3f}  P={o['priority']}", ""]
        else:
            lines += ["объект не выбран", ""]
        layout_errors = scenario.validate(self.objects) if self.objects else ["карта пуста"]
        self.layout_errors = layout_errors
        if layout_errors:
            lines.append("ПОЛИГОН: нельзя применить")
            lines += [f"· {err}" for err in layout_errors[:4]]
            lines.append("")
        else:
            lines.append("ПОЛИГОН: проверка пройдена")
            lines.append("")
        for row in rows:
            if row["reasons"] and not layout_errors:
                lines.append(f"{row['id']} {row['klass']} — {', '.join(row['reasons'])}")
        if self.mission_path:
            lines += ["", "УЧЕБНАЯ МИССИЯ",
                      f"целей: {len(self.mission.get('targets', [])) if self.mission else 0}",
                      f"файл: {Path(self.mission_path).name}",
                      "статус: " + ("ГОТОВА" if self.mission_valid else "НЕ ГОТОВА")]
        if self.last_exec:
            lines += ["", self.last_exec]
        wrapped: list[str] = []
        for line in lines:
            wrapped.extend(textwrap.wrap(line, 54) or [""])
        self.check_text.text = "\n".join(wrapped[:34])
        applied_state = "применено" if not self.dirty else "есть правки"
        extra = self.layout_errors[0] if self.layout_errors else ""
        self.status.text = (
            f"класс: {CLASS_NAMES[self.current_class]}\n"
            f"поле: {applied_state} · объектов: {len(self.objects)}\n"
            f"допустимых A: {len(accepted)}"
            + (f"\n{extra[:46]}" if extra else "")
        )
        ui.set_button(self.launch_btn, bool(self.mission_path and self.mission_valid))
        apply_ready = (not self.layout_errors and self.dirty
                       and self._field_busy() is None)
        ui.set_button(self.apply_btn, apply_ready, tint=ui.BUTTON_ACTIVE)

    def invalidate_mission(self) -> None:
        if self.mission_path is not None:
            self.mission_path = None
            self.mission = None
            self.mission_valid = False
            self.mission_errors = []
            self.last_exec = ""

    def delete_selected(self) -> None:
        if self.selected is not None:
            self.remove_object(self.selected)
            self.validate_map_ui()

    def bump_priority(self, delta: int) -> None:
        if self.selected is None:
            return
        order = sorted(self.objects, key=lambda o: (o["priority"], o["id"]))
        index = order.index(self.selected)
        target = max(0, min(len(order) - 1, index + delta))
        order.insert(target, order.pop(index))
        for i, obj in enumerate(order, 1):
            obj["priority"] = i
        self._touch()
        self.validate_map_ui()

    # ------------------------------------------------------------------ #
    def generate_mission(self) -> None:
        accepted, rows = self._evaluate()
        if not accepted:
            self.mission_valid = False
            self.mission_errors = ["нет допустимых целей A"]
            self.status.text = "миссия не создана: нет целей A"
            return
        created = utc_now()
        mission_id = new_id("manual_mission")
        targets = []
        for i, obj in enumerate(accepted, 1):
            targets.append({
                "target_id": f"T{i:02d}",
                "action_id": f"{mission_id}-T{i:02d}",
                "klass": "A",
                "point": {"x": obj["x"], "y": obj["y"]},
                "position_error_m": RECON_ERROR,
                "priority": i,
                "source_object_id": obj["id"],
            })
        rejected = [{"object_id": row["id"], "class": row["klass"],
                     "x": row["x"], "y": row["y"], "reasons": row["reasons"]}
                    for row in rows if row["reasons"]]
        payload = {
            "mission_id": mission_id,
            "coordinate_frame": "F",
            "units": "m",
            "created_at": iso(created),
            "valid_until": iso(created + timedelta(seconds=MAP_TTL_SECONDS)),
            "calibration_version": "manual_training",
            "source": "manual_map_training",
            "field": {"width_m": FIELD_W, "height_m": FIELD_H},
            "targets": targets,
            "forbidden": [{"zone_id": "NFZ-1",
                           "vertices": [{"x": x, "y": y} for x, y in NO_FLY]}],
            "rejected": rejected,
            "note": "training only: coordinates were entered by hand",
        }
        payload["checksum"] = checksum_of(payload)
        MANUAL_MISSIONS.mkdir(parents=True, exist_ok=True)
        path = MANUAL_MISSIONS / f"{mission_id}.json"
        write_json(path, payload)
        self.mission = payload
        self.mission_path = str(path)
        self.mission_errors = validate_mission(payload)
        self.mission_valid = not self.mission_errors
        if not self.mission_valid:
            self.mission_path = None
        self.last_exec = ""
        self.validate_map_ui()

    def launch_executor(self) -> None:
        if not self.mission_path or not self.mission_valid:
            return
        self.last_exec = "исполнитель запущен"
        self.app.drone.rig.set_role("executor")
        self.app.scene.clear_trail()
        self.app.start_executor(self.mission_path, source=self)

    def on_execution_done(self, run) -> None:
        if run.failed:
            self.last_exec = f"ошибка исполнителя: {run.error}"
        else:
            confirmed = sum(1 for a in run.actions if a.get("status") == "confirmed")
            total = len(run.plan.get("targets", [])) if run.plan else 0
            self.last_exec = f"исполнитель завершил: {confirmed}/{total} подтверждено"
        self.validate_map_ui()

    def refresh(self) -> None:
        accepts = self.mission_path is not None and self.mission_valid
        ui.set_button(self.launch_btn, accepts and self.app.execution is None)
        for klass, button in self.class_buttons.items():
            ui.highlight_button(button, klass == self.current_class)
