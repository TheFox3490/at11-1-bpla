"""Mode 2: full manual map work (training only).

Everything here is deliberately manual: the operator places, moves, prioritises
and deletes objects, validates the map and generates a mission.  The banner in
the panel states that hand-entered coordinates are not allowed in the official
scenario; mode 3 is the official recon -> CV -> executor path.
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
from sim import ui
from sim.modes.base import Mode

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
MANUAL_MISSIONS = PROJECT_ROOT / "missions" / "manual"

CLASS_COLORS = {"A": color.rgb32(226, 62, 62),
                "B": color.rgb32(56, 104, 226),
                "U": color.rgb32(190, 190, 190)}
CLASS_NAMES = {"A": "A · треугольник", "B": "B · круг", "U": "U · не определено"}


class ManualMarker:
    def __init__(self, root: Entity, obj: dict):
        self.obj = obj
        self.ring = Entity(parent=root, model="sphere",
                           scale=(0.58, 0.012, 0.58),
                           position=(obj["x"], 0.026, obj["y"]),
                           color=color.rgb32(248, 248, 248), visible=False)
        self.disc = Entity(parent=root, model="sphere",
                           scale=(0.38, 0.018, 0.38),
                           position=(obj["x"], 0.032, obj["y"]),
                           color=CLASS_COLORS[obj["klass"]])
        self.core = Entity(parent=root, model="sphere", scale=0.055,
                           position=(obj["x"], 0.055, obj["y"]),
                           color=color.rgb32(20, 20, 20))

    def sync(self, selected: bool) -> None:
        x, y = self.obj["x"], self.obj["y"]
        self.ring.position = (x, 0.026, y)
        self.disc.position = (x, 0.032, y)
        self.core.position = (x, 0.055, y)
        self.disc.color = CLASS_COLORS.get(self.obj["klass"], color.white)
        self.ring.visible = selected
        self.ring.scale = (0.62, 0.012, 0.62) if selected else (0.58, 0.012, 0.58)

    def destroy(self) -> None:
        destroy(self.ring)
        destroy(self.disc)
        destroy(self.core)


class ManualMode(Mode):
    name = "manual"
    title = "2 · РУЧНАЯ КАРТА"
    role = "neutral"

    def __init__(self, app):
        super().__init__(app)
        self.objects: list[dict] = []
        self.markers: dict[str, ManualMarker] = {}
        self.next_index = 1
        self.selected: dict | None = None
        self.current_class = "A"
        self.drag: dict | None = None
        self._was_left = False
        self._was_right = False
        self._hover_point: tuple[float, float] | None = None
        self.mission_path: str | None = None
        self.mission: dict | None = None
        self.mission_valid = False
        self.mission_errors: list[str] = []
        self.last_exec: str = ""
        self.validation_rows: list[dict] = []
        self.marker_root = Entity(name="manual_markers")
        self.marker_root.enabled = False
        self._build_ui()

    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        self.add_ui(ui.panel(position=(-0.645, -0.02), scale=(0.43, 0.84),
                             tint=ui.PANEL, name="manual_panel"))
        self.add_ui(ui.label(None, "РУЧНАЯ КАРТА", (-0.84, 0.375),
                             scale=0.78, tint=ui.ACCENT))
        self.add_ui(ui.label(None, "УЧЕБНЫЙ РЕЖИМ\nручные координаты недопустимы\nв официальном сценарии",
                             (-0.84, 0.315), scale=0.50, tint=ui.WARN))
        self.add_ui(ui.label(None, "КЛАСС ОБЪЕКТА", (-0.84, 0.205),
                             scale=0.55, tint=ui.MUTED))

        self.class_buttons = {}
        for i, klass in enumerate("ABU"):
            b = ui.button(None, klass, (-0.80 + i * 0.155, 0.155),
                          scale=(0.14, 0.042), text_size=0.60,
                          on_click=lambda k=klass: self.set_class(k),
                          name=f"class_{klass}")
            self.class_buttons[klass] = self.add_ui(b)

        self.add_ui(ui.label(None, "ПРИОРИТЕТ И УДАЛЕНИЕ", (-0.84, 0.09),
                             scale=0.50, tint=ui.MUTED))
        self.add_ui(ui.button(None, "P −", (-0.80, 0.045), scale=(0.14, 0.038),
                              text_size=0.55, on_click=lambda: self.bump_priority(-1)))
        self.add_ui(ui.button(None, "P +", (-0.645, 0.045), scale=(0.14, 0.038),
                              text_size=0.55, on_click=lambda: self.bump_priority(+1)))
        self.add_ui(ui.button(None, "Удалить", (-0.49, 0.045), scale=(0.15, 0.038),
                              text_size=0.52, danger=True, on_click=self.delete_selected))

        self.add_ui(ui.button(None, "Проверить карту", (-0.645, -0.02),
                              scale=(0.36, 0.043), on_click=self.validate_map_ui))
        self.add_ui(ui.button(None, "Сгенерировать миссию", (-0.645, -0.075),
                              scale=(0.36, 0.043), on_click=self.generate_mission))
        self.launch_btn = self.add_ui(ui.button(
            None, "Запустить исполнитель", (-0.645, -0.13),
            scale=(0.36, 0.043), on_click=self.launch_executor))
        self.add_ui(ui.button(None, "Очистить карту", (-0.645, -0.185),
                              scale=(0.36, 0.038), danger=True,
                              on_click=self.clear_map))
        self.status = self.add_ui(ui.label(None, "", (-0.84, -0.235), scale=0.52,
                                           tint=ui.INK))

        self.add_ui(ui.panel(position=(0.63, -0.02), scale=(0.46, 0.84),
                             tint=ui.PANEL_SOFT, name="manual_check_panel"))
        self.add_ui(ui.label(None, "ПРОВЕРКА КАРТЫ", (0.42, 0.375), scale=0.68,
                             tint=ui.ACCENT))
        self.check_text = self.add_ui(ui.label(None, "", (0.42, 0.325), scale=0.475,
                                               tint=ui.INK))

        self.add_ui(ui.label(
            None, "ЛКМ — поставить / выбрать и перетащить · ПКМ — удалить · 1/2/3 — класс",
            (0.0, -0.467), origin=(-0.5, 0.5), scale=0.55, tint=ui.MUTED))

    # ------------------------------------------------------------------ #
    def on_enter(self) -> None:
        camera.position = (FIELD_W / 2, 7.2, FIELD_H / 2)
        camera.rotation = (90, 0, 0)
        camera.fov = 86.0
        mouse.locked = False
        self.app.scene.set_overlays(True)
        self.marker_root.enabled = True
        self.app.drone.reset()
        self.app.drone.rig.set_role("neutral")
        self.app.scene.clear_trail()
        self._sync_markers()
        self.validate_map_ui()

    def on_exit(self) -> None:
        self.marker_root.enabled = False
        self.drag = None
        self._was_left = False
        if self.app.execution is not None and self.app.execution_source is self:
            self.app.cancel_execution()

    # ------------------------------------------------------------------ #
    def set_class(self, klass: str) -> None:
        self.current_class = klass
        for k, button in self.class_buttons.items():
            ui.highlight_button(button, k == klass)

    def add_object(self, x: float, y: float, klass: str | None = None) -> dict:
        klass = klass or self.current_class
        x, y = ui.clamp_field(x, y)
        obj = {
            "id": f"M{self.next_index:02d}",
            "klass": klass,
            "x": round(x, 3),
            "y": round(y, 3),
            "priority": max((o["priority"] for o in self.objects), default=0) + 1,
        }
        self.next_index += 1
        self.objects.append(obj)
        self.markers[obj["id"]] = ManualMarker(self.marker_root, obj)
        self.selected = obj
        self.invalidate_mission()
        self._sync_markers()
        return obj

    def remove_object(self, obj: dict) -> None:
        marker = self.markers.pop(obj["id"], None)
        if marker:
            marker.destroy()
        self.objects = [o for o in self.objects if o is not obj]
        if self.selected is obj:
            self.selected = None
        self.invalidate_mission()
        self._sync_markers()

    def clear_map(self) -> None:
        for marker in self.markers.values():
            marker.destroy()
        self.markers.clear()
        self.objects.clear()
        self.selected = None
        self.next_index = 1
        self.invalidate_mission()
        self.last_exec = ""
        self._sync_markers()
        self.validate_map_ui()

    def load_demo_map(self) -> None:
        for x, y, k in [
            (2.1, 0.7, "A"), (3.0, 1.4, "A"), (4.0, 0.7, "A"),
            (4.2, 2.2, "A"), (1.6, 2.3, "A"), (2.6, 3.2, "A"),
            (3.6, 2.4, "B"), (3.2, 3.5, "U"), (5.1, 2.1, "A"),
        ]:
            self.add_object(x, y, k)
        self.selected = None
        self._sync_markers()
        self.validate_map_ui()

    def _sync_markers(self) -> None:
        for oid, marker in self.markers.items():
            marker.sync(self.selected is marker.obj)

    def nearest(self, point: tuple[float, float], radius: float = 0.22) -> dict | None:
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
                hit = self.nearest(self._hover_point, 0.24)
                if hit is not None:
                    self.selected = hit
                    self.drag = hit
                else:
                    self.drag = self.add_object(*self._hover_point)
                self._sync_markers()
                self.validate_map_ui()
        elif mouse.left and self.drag is not None and self._hover_point is not None:
            self.drag["x"] = round(self._hover_point[0], 3)
            self.drag["y"] = round(self._hover_point[1], 3)
            self._sync_markers()
            self.validate_map_ui()

        if not mouse.left and self._was_left:
            self.drag = None

        if mouse.right and not self._was_right and self._hover_point is not None:
            hit = self.nearest(self._hover_point, 0.24)
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
        elif key == "enter":
            self.generate_mission()
        elif key == "v":
            self.validate_map_ui()
        elif key == "escape":
            self.selected = None
            self.drag = None
            self._sync_markers()

    # ------------------------------------------------------------------ #
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
        for row in rows:
            if row["reasons"]:
                lines.append(f"{row['id']} {row['klass']} — {', '.join(row['reasons'])}")
        if self.mission_path:
            lines += ["", "МИССИЯ СГЕНЕРИРОВАНА",
                      f"целей: {len(self.mission.get('targets', [])) if self.mission else 0}",
                      f"файл: {Path(self.mission_path).name}",
                      "статус: " + ("ГОТОВА" if self.mission_valid else "НЕ ГОТОВА")]
        if self.last_exec:
            lines += ["", self.last_exec]
        wrapped: list[str] = []
        for line in lines:
            wrapped.extend(textwrap.wrap(line, 54) or [""])
        self.check_text.text = "\n".join(wrapped[:32])
        self.status.text = (
            f"класс: {CLASS_NAMES[self.current_class]}\n"
            f"объектов: {len(self.objects)}   допустимых A: {len(accepted)}\n"
            f"миссия: {'готова' if self.mission_valid else 'не готова'}"
        )
        ui.set_button(self.launch_btn, bool(self.mission_path and self.mission_valid))

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
        self.invalidate_mission()
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
