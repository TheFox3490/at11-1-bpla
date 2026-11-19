"""Mode 1: free flight with the observer camera and manual drone control."""
from __future__ import annotations

import math

from panda3d.core import WindowProperties
from ursina import Vec3, camera, held_keys, mouse

from shared.world import ALT_MAX, ALT_MIN, FIELD_H, FIELD_W, SPEED_LIMIT
from sim import ui
from sim.modes.base import Mode

OBSERVER_FOV = 90.0
OBSERVER_SPEED = 3.0
MOUSE_SENSITIVITY = 130.0
PITCH_LIMIT = 89.0
ALT_SPEED = 0.45
BOOST = 2.5


class FreeMode(Mode):
    name = "free"
    title = "1 · СВОБОДНЫЙ ПОЛЁТ"
    role = "free"

    def __init__(self, app):
        super().__init__(app)
        self.looking = False
        self.cursor_saved: tuple[int, int] | None = None
        self.look_geometry: tuple[int | None, int | None, int, int] | None = None
        self.look_delta: tuple[float, float] | None = None
        self.obs_pos = Vec3(8.6, 5.2, -3.2)
        self.obs_yaw = 0.0
        self.obs_pitch = 0.0
        self.payload = False
        self._build_ui()

    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        self.left_panel = self.add_ui(ui.panel(
            position=(-0.645, -0.02), scale=(0.43, 0.84), tint=ui.PANEL, name="free_panel"))
        self.add_ui(ui.label(
            None, "СВОБОДНЫЙ ПОЛЁТ", (-0.84, 0.375), origin=(-0.5, 0.5),
            scale=0.78, tint=ui.ACCENT))
        self.help_text = self.add_ui(ui.label(
            None,
            "КАМЕРА\n"
            "  WASD — движение\n"
            "  Space / Ctrl — вверх / вниз\n"
            "  ПКМ — вращать мышью\n"
            "  Shift — ускорение\n"
            "\n"
            "ДРОН\n"
            "  T / G — вперёд / назад (+y / -y)\n"
            "  H / F — вправо / влево (+x / -x)\n"
            "  R / V — выше / ниже\n"
            "  P — подсветка\n"
            "  Backspace — сброс дрона\n"
            "\n"
            "Режим 1 не используется в зачётном\n"
            "сценарии: это свободный полёт.\n",
            (-0.84, 0.305), origin=(-0.5, 0.5), scale=0.58, tint=ui.INK,
            max_width=0.39))
        self.status = self.add_ui(ui.label(
            None, "", (-0.84, -0.30), origin=(-0.5, 0.5),
            scale=0.60, tint=ui.WARN, max_width=0.39))

    # ------------------------------------------------------------------ #
    def on_enter(self) -> None:
        self.app.reset_drone("free", reset_led=True)
        self.obs_pos = Vec3(8.6, 5.2, -3.2)
        target = Vec3(3.0, 0.6, 2.0)
        d = target - self.obs_pos
        self.obs_yaw = math.degrees(math.atan2(d.x, d.z))
        self.obs_pitch = -math.degrees(math.atan2(d.y, math.hypot(d.x, d.z)))
        self.app.set_observer_fov(OBSERVER_FOV)
        camera.position = self.obs_pos
        camera.rotation = (self.obs_pitch, self.obs_yaw, 0)
        self.app.scene.set_overlays(True)
        self.app.scene.clear_trail()
        self.payload = False
        self._unlock()

    def on_exit(self) -> None:
        self._unlock()
        self.app.drone.stop()

    def _set_cursor_capture(self, active: bool) -> None:
        # Leave geometry to the window manager. Ursina's shared properties
        # can still contain the startup origin.
        props = WindowProperties()
        props.setCursorHidden(active)
        props.setMouseMode(WindowProperties.M_confined if active else WindowProperties.M_absolute)
        self.app.ursina.win.requestProperties(props)

    def _unlock(self, restore_cursor: bool = True) -> None:
        if not self.looking:
            return
        self.looking = False
        self.look_geometry = None
        self.look_delta = None
        self._set_cursor_capture(False)
        win = self.app.ursina.win
        if restore_cursor and self.cursor_saved and win.getProperties().getForeground():
            x, y = self.cursor_saved
            win.movePointer(0, min(x, win.getXSize() - 1), min(y, win.getYSize() - 1))
        self.cursor_saved = None

    def _lock(self) -> None:
        if self.looking or ui.pointer_over_ui():
            return
        win = self.app.ursina.win
        ptr = win.getPointer(0)
        if not ptr.in_window or not win.getProperties().getForeground():
            return
        self.cursor_saved = (int(ptr.getX()), int(ptr.getY()))
        self.looking = True
        self._set_cursor_capture(True)

    def input(self, key: str) -> None:
        if key == "right mouse down":
            self._lock()
        elif key == "right mouse up":
            self._unlock()
        elif key == "p":
            self.payload = not self.payload
            self.app.scene.rig.set_led(self.payload)
        elif key == "backspace":
            self.app.reset_drone("free", reset_led=True)
            self.payload = False

    def _apply_look(self, vx: float, vy: float) -> None:
        self.obs_yaw = (self.obs_yaw + vx * MOUSE_SENSITIVITY) % 360
        self.obs_pitch -= vy * MOUSE_SENSITIVITY
        self.obs_pitch = max(-PITCH_LIMIT, min(PITCH_LIMIT, self.obs_pitch))

    def update_frame(self) -> None:
        if not self.looking:
            return
        win = self.app.ursina.win
        props = win.getProperties()
        if not props.getForeground() or props.getMinimized() or not mouse.right:
            self._unlock(restore_cursor=False)
            return
        width, height = win.getSize()
        if width <= 0 or height <= 0:
            return
        center_x, center_y = width // 2, height // 2
        x, y = props.getOrigin() if props.hasOrigin() else (None, None)
        geometry = (x, y, width, height)
        if self.look_geometry != geometry:
            self.look_delta = None
            if win.movePointer(0, center_x, center_y):
                self.look_geometry = geometry
            return
        ptr = win.getPointer(0)
        if not ptr.in_window:
            self._unlock(restore_cursor=False)
            return
        if self.look_delta is not None:
            self._apply_look(*self.look_delta)
        self.look_delta = None
        dx, dy = ptr.getX() - center_x, center_y - ptr.getY()
        # The pointer can reflect a resize one frame before Panda reports it.
        # Commit the movement only if the geometry stayed stable.
        if (dx or dy) and win.movePointer(0, center_x, center_y):
            self.look_delta = (dx / height, dy / height)

    def step(self, dt: float) -> None:
        drone = self.app.drone
        yaw_r = math.radians(self.obs_yaw)
        pitch_r = math.radians(self.obs_pitch)
        forward = Vec3(math.sin(yaw_r) * math.cos(pitch_r),
                       -math.sin(pitch_r),
                       math.cos(yaw_r) * math.cos(pitch_r))
        right = Vec3(math.cos(yaw_r), 0, -math.sin(yaw_r))
        move = Vec3(0, 0, 0)
        if held_keys["w"]:
            move += forward
        if held_keys["s"]:
            move -= forward
        if held_keys["d"]:
            move += right
        if held_keys["a"]:
            move -= right
        if held_keys["space"]:
            move.y += 1
        if held_keys["left control"]:
            move.y -= 1
        boost = BOOST if held_keys["left shift"] else 1.0
        if move.length() > 0:
            self.obs_pos += move.normalized() * OBSERVER_SPEED * boost * dt
            self.obs_pos.x = max(-25.0, min(FIELD_W + 25.0, self.obs_pos.x))
            self.obs_pos.y = max(0.25, min(20.0, self.obs_pos.y))
            self.obs_pos.z = max(-25.0, min(FIELD_H + 25.0, self.obs_pos.z))
        camera.position = self.obs_pos
        camera.rotation = (self.obs_pitch, self.obs_yaw, 0)

        cmd = Vec3(0, 0, 0)
        if held_keys["t"]:
            cmd.z += 1
        if held_keys["g"]:
            cmd.z -= 1
        if held_keys["h"]:
            cmd.x += 1
        if held_keys["f"]:
            cmd.x -= 1
        if cmd.length() > 0:
            cmd = cmd.normalized() * SPEED_LIMIT * boost
        valt = 0.0
        if held_keys["r"]:
            valt += ALT_SPEED
        if held_keys["v"]:
            valt -= ALT_SPEED
        drone.set_manual(cmd.x, cmd.z, valt)
        drone.update(dt)

    def refresh(self) -> None:
        d = self.app.drone
        led = "ВКЛ" if self.payload else "выкл"
        self.status.text = (
            f"дрон  x={d.x:4.2f}  y={d.y:4.2f}  h={d.alt:4.2f}\n"
            f"скорость {math.hypot(d.vx, d.vy):4.2f} м/с   подсветка {led}\n"
            f"коридор высот {ALT_MIN:.1f}–{ALT_MAX:.1f} м\n"
            f"камера: наблюдатель (ПКМ — обзор)"
        )
