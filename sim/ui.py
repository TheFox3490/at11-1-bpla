"""Small UI layer: Cyrillic-capable text, panels, buttons and helpers.

Ursina's bundled OpenSans font has no Cyrillic glyphs, so the application
copies Arial (present on every Windows install) into the asset folder and makes
it the default text font.  Everything else here is deliberately thin: the
modes own their content, this module only keeps the styling consistent.
"""
from __future__ import annotations

import math
import shutil
from pathlib import Path

from ursina import Button, Entity, Text, application, camera, color, mouse

PROJECT_ROOT = Path(__file__).resolve().parent.parent
INK = color.rgb32(232, 238, 246)
MUTED = color.rgb32(160, 172, 188)
ACCENT = color.rgb32(90, 170, 255)
GOOD = color.rgb32(96, 208, 132)
WARN = color.rgb32(240, 188, 78)
BAD = color.rgb32(238, 100, 100)

PANEL = color.rgba32(10, 15, 25, 218)
PANEL_SOFT = color.rgba32(20, 28, 42, 210)
BUTTON = color.rgb32(44, 66, 98)
BUTTON_ACTIVE = color.rgb32(58, 118, 92)
BUTTON_OFF = color.rgb32(50, 52, 60)
BUTTON_DANGER = color.rgb32(120, 60, 60)

FONT_NAME = "arial.ttf"


def configure_fonts() -> None:
    """Make a Cyrillic font the default for every Text created afterwards."""
    source = Path(r"C:\Windows\Fonts\arial.ttf")
    targets = {
        PROJECT_ROOT / "fonts",
        Path(getattr(application, "fonts_folder", PROJECT_ROOT / "fonts")),
    }
    for folder in targets:
        try:
            folder.mkdir(parents=True, exist_ok=True)
            dest = folder / FONT_NAME
            if source.exists() and not dest.exists():
                shutil.copy2(source, dest)
        except OSError:
            pass
    Text.default_font = FONT_NAME
    Text.default_resolution = 1080 * Text.size * 2


def panel(parent=None, position=(0, 0), scale=(0.4, 0.2), tint=PANEL, z=0.2, name=""):
    return Entity(parent=parent or camera.ui, model="quad", color=tint,
                  position=(position[0], position[1], z), scale=scale, name=name)


def label(parent, text, position, origin=(-0.5, 0.5), scale=0.66, tint=INK,
          background=False, z=-0.02):
    item = Text(text=text, parent=parent or camera.ui, position=(position[0], position[1], z),
                origin=origin, scale=scale, color=tint)
    if background:
        item.background = True
        item.background.color = color.rgba32(8, 12, 20, 205)
    return item


def button(parent, text, position, scale=(0.24, 0.046), on_click=None,
           tint=BUTTON, text_size=0.62, danger=False, name=""):
    item = Button(parent=parent or camera.ui, model="quad", text=text,
                  position=(position[0], position[1], -0.04), scale=scale,
                  color=tint, text_size=text_size, text_color=INK,
                  highlight_color=tint.tint(0.18), pressed_color=tint.tint(-0.18),
                  radius=0.015, on_click=on_click, name=name or text)
    if danger:
        item.color = BUTTON_DANGER
        item.highlight_color = BUTTON_DANGER.tint(0.18)
    return item


def set_button(item: Button, enabled: bool, tint: color.Color | None = None) -> None:
    item.disabled = not enabled
    base_tint = tint if tint is not None else BUTTON
    item.color = base_tint if enabled else BUTTON_OFF
    item.highlight_color = (base_tint if enabled else BUTTON_OFF).tint(0.18)
    if item.text_entity:
        item.text_entity.color = INK if enabled else color.rgb32(120, 124, 132)


def highlight_button(item: Button, active: bool, active_tint=BUTTON_ACTIVE) -> None:
    item.color = active_tint if active else BUTTON
    item.highlight_color = item.color.tint(0.18)
    item.text_entity.color = INK


def pointer_over_ui() -> bool:
    """True when the cursor is over a camera.ui widget (so a map click is not a map click)."""
    entity = getattr(mouse, "hovered_entity", None)
    while entity is not None:
        if entity is camera.ui:
            return True
        entity = getattr(entity, "parent", None)
    return False


def field_point_under_mouse() -> tuple[float, float] | None:
    """Intersect the pointer ray with the field plane y=0, in field coordinates."""
    point = getattr(mouse, "world_point", None)
    if point is not None:
        return float(point.x), float(point.z)

    # Analytical fallback: camera ray against y=0.
    origin = camera.world_position
    direction = camera.forward
    if abs(direction.y) < 1e-6:
        return None
    t = -origin.y / direction.y
    if t <= 0:
        return None
    hit = origin + direction * t
    return float(hit.x), float(hit.z)


def clamp_field(x: float, y: float) -> tuple[float, float]:
    from shared.world import FIELD_H, FIELD_W
    return max(0.05, min(FIELD_W - 0.05, x)), max(0.05, min(FIELD_H - 0.05, y))


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
