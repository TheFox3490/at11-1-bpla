"""Probe: Cyrillic font + Ursina button/text sizes."""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from panda3d.core import Filename, PNMImage
from ursina import Button, Entity, Text, Ursina, camera, color, window

FONT_DIR = ROOT / "fonts"
FONT_DIR.mkdir(exist_ok=True)
font = FONT_DIR / "arial.ttf"
if not font.exists():
    shutil.copy2(r"C:\Windows\Fonts\arial.ttf", font)

app = Ursina(size=(1280, 720), borderless=False, development_mode=False,
             editor_ui_enabled=False, show_ursina_splash=False)
window.color = color.rgb32(30, 34, 42)
Text.default_font = "arial.ttf"

Entity(parent=camera.ui, model="quad", color=color.rgba32(15, 22, 34, 230),
       position=(0, 0.44), scale=(1.7, 0.06), z=0.05)

for i, (label, size) in enumerate([("СВОБОДНЫЙ ПОЛЁТ", 1.0), ("КАРТА", .7), ("РАЗВЕДКА", .5)]):
    Button(parent=camera.ui, model="quad", text=label,
           position=(-0.78 + i * 0.31, 0.44), scale=(0.29, 0.05),
           color=color.rgb32(40, 70, 120), text_size=size,
           text_color=color.white, radius=0.02)

Text(text="Режим 3 · Отчёт\nРазведка завершена: 10 объектов\nA=6  B=1  U=1",
     position=(-0.85, 0.30), origin=(-0.5, 0.5), scale=0.7,
     color=color.rgb32(235, 240, 250), background=True).background.color = color.rgba32(10, 15, 25, 210)
Text(text="Режим 3 · Отчёт", position=(0.2, 0.30), origin=(-0.5, 0.5), scale=1.0,
     color=color.rgb32(235, 240, 250))

for _ in range(4):
    app.step()
out = ROOT / "spike" / "out" / "ui_probe.png"
pnm = PNMImage()
base.win.getScreenshot(pnm)
pnm.write(Filename.fromOsSpecific(str(out.resolve())))
print("saved", out)
base.userExit()
