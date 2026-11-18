"""Minimal probe: Ursina renders, and a downward view can be saved as PNG.

The whole capture approach rests on this working, so it is kept as the
smallest possible executable proof.

Run:  .venv\\Scripts\\python.exe spike\\00_probe_capture.py
"""
from pathlib import Path

from panda3d.core import Filename, PNMImage
from ursina import Entity, Ursina, camera, color, window

OUT = Path(__file__).with_name("out")
OUT.mkdir(exist_ok=True)

app = Ursina(
    title="probe",
    size=(640, 480),
    development_mode=False,
    editor_ui_enabled=False,
    show_ursina_splash=False,
)
window.color = color.black

# Ursina space: X right, Y up, Z forward -> the field lies in the XZ plane.
Entity(
    model="quad",
    color=color.rgb32(112, 86, 58),
    scale=(6, 4),
    position=(3, 0, 2),
    rotation_x=90,
    double_sided=True,
)
Entity(
    model="quad",
    color=color.rgb32(210, 30, 30),
    scale=0.5,
    position=(3, 0.01, 2),
    rotation_x=90,
    double_sided=True,
)

# drone camera: 1.2 m above the field, looking straight down
camera.position = (3, 1.2, 2)
camera.rotation = (90, 0, 0)
camera.fov = 70
camera.clip_plane_far = 100

for _ in range(4):          # couple of frames so textures/shaders are ready
    app.step()

shot = OUT / "probe.png"
pnm = PNMImage()            # Panda3D 1.10.16 wants a PNMImage, not a Filename
base.win.getScreenshot(pnm)
pnm.write(Filename.fromOsSpecific(str(shot)))

print("saved:", shot, shot.stat().st_size, "bytes")
base.userExit()
