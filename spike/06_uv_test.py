"""Which way does a textured quad lie after rotation_x=90?"""
from pathlib import Path

from PIL import Image, ImageDraw
from panda3d.core import Filename, PNMImage
from ursina import Entity, Texture, Ursina, camera, color, window

OUT = Path(__file__).with_name("out")
OUT.mkdir(exist_ok=True)

img = Image.new("RGB", (256, 256), (240, 240, 240))
d = ImageDraw.Draw(img)
d.rectangle([0, 0, 127, 255], fill=(220, 40, 40))     # left half red
d.rectangle([128, 0, 255, 127], fill=(40, 40, 220))   # top-right blue
img.save(OUT / "uv_test.png")

app = Ursina(size=(480, 360), development_mode=False, editor_ui_enabled=False,
             show_ursina_splash=False)
window.color = color.black

Entity(model="quad", texture=Texture(OUT / "uv_test.png"), scale=(2, 2),
       position=(0, 0, 0), rotation_x=90, double_sided=True)

# overhead camera, image up should be +field y if everything is consistent
camera.position = (0, 5, 0)
camera.rotation = (90, 0, 0)
camera.fov = 40
for _ in range(4):
    app.step()

pnm = PNMImage()
base.win.getScreenshot(pnm)
pnm.write(Filename.fromOsSpecific(str(OUT / "uv_test_render.png")))
print("saved", OUT / "uv_test_render.png")
base.userExit()
