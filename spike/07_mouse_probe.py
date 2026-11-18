"""Probe: does the pointer report movement while the mouse is locked?

Simulates "user moved the mouse" by warping the OS pointer, then reads back
base.win.getPointer(0) and ursina.mouse.velocity, both locked and unlocked.
"""
from ursina import Ursina, camera, color, mouse, window

app = Ursina(size=(800, 600), development_mode=False, editor_ui_enabled=False,
             show_ursina_splash=False)
window.color = color.black
camera.position = (0, 0, -20)

W, H = base.win.getXSize(), base.win.getYSize()
CX, CY = W // 2, H // 2
print(f"window {W}x{H}, centre {CX},{CY}")
print("pointer initially:", base.win.getPointer(0).getX(), base.win.getPointer(0).getY())


def warp(dx, dy):
    base.win.movePointer(0, CX + dx, CY + dy)


def show(tag):
    p = base.win.getPointer(0)
    print(f"  {tag}: pointer=({p.getX()},{p.getY()}) in_window={p.in_window} "
          f"mouse.velocity=({mouse.velocity[0]:+.4f},{mouse.velocity[1]:+.4f}) "
          f"mouse.pos=({mouse.position[0]:+.4f},{mouse.position[1]:+.4f})")


print("\n--- unlocked ---")
for dx in (50, 100):
    warp(dx, 0)
    app.step()
    show(f"warp +{dx}")

print("\n--- locked (M_confined, as in demo) ---")
mouse.locked = True
app.step()
for dx in (50, 100, 200):
    warp(dx, 0)
    app.step()
    show(f"warp +{dx}")
mouse.locked = False
app.step()

print("\n--- locked, then re-centre manually ---")
mouse.locked = True
app.step()
for dx in (50, 100):
    warp(dx, 0)
    app.step()
    p = base.win.getPointer(0)
    print(f"  before recentre: {p.getX()},{p.getY()}")
    warp(0, 0)
    app.step()
    p = base.win.getPointer(0)
    print(f"  after  recentre: {p.getX()},{p.getY()}")
mouse.locked = False

print("\nmouse mode:", window.mouse_mode)
base.userExit()
