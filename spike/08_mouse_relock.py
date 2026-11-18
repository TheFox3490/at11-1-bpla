"""Probe: is there a camera jump when you re-grab the mouse (RMB after release)?

Drives the real demo module through the same paths Ursina uses: __main__.input
and __main__.update are forwarded, so the frame order (mouse.update -> update)
is identical to the game.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import sim.demo as demo  # builds the app and the scene

app = demo.app
W, H = base.win.getXSize(), base.win.getYSize()
CX, CY = W // 2, H // 2


def input(key):        # noqa: A001 - ursina dispatches to __main__.input
    demo.input(key)


def update():
    demo.update()


demo.mouse.locked = False
demo.State.looking = False


def frame(tag: str) -> None:
    yaw0, pitch0 = demo.State.obs_yaw, demo.State.obs_pitch
    app.step()
    dyaw = (demo.State.obs_yaw - yaw0 + 180) % 360 - 180
    print(f"    {tag:<28} vel=({demo.mouse.velocity[0]:+.4f},{demo.mouse.velocity[1]:+.4f}) "
          f"dyaw={dyaw:+7.2f} "
          f"dpitch={demo.State.obs_pitch - pitch0:+7.2f}")


app.step()

print("1) press RMB, drag right 120 px")
app.input("right mouse down")
frame("lock frame")
base.win.movePointer(0, CX + 120, CY)
frame("drag +120px")
app.input("right mouse up")
frame("release")

print("\n2) move the cursor away, wander around, no looking")
base.win.movePointer(0, 60, 60)
for i in range(4):
    frame(f"unlocked {i}")

print("\n3) press RMB again WITHOUT moving the mouse, then nudge it 2 px")
app.input("right mouse down")
for i in range(2):
    frame(f"relock {i}")
base.win.movePointer(0, CX - 2, CY)   # 2 px nudge from the centre
frame("nudge 2px after relock")
app.input("right mouse up")
frame("release")

print("\n4) press RMB while the cursor is in the far corner, then nudge 2 px")
base.win.movePointer(0, W - 80, H - 60)
frame("moved to corner")
app.input("right mouse down")
for i in range(2):
    frame(f"corner relock {i}")
base.win.movePointer(0, CX + 2, CY)
frame("nudge 2px from centre")
app.input("right mouse up")

base.userExit()
