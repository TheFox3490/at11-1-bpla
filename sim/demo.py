"""Demo scene: a flyable drone over the 6 x 4 m training field.

Two independent things are controlled:
  * the observer camera (Minecraft-creative style, WASD + mouse while RMB held)
  * the drone itself (TFGH, altitude on R/V)

Run:
  .venv\\Scripts\\python.exe sim\\demo.py
  .venv\\Scripts\\python.exe sim\\demo.py --shot out\\demo.png   # headless-ish check
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from panda3d.core import Filename, PNMImage  # noqa: E402
from ursina import (  # noqa: E402
    Entity, Mesh, Sky, Text, Texture, Ursina, Vec3, camera, color, held_keys, mouse,
    scene, time, window,
)

from sim.field import (  # noqa: E402
    ALT_CORRIDOR, ALT_NOMINAL, CALIB_MARKERS, CHECK_MARKERS, FIELD_H, FIELD_W,
    HFOV, MARKER_SIZE, NO_FLY, PLOTS, PLOT_SIZE, SPEED_LIMIT, plot_texture_paths,
    validate_layout,
)

# --------------------------------------------------------------------------- #
# tunables
# --------------------------------------------------------------------------- #
OBSERVER_FOV = 90.0
OBSERVER_SPEED = 3.0          # m/s, it is just a camera
# Degrees per unit of ursina's mouse.velocity (1.0 = one window width).
MOUSE_SENSITIVITY = 130.0
PITCH_LIMIT = 89.0
DRONE_ACCEL = 8.0
ALT_SPEED = 0.45              # m/s
BOOST = 2.5

# --------------------------------------------------------------------------- #
# scene
# --------------------------------------------------------------------------- #
validate_layout()
app = Ursina(
    title="BPLA demo - observer camera + drone",
    size=(1280, 720),
    borderless=False,
    development_mode=False,
    editor_ui_enabled=False,
    show_ursina_splash=False,
)
window.color = color.rgb32(128, 176, 222)
scene.fog_color = color.rgb32(150, 190, 225)
scene.fog_density = 0.012

Sky()

# ground far beyond the field, so the world does not look like a floating plate
Entity(
    model="plane",
    scale=(60, 1, 60),
    color=color.rgb32(104, 132, 74),
    position=(FIELD_W / 2, -0.02, FIELD_H / 2),
)

# the study field itself: soil block
Entity(
    model="quad",
    color=color.rgb32(112, 86, 58),
    scale=(FIELD_W, FIELD_H),
    position=(FIELD_W / 2, 0, FIELD_H / 2),
    rotation_x=90,
    double_sided=True,
)

# 1 m grid: thin bars, cheap and exactly aligned with frame F
overlays: list[Entity] = []
for gx in range(1, int(FIELD_W)):
    overlays.append(Entity(model="cube", color=color.rgba32(255, 255, 255, 42),
                           scale=(0.012, 0.002, FIELD_H), position=(gx, 0.003, FIELD_H / 2)))
for gy in range(1, int(FIELD_H)):
    overlays.append(Entity(model="cube", color=color.rgba32(255, 255, 255, 42),
                           scale=(FIELD_W, 0.002, 0.012), position=(FIELD_W / 2, 0.003, gy)))

# field border
for x, z, sx, sz in [
    (FIELD_W / 2, -0.02, FIELD_W, 0.04),
    (FIELD_W / 2, FIELD_H + 0.02, FIELD_W, 0.04),
    (-0.02, FIELD_H / 2, 0.04, FIELD_H),
    (FIELD_W + 0.02, FIELD_H / 2, 0.04, FIELD_H),
]:
    Entity(model="cube", color=color.rgb32(235, 235, 235),
           scale=(sx, 0.01, sz), position=(x, 0.005, z))

# plots
plot_textures = plot_texture_paths()
for pid, cls, (x, y) in PLOTS:
    Entity(
        model="quad",
        texture=Texture(plot_textures[pid]),
        scale=PLOT_SIZE,
        position=(x, 0.004, y),
        rotation_x=90,
        double_sided=True,
        name=pid,
    )

# calibration markers (the camera sees these; the observer sees them too)
for name, (x, y, rgb) in {**CALIB_MARKERS, **CHECK_MARKERS}.items():
    Entity(
        model="quad",
        color=color.rgb32(*rgb),
        scale=MARKER_SIZE,
        position=(x, 0.006, y),
        rotation_x=90,
        double_sided=True,
        name=name,
    )

# no-fly zone: translucent fill + outline, demo only (the drone camera does not see it)
zone_verts = [Vec3(x, 0.008, y) for x, y in NO_FLY]
overlays.append(Entity(
    model=Mesh(vertices=zone_verts, triangles=[(0, 1, 2), (0, 2, 3)], mode="triangle"),
    color=color.rgba32(210, 50, 50, 70),
    double_sided=True,
))
overlays.append(Entity(
    model=Mesh(vertices=zone_verts + [zone_verts[0]], mode="line", thickness=3),
    color=color.rgb32(235, 60, 60),
))

# axis markers at the field origin: x along the 6 m side, y along the 4 m side
overlays.append(Entity(model="cube", color=color.rgb32(230, 70, 70),
                       scale=(1.4, 0.012, 0.05), position=(0.7, 0.01, 0.0)))
overlays.append(Entity(model="cube", color=color.rgb32(90, 220, 120),
                       scale=(0.05, 0.012, 1.4), position=(0.0, 0.01, 0.7)))

# --------------------------------------------------------------------------- #
# drone
# --------------------------------------------------------------------------- #
DRONE_COLOR = color.rgb32(232, 128, 32)
DRONE_DARK = color.rgb32(42, 44, 50)
ROTOR_COLOR = color.rgb32(225, 228, 235)
DRONE_SCALE = 1.6

drone = Entity(position=(3.0, ALT_NOMINAL, 2.0), scale=DRONE_SCALE, name="drone")
Entity(parent=drone, model="cube", scale=(0.15, 0.05, 0.15), color=DRONE_COLOR)
Entity(parent=drone, model="cube", scale=(0.03, 0.02, 0.30), color=DRONE_DARK)
Entity(parent=drone, model="cube", scale=(0.30, 0.02, 0.03), color=DRONE_DARK)
Entity(parent=drone, model="sphere", scale=0.045, y=0.04,
       color=color.rgb32(90, 220, 140))

rotors: list[Entity] = []
for sx in (-1, 1):
    for sz in (-1, 1):
        rotors.append(Entity(
            parent=drone, model="circle", rotation_x=90,
            scale=(0.17, 0.17), position=(0.11 * sx, 0.03, 0.11 * sz),
            color=ROTOR_COLOR, double_sided=True,
        ))

# payload indicator (the "operation" LED from the task)
led = Entity(parent=drone, model="sphere", scale=0.045, position=(0, -0.035, 0),
             color=color.rgb32(120, 40, 40))

# vertical projection line + ground spot: makes the position over the field obvious
plumb = Entity(model="cube", scale=(0.02, 1, 0.02),
               color=color.rgba32(255, 235, 120, 200))
spot = Entity(model="circle", rotation_x=90, scale=0.22,
              color=color.rgba32(255, 220, 90, 200))

# flight trail
trail_points: list[Vec3] = []
trail_mesh = Mesh(vertices=[Vec3(0, 0, 0), Vec3(0, 0, 0)], mode="line", thickness=2)
trail = Entity(model=trail_mesh, color=color.rgb32(255, 210, 70))


# --------------------------------------------------------------------------- #
# state
# --------------------------------------------------------------------------- #
class State:
    obs_yaw = 0.0
    obs_pitch = 0.0
    obs_pos = Vec3(8.6, 5.2, -3.2)
    looking = False
    cursor_saved: tuple[int, int] | None = None
    drone_view = False
    saved_view: tuple[Vec3, float, float, float] | None = None
    payload = False
    vel = Vec3(0, 0, 0)
    yaw = 0.0


CAMERA_OVERRIDE: tuple[Vec3, tuple[float, float, float], float] | None = None


def look_at_euler(origin: Vec3, target: Vec3) -> tuple[float, float]:
    d = target - origin
    yaw = math.degrees(math.atan2(d.x, d.z))
    pitch = -math.degrees(math.atan2(d.y, math.hypot(d.x, d.z)))
    return yaw, pitch


State.obs_yaw, State.obs_pitch = look_at_euler(State.obs_pos, Vec3(3.0, 0.6, 2.0))
camera.fov = OBSERVER_FOV
camera.position = State.obs_pos
camera.rotation = (State.obs_pitch, State.obs_yaw, 0)


# --------------------------------------------------------------------------- #
# HUD
# --------------------------------------------------------------------------- #
# Letterbox that shows the real 4:3 camera frame inside the 16:9 window.
# The visible part is 960 px wide, so the lens angle has to be widened by 16/9 / 4/3.
# Created before the HUD text on purpose: in camera.ui the draw order follows creation.
DRONE_VIEW_FOV = HFOV * 4 / 3
frame_bars = [
    Entity(parent=camera.ui, model="quad", color=color.black, visible=False,
           scale=(0.2222, 1.0), position=(sx * 0.7778, 0.0))
    for sx in (-1, 1)
]

help_line = (
    "<yellow>ПОЛЕ</yellow> 6×4 м, сетка 1 м, x — красная ось, y — зелёная\n"
    "\n"
    "<yellow>КАМЕРА (наблюдатель)</yellow>\n"
    "  WASD — полёт, Space/Ctrl — вверх/вниз\n"
    "  ПКМ (зажать) — вращать мышью, Shift — быстрее\n"
    "\n"
    "<yellow>ДРОН</yellow>\n"
    "  T — вперёд (+y)   G — назад (-y)\n"
    "  F — влево (-x)    H — вправо (+x)\n"
    "  R — выше          V — ниже\n"
    "\n"
    "<yellow>ПРОЧЕЕ</yellow>\n"
    "  C — вид с камеры дрона\n"
    "  P — подсветка (операция)\n"
    "  Backspace — сброс дрона\n"
    "\n"        # reserved rows: the live status is drawn on the panel below
    "\n"
    "\n"
    "\n"
    "\n"
)
hud_help = Text(text=help_line, position=(-0.86, 0.47), origin=(-0.5, 0.5),
                scale=0.72, background=True, color=color.rgb32(240, 240, 240))
hud_help.background.color = color.rgba32(10, 15, 25, 190)

hud_status = Text(text="", position=(-0.855, 0.175), origin=(-0.5, 0.5),
                  scale=0.75, color=color.rgb32(255, 245, 200))

hud_alarm = Text(text="", position=(0.0, -0.44), origin=(0, 0), scale=0.9,
                 color=color.rgb32(255, 120, 120))

# Shown only in the drone view: explains that the black bars are the real frame.
hud_cam_label = Text(
    text="камера дрона · кадр 640×480 (4:3) · HFOV 70°",
    position=(0.0, 0.455), origin=(0, 0.5), scale=0.8, background=True,
    color=color.rgb32(235, 240, 250), visible=False,
)
hud_cam_label.background.color = color.rgba32(10, 15, 25, 200)


# --------------------------------------------------------------------------- #
# controls
# --------------------------------------------------------------------------- #
def lock_mouse(on: bool) -> None:
    """Right mouse = look around.

    In locked mode Panda3D pins the cursor to the window centre itself and
    reports movement only as a delta, so base.win.getPointer() always returns
    the centre. The delta to use is ursina's mouse.velocity.

    Ursina only recentres the cursor on a frame where the mouse actually moved,
    and until then mouse.velocity is the *absolute* offset from the centre.
    Without the recentre below, the first pixel after a re-grab turns the
    camera by the whole distance between the cursor and the centre (~100°).
    """
    if on == State.looking:
        return
    if on:
        ptr = base.win.getPointer(0)
        if ptr.in_window:
            State.cursor_saved = (int(ptr.getX()), int(ptr.getY()))
    State.looking = on
    mouse.locked = on
    if on:
        mouse.position = Vec3(0, 0, 0)   # warp to the window centre
        mouse.prev_x = mouse.prev_y = 0.0
        mouse.velocity = Vec3(0, 0, 0)
    elif State.cursor_saved:
        base.win.movePointer(0, *State.cursor_saved)
        State.cursor_saved = None


def apply_look(vx: float, vy: float) -> None:
    """Turn the observer. vx/vy are ursina mouse deltas: +x right, +y down."""
    State.obs_yaw = (State.obs_yaw + vx * MOUSE_SENSITIVITY) % 360
    State.obs_pitch -= vy * MOUSE_SENSITIVITY
    State.obs_pitch = max(-PITCH_LIMIT, min(PITCH_LIMIT, State.obs_pitch))


def toggle_drone_view() -> None:
    if not State.drone_view:
        State.saved_view = (Vec3(camera.position), State.obs_yaw, State.obs_pitch,
                            camera.fov)
        State.drone_view = True
    else:
        State.drone_view = False
        if State.saved_view:
            pos, yaw, pitch, fov = State.saved_view
            camera.position = pos
            camera.rotation = (pitch, yaw, 0)
            camera.fov = fov
    show_overlays(not State.drone_view)


def show_overlays(on: bool) -> None:
    for ent in overlays + [plumb, spot, trail]:
        ent.visible = on
    for bar in frame_bars:
        bar.visible = not on
    hud_help.visible = on
    hud_status.visible = on
    hud_cam_label.visible = not on


def reset_drone() -> None:
    drone.position = Vec3(3.0, ALT_NOMINAL, 2.0)
    State.vel = Vec3(0, 0, 0)
    State.yaw = 0.0
    trail_points.clear()
    trail_mesh.vertices = [Vec3(0, 0, 0), Vec3(0, 0, 0)]
    trail_mesh.generate()


def set_payload(on: bool) -> None:
    State.payload = on
    led.color = color.rgb32(80, 235, 110) if on else color.rgb32(120, 40, 40)


def input(key: str) -> None:
    if key == "right mouse down":
        lock_mouse(True)
    elif key == "right mouse up":
        lock_mouse(False)
    elif key == "c":
        toggle_drone_view()
    elif key == "p":
        set_payload(not State.payload)
    elif key == "backspace":
        reset_drone()


def decay(current: Vec3, target: Vec3, rate: float, dt: float) -> Vec3:
    t = 1.0 - math.exp(-rate * dt)
    return current + (target - current) * t


# --------------------------------------------------------------------------- #
# per-frame
# --------------------------------------------------------------------------- #
def update() -> None:
    dt = min(time.dt, 1 / 20)  # never let a stalled frame teleport the drone
    boost = BOOST if held_keys["left shift"] else 1.0

    # ---- observer ---------------------------------------------------------- #
    if State.looking:
        vx, vy = mouse.velocity[0], mouse.velocity[1]
        if vx or vy:
            apply_look(vx, vy)
    elif mouse.right and not State.looking:
        lock_mouse(True)
    if not mouse.right and State.looking:
        lock_mouse(False)

    yaw_r = math.radians(State.obs_yaw)
    pitch_r = math.radians(State.obs_pitch)
    forward = Vec3(math.sin(yaw_r) * math.cos(pitch_r),
                   -math.sin(pitch_r),
                   math.cos(yaw_r) * math.cos(pitch_r))
    right = Vec3(math.cos(yaw_r), 0, -math.sin(yaw_r))

    if not State.drone_view:
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
        if move.length() > 0:
            State.obs_pos += move.normalized() * OBSERVER_SPEED * boost * dt
            State.obs_pos.x = max(-25.0, min(FIELD_W + 25.0, State.obs_pos.x))
            State.obs_pos.y = max(0.25, min(20.0, State.obs_pos.y))
            State.obs_pos.z = max(-25.0, min(FIELD_H + 25.0, State.obs_pos.z))
        camera.position = State.obs_pos
        camera.rotation = (State.obs_pitch, State.obs_yaw, 0)
        camera.fov = OBSERVER_FOV

    # ---- drone ------------------------------------------------------------- #
    cmd = Vec3(0, 0, 0)
    if held_keys["t"]:
        cmd.z += 1
    if held_keys["g"]:
        cmd.z -= 1
    if held_keys["h"]:
        cmd.x += 1
    if held_keys["f"]:
        cmd.x -= 1
    target_vel = cmd.normalized() * SPEED_LIMIT * boost if cmd.length() > 0 else Vec3(0, 0, 0)
    State.vel = decay(State.vel, target_vel, DRONE_ACCEL, dt)

    pos = drone.position
    pos += State.vel * dt
    if held_keys["r"]:
        pos.y += ALT_SPEED * dt
    if held_keys["v"]:
        pos.y -= ALT_SPEED * dt
    pos.x = max(0.15, min(FIELD_W - 0.15, pos.x))
    pos.z = max(0.15, min(FIELD_H - 0.15, pos.z))
    pos.y = max(ALT_CORRIDOR[0], min(ALT_CORRIDOR[1], pos.y))
    drone.position = pos

    if State.vel.length() > 0.02:
        target_yaw = math.degrees(math.atan2(State.vel.x, State.vel.z))
        delta = (target_yaw - State.yaw + 180) % 360 - 180
        State.yaw += delta * min(1.0, 8.0 * dt)
        drone.rotation_y = State.yaw

    for rotor in rotors:
        rotor.rotation_y += 2600 * dt

    # plumb line and ground spot
    plumb.scale = (0.02, pos.y, 0.02)
    plumb.position = (pos.x, pos.y / 2, pos.z)
    spot.position = (pos.x, 0.012, pos.z)
    spot.rotation_z = -State.yaw

    # trail
    flat = Vec3(pos.x, 0.02, pos.z)
    if not trail_points or (flat - trail_points[-1]).length() > 0.04:
        trail_points.append(flat)
        if len(trail_points) >= 2:  # a line primitive needs at least 2 vertices
            trail_mesh.vertices = trail_points
            trail_mesh.generate()

    # ---- drone camera view -------------------------------------------------- #
    if State.drone_view:
        camera.position = (pos.x, pos.y, pos.z)
        camera.rotation = (90, 0, State.yaw)
        camera.fov = DRONE_VIEW_FOV

    # ---- camera override (screenshots / documentation) ----------------------
    if CAMERA_OVERRIDE is not None:
        pos_o, rot_o, fov_o = CAMERA_OVERRIDE
        camera.position = pos_o
        camera.rotation = rot_o
        camera.fov = fov_o

    # ---- HUD ---------------------------------------------------------------- #
    hud_status.text = (
        f"дрон  x={pos.x:4.2f}  y={pos.z:4.2f}  h={pos.y:4.2f}\n"
        f"скорость {State.vel.length():4.2f} м/с   курс {State.yaw:6.1f}°\n"
        f"камера дрона: {'ВКЛ' if State.drone_view else 'выкл'}   "
        f"подсветка: {'ВКЛ' if State.payload else 'выкл'}"
    )
    hud_alarm.text = (
        "высота на границе коридора 1,0–1,4 м" if pos.y in (ALT_CORRIDOR[0], ALT_CORRIDOR[1]) else ""
    )


def selftest() -> None:
    """Prove the control mapping without a human at the keyboard."""
    failures: list[str] = []

    def run(keys: dict[str, int], frames: int = 60) -> None:
        for k in keys:
            held_keys[k] = 1
        for _ in range(frames):
            app.step()
        for k in keys:
            held_keys[k] = 0
        for _ in range(2):
            app.step()

    def check(name: str, value: float, sign: int, minimum: float) -> None:
        ok = value * sign > minimum
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {value:+.3f}")
        if not ok:
            failures.append(name)

    reset_drone()
    for _ in range(3):
        app.step()

    z0 = drone.z
    run({"t": 1})
    check("T moves +y", drone.z - z0, +1, 0.05)

    reset_drone()
    for _ in range(2):
        app.step()
    z0 = drone.z
    run({"g": 1})
    check("G moves -y", drone.z - z0, -1, 0.05)

    reset_drone()
    for _ in range(2):
        app.step()
    x0 = drone.x
    run({"h": 1})
    check("H moves +x", drone.x - x0, +1, 0.05)

    reset_drone()
    for _ in range(2):
        app.step()
    x0 = drone.x
    run({"f": 1})
    check("F moves -x", drone.x - x0, -1, 0.05)

    reset_drone()
    for _ in range(3):
        app.step()
    h0 = drone.y
    run({"r": 1}, 10)
    check("R climbs", drone.y - h0, +1, 0.01)

    run({"r": 1}, 120)
    ok = abs(drone.y - 1.4) < 1e-6
    print(f"  {'PASS' if ok else 'FAIL'}  altitude clamps at 1.4 m: {drone.y:.3f}")
    if not ok:
        failures.append("altitude clamp")

    reset_drone()
    run({"t": 1}, 400)
    ok = drone.z <= 3.85 + 1e-6
    print(f"  {'PASS' if ok else 'FAIL'}  drone stays inside the field: z={drone.z:.3f}")
    if not ok:
        failures.append("field clamp")

    # right mouse + drag must actually turn the camera. Regression guard: in
    # locked mode the pointer is pinned to the centre, so the delta has to come
    # from mouse.velocity, not from base.win.getPointer().
    State.obs_yaw, State.obs_pitch = 0.0, 0.0
    app.input("right mouse down")
    app.step()
    ok = State.looking and mouse.locked
    print(f"  {'PASS' if ok else 'FAIL'}  RMB grabs the mouse: looking={State.looking}")
    if not ok:
        failures.append("RMB grab")

    cx, cy = base.win.getXSize() // 2, base.win.getYSize() // 2
    base.win.movePointer(0, cx + 80, cy)          # drag right
    app.step()
    yaw_right = State.obs_yaw
    base.win.movePointer(0, cx, cy + 60)          # drag down
    app.step()
    pitch_down = State.obs_pitch
    ok = yaw_right > 5.0 and pitch_down > 3.0
    print(f"  {'PASS' if ok else 'FAIL'}  drag turns camera: yaw={yaw_right:+.1f}° "
          f"pitch={pitch_down:+.1f}° (down)")
    if not ok:
        failures.append("mouse look")

    app.input("right mouse up")
    app.step()
    ok = not State.looking and not mouse.locked
    print(f"  {'PASS' if ok else 'FAIL'}  RMB release frees the mouse")
    if not ok:
        failures.append("RMB release")

    # Re-grab must not teleport. The cursor is far from the centre when RMB is
    # pressed again; the first pixel of movement must stay one pixel worth.
    base.win.movePointer(0, 80, 80)
    app.step()
    app.input("right mouse down")
    app.step()
    app.step()
    yaw0 = State.obs_yaw
    base.win.movePointer(0, cx + 2, cy)
    app.step()
    dyaw = (State.obs_yaw - yaw0 + 180) % 360 - 180
    app.input("right mouse up")
    app.step()
    ok = abs(dyaw) < 2.0
    print(f"  {'PASS' if ok else 'FAIL'}  re-grab does not teleport: dyaw={dyaw:+.2f}° "
          f"(2 px)")
    if not ok:
        failures.append("re-grab jump")

    State.obs_pitch = 0.0
    apply_look(0, 10)     # huge upward drags
    apply_look(0, 10)
    ok = abs(abs(State.obs_pitch) - PITCH_LIMIT) < 1e-6
    print(f"  {'PASS' if ok else 'FAIL'}  pitch clamps: {State.obs_pitch:+.1f}°")
    if not ok:
        failures.append("pitch clamp")

    toggle_drone_view()
    app.step()
    ok = (abs(camera.fov - DRONE_VIEW_FOV) < 1e-6
          and all(bar.visible for bar in frame_bars))
    print(f"  {'PASS' if ok else 'FAIL'}  drone view: fov={camera.fov:.1f}°, 4:3 bars on")
    if not ok:
        failures.append("drone view")
    toggle_drone_view()
    app.step()
    ok = abs(camera.fov - OBSERVER_FOV) < 1e-6 and not hud_cam_label.visible
    print(f"  {'PASS' if ok else 'FAIL'}  back to observer: fov={camera.fov:.1f}°, bars off")
    if not ok:
        failures.append("observer view")

    print("\nselftest:", "OK" if not failures else f"FAILED {failures}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shot", type=str, default=None,
                        help="render a few frames to this PNG and exit")
    parser.add_argument("--overhead", action="store_true",
                        help="with --shot: top-down view of the whole field")
    parser.add_argument("--drone-view", action="store_true",
                        help="with --shot: render the drone camera instead")
    parser.add_argument("--selftest", action="store_true",
                        help="drive the drone with synthetic key presses and check the axes")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        base.userExit()
        return

    if args.shot:
        global CAMERA_OVERRIDE
        if args.drone_view:
            toggle_drone_view()
        elif args.overhead:
            CAMERA_OVERRIDE = (Vec3(3.0, 7.0, 2.0), (90, 0, 0), 55.0)
        for _ in range(5):
            app.step()
        out = Path(args.shot)
        out.parent.mkdir(parents=True, exist_ok=True)
        pnm = PNMImage()
        base.win.getScreenshot(pnm)
        pnm.write(Filename.fromOsSpecific(str(out.resolve())))
        print("saved:", out)
        base.userExit()
        return

    print(__doc__)
    app.run()


if __name__ == "__main__":
    main()
