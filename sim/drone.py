"""One drone model that changes role/colour, plus its kinematic state."""
from __future__ import annotations

import math
import time

from ursina import Entity, color

from shared.world import ALT_MAX, ALT_MIN, ALT_NOMINAL, FIELD_H, FIELD_W, HOME, SPEED_LIMIT

DRONE_SCALE = 1.6
ROLE_COLORS = {
    "free": (232, 128, 32),
    "recon": (245, 196, 48),
    "executor": (62, 132, 238),
    "neutral": (200, 200, 200),
}


class DroneRig:
    def __init__(self):
        self.root = Entity(position=(HOME[0], ALT_NOMINAL, HOME[1]),
                           scale=DRONE_SCALE, name="drone")
        self.body = Entity(parent=self.root, model="cube", scale=(0.15, 0.05, 0.15),
                           color=color.rgb32(*ROLE_COLORS["free"]))
        Entity(parent=self.root, model="cube", scale=(0.03, 0.02, 0.30),
               color=color.rgb32(42, 44, 50))
        Entity(parent=self.root, model="cube", scale=(0.30, 0.02, 0.03),
               color=color.rgb32(42, 44, 50))
        Entity(parent=self.root, model="sphere", scale=0.045, y=0.04,
               color=color.rgb32(90, 220, 140))
        self.rotors = []
        for sx in (-1, 1):
            for sz in (-1, 1):
                self.rotors.append(Entity(
                    parent=self.root, model="circle", rotation_x=90,
                    scale=(0.17, 0.17), position=(0.11 * sx, 0.03, 0.11 * sz),
                    color=color.rgb32(225, 228, 235), double_sided=True,
                ))
        self.led = Entity(parent=self.root, model="sphere", scale=0.045,
                          position=(0, -0.035, 0), color=color.rgb32(120, 40, 40))
        self.role = "free"
        self.set_led(False)

    def set_role(self, role: str) -> None:
        self.role = role
        self.body.color = color.rgb32(*ROLE_COLORS.get(role, ROLE_COLORS["neutral"]))
        self.root.name = f"drone_{role}"

    def set_pose(self, x: float, y: float, alt: float, yaw: float) -> None:
        self.root.position = (x, alt, y)
        self.root.rotation_y = yaw

    def set_led(self, on: bool) -> None:
        self.led.color = color.rgb32(80, 235, 110) if on else color.rgb32(120, 40, 40)

    def spin(self, dt: float) -> None:
        for r in self.rotors:
            r.rotation_y += 2600.0 * dt


class Drone:
    """Field-frame kinematics; the visual rig only mirrors the state."""

    def __init__(self, rig: DroneRig):
        self.rig = rig
        self.speed_limit = SPEED_LIMIT
        self.reset()

    def reset(self) -> None:
        self.x, self.y = HOME
        self.alt = ALT_NOMINAL
        self.yaw = 0.0
        self.vx = self.vy = 0.0
        self.valt = 0.0
        self.target: tuple[float, float] | None = None
        self.target_alt = ALT_NOMINAL
        self.landing = False
        self.arrived = True
        self.alt_reached = True
        self.last_pose_wall = time.time()
        self.rig.set_pose(self.x, self.y, self.alt, self.yaw)
        self.rig.set_led(False)

    # ------------------------------------------------------------------ #
    def set_manual(self, vx: float, vy: float, valt: float) -> None:
        self.target = None
        self.vx, self.vy, self.valt = vx, vy, valt

    def stop(self) -> None:
        self.set_manual(0.0, 0.0, 0.0)

    def goto(self, x: float, y: float, alt: float = ALT_NOMINAL,
             landing: bool = False) -> None:
        self.target = (x, y)
        self.target_alt = alt
        self.landing = landing
        self.arrived = False

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)

    # ------------------------------------------------------------------ #
    def update(self, dt: float) -> None:
        if self.target is not None:
            tx, ty = self.target
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            if dist <= 0.045:
                self.x, self.y = tx, ty
                self.vx = self.vy = 0.0
                self.arrived = True
            else:
                step = min(self.speed_limit * dt, dist)
                ux, uy = dx / dist, dy / dist
                self.x += ux * step
                self.y += uy * step
                self.vx, self.vy = ux * self.speed_limit, uy * self.speed_limit
                self.arrived = False
            # Altitude is independent so landing can descend after arriving.
            da = self.target_alt - self.alt
            if abs(da) <= 0.01:
                self.alt = self.target_alt
                self.alt_reached = True
            else:
                step_a = math.copysign(min(0.45 * dt, abs(da)), da)
                self.alt += step_a
                self.alt_reached = False
        else:
            self.x += self.vx * dt
            self.y += self.vy * dt
            self.alt += self.valt * dt
            self.arrived = True
            self.alt_reached = True

        self.x = max(0.15, min(FIELD_W - 0.15, self.x))
        self.y = max(0.15, min(FIELD_H - 0.15, self.y))
        if self.landing:
            self.alt = max(0.04, min(ALT_MAX, self.alt))
        else:
            self.alt = max(ALT_MIN, min(ALT_MAX, self.alt))

        speed = math.hypot(self.vx, self.vy)
        if speed > 0.02:
            target_yaw = math.degrees(math.atan2(self.vx, self.vy))
            delta = (target_yaw - self.yaw + 180.0) % 360.0 - 180.0
            self.yaw += delta * min(1.0, 8.0 * dt)
            self.yaw = (self.yaw + 180.0) % 360.0 - 180.0

        self.rig.set_pose(self.x, self.y, self.alt, self.yaw)
        self.rig.spin(dt)
        self.last_pose_wall = time.time()
