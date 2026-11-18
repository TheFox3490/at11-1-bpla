"""Static simulator scene shared by all three modes."""
from __future__ import annotations

from ursina import Entity, Mesh, Sky, Texture, Vec3, color, destroy, scene

from shared.world import (
    ALT_NOMINAL, CALIB_MARKERS, CHECK_MARKERS, FIELD_H, FIELD_W, MARKER_SIZE,
    NO_FLY, PLOT_SIZE,
)
from sim.drone import DroneRig
from sim.field import class_texture_paths, route_number_texture
from sim.layout import validate_consistency
from sim.scenario import default_objects


def _xy(point) -> tuple[float, float]:
    """Point as ``(x, y)``; the planner stores route data as dicts."""
    if isinstance(point, dict):
        return float(point["x"]), float(point["y"])
    return float(point[0]), float(point[1])


class WorldScene:
    def __init__(self, layout: list[dict] | None = None):
        validate_consistency()
        scene.fog_color = color.rgb32(150, 190, 225)
        scene.fog_density = 0.012
        Sky()

        self.grid: list[Entity] = []
        self.axes: list[Entity] = []
        self.nfz: list[Entity] = []
        self.border: list[Entity] = []
        self.calib_entities: list[Entity] = []
        self.route_entities: list[Entity] = []
        self.route_numbers: list[Entity] = []
        self._badge_textures: dict[int, Texture] = {}

        # ground and field
        Entity(model="plane", scale=(60, 1, 60), color=color.rgb32(104, 132, 74),
               position=(FIELD_W / 2, -0.02, FIELD_H / 2))
        self.field_entity = Entity(
            model="quad", color=color.rgb32(112, 86, 58),
            scale=(FIELD_W, FIELD_H), position=(FIELD_W / 2, 0, FIELD_H / 2),
            rotation_x=90, double_sided=True, collider="box", name="field",
        )

        # 1 m grid
        for gx in range(1, int(FIELD_W)):
            self.grid.append(Entity(model="cube", color=color.rgba32(255, 255, 255, 42),
                                    scale=(0.012, 0.002, FIELD_H),
                                    position=(gx, 0.003, FIELD_H / 2)))
        for gy in range(1, int(FIELD_H)):
            self.grid.append(Entity(model="cube", color=color.rgba32(255, 255, 255, 42),
                                    scale=(FIELD_W, 0.002, 0.012),
                                    position=(FIELD_W / 2, 0.003, gy)))

        # field border
        for x, z, sx, sz in [
            (FIELD_W / 2, -0.02, FIELD_W, 0.04),
            (FIELD_W / 2, FIELD_H + 0.02, FIELD_W, 0.04),
            (-0.02, FIELD_H / 2, 0.04, FIELD_H),
            (FIELD_W + 0.02, FIELD_H / 2, 0.04, FIELD_H),
        ]:
            self.border.append(Entity(model="cube", color=color.rgb32(235, 235, 235),
                                      scale=(sx, 0.01, sz), position=(x, 0.005, z)))

        # plots come from the editable layout (the ground truth the drone sees)
        self._plot_textures: dict[str, Texture] = {}
        for klass, path in class_texture_paths().items():
            self._plot_textures[klass] = Texture(path)
        self.plot_entities: dict[str, Entity] = {}
        self.set_layout(layout or default_objects())

        # calibration markers: hidden by default, shown only for the calib frame
        for name, (x, y, rgb) in {**CALIB_MARKERS, **CHECK_MARKERS}.items():
            marker = Entity(model="quad", color=color.rgb32(*rgb), scale=MARKER_SIZE,
                            position=(x, 0.006, y), rotation_x=90, double_sided=True,
                            name=name, visible=False)
            self.calib_entities.append(marker)

        # Editor-only hint: the ground area reserved for the calibration pattern.
        # The individual markers stay hidden outside frames/calib.png.
        zone = [Vec3(0.34, 0.007, 0.34), Vec3(1.26, 0.007, 0.34),
                Vec3(1.26, 0.007, 1.26), Vec3(0.34, 0.007, 1.26),
                Vec3(0.34, 0.007, 0.34)]
        self.calib_zone = Entity(
            model=Mesh(vertices=zone, mode="line", thickness=2),
            color=color.rgba32(90, 170, 255, 170), name="calib_zone", visible=False)

        # no-fly zone
        zone_verts = [Vec3(x, 0.008, y) for x, y in NO_FLY]
        self.nfz.append(Entity(
            model=Mesh(vertices=zone_verts, triangles=[(0, 1, 2), (0, 2, 3)],
                       mode="triangle"),
            color=color.rgba32(210, 50, 50, 70), double_sided=True,
        ))
        self.nfz.append(Entity(
            model=Mesh(vertices=zone_verts + [zone_verts[0]], mode="line", thickness=3),
            color=color.rgb32(235, 60, 60),
        ))

        # axes at the origin
        self.axes.append(Entity(model="cube", color=color.rgb32(230, 70, 70),
                                scale=(1.4, 0.012, 0.05), position=(0.7, 0.01, 0.0)))
        self.axes.append(Entity(model="cube", color=color.rgb32(90, 220, 120),
                                scale=(0.05, 0.012, 1.4), position=(0.0, 0.01, 0.7)))

        # drone and flight trail
        self.rig = DroneRig()
        self.trail_points: list[Vec3] = []
        self.trail_mesh = Mesh(vertices=[Vec3(0, 0, 0), Vec3(0, 0, 0)],
                               mode="line", thickness=3)
        self.trail = Entity(model=self.trail_mesh, color=color.rgb32(255, 210, 70))
        self.plumb = Entity(model="cube", scale=(0.02, 1, 0.02),
                            color=color.rgba32(255, 235, 120, 200))
        self.spot = Entity(model="circle", rotation_x=90, scale=0.22,
                           color=color.rgba32(255, 220, 90, 200))
        self.sync_visuals(ALT_NOMINAL)

    # ------------------------------------------------------------------ #
    def set_layout(self, objects: list[dict]) -> None:
        """Rebuild the plot quads from a scenario layout."""
        for entity in self.plot_entities.values():
            destroy(entity)
        self.plot_entities.clear()
        for obj in objects:
            klass = obj.get("klass", "U")
            texture = self._plot_textures.get(klass, self._plot_textures["U"])
            self.plot_entities[str(obj["id"])] = Entity(
                model="quad", texture=texture, scale=PLOT_SIZE,
                position=(float(obj["x"]), 0.004, float(obj["y"])),
                rotation_x=90, double_sided=True, name=str(obj["id"]),
            )

    def set_calibration_visible(self, on: bool) -> None:
        for entity in self.calib_entities:
            entity.visible = on

    def set_calibration_zone_visible(self, on: bool) -> None:
        self.calib_zone.visible = on

    # ------------------------------------------------------------------ #
    def show_route(self, route: list, targets: list[dict],
                   expanded: list | None = None) -> None:
        """Preview the executor route: polyline, safety contour and order badges."""
        self.clear_route()
        points = [(float(p["x"]), float(p["y"])) for p in route]
        if len(points) >= 2:
            verts = [Vec3(x, 0.028, y) for x, y in points]
            self.route_entities.append(Entity(
                model=Mesh(vertices=verts, mode="line", thickness=5),
                color=color.rgb32(80, 230, 255), name="route_line"))
        if expanded:
            ring = [Vec3(float(_xy(p)[0]), 0.018, float(_xy(p)[1])) for p in expanded]
            if ring:
                ring.append(ring[0])
                self.route_entities.append(Entity(
                    model=Mesh(vertices=ring, mode="line", thickness=2),
                    color=color.rgba32(250, 170, 60, 220), name="route_margin"))
        for index, target in enumerate(targets, 1):
            point = target["point"]
            texture = self._badge_textures.get(index)
            if texture is None:
                texture = Texture(route_number_texture(index))
                self._badge_textures[index] = texture
            x, y = float(point["x"]), float(point["y"])
            self.route_entities.append(Entity(
                model=Mesh(vertices=[Vec3(x, 0.02, y), Vec3(x, 0.30, y)],
                           mode="line", thickness=2),
                color=color.rgba32(80, 230, 255, 150), name=f"route_stem_{index}"))
            badge = Entity(
                model="quad", texture=texture, scale=0.30,
                position=(x, 0.42, y), billboard=True,
                name=f"route_badge_{index}")
            self.route_entities.append(badge)
            self.route_numbers.append(badge)

    def clear_route(self) -> None:
        for entity in self.route_entities:
            destroy(entity)
        self.route_entities.clear()
        self.route_numbers.clear()

    # ------------------------------------------------------------------ #
    def sync_visuals(self, alt: float) -> None:
        pos = self.rig.root.position
        self.plumb.scale = (0.02, max(alt, 0.02), 0.02)
        self.plumb.position = (pos.x, alt / 2, pos.z)
        self.spot.position = (pos.x, 0.012, pos.z)
        self.spot.rotation_z = -self.rig.root.rotation_y
        flat = Vec3(pos.x, 0.035, pos.z)
        if not self.trail_points or (flat - self.trail_points[-1]).length() > 0.04:
            self.trail_points.append(flat)
            if len(self.trail_points) >= 2:
                self.trail_mesh.vertices = list(self.trail_points)
                self.trail_mesh.generate()

    def clear_trail(self) -> None:
        self.trail_points.clear()
        self.trail_mesh.vertices = [Vec3(0, 0, 0), Vec3(0, 0, 0)]
        self.trail_mesh.generate()

    def set_overlays(self, on: bool) -> None:
        for group in (self.grid, self.axes, self.nfz, self.border):
            for e in group:
                e.visible = on
        self.plumb.visible = on
        self.spot.visible = on
        self.trail.visible = on

    def capture_hidden_entities(self) -> list[Entity]:
        out = [self.plumb, self.spot, self.trail, self.calib_zone, *self.route_entities]
        for group in (self.grid, self.axes, self.nfz):
            out.extend(group)
        return out

    def set_border_visible(self, on: bool) -> None:
        for e in self.border:
            e.visible = on
