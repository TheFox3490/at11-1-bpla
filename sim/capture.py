"""Capture a real 640x480 nadir frame from the Ursina window.

We temporarily point the camera down, hide the UI and overlays, and use a
central 4:3 crop of the current framebuffer. The projection keeps the drone's
horizontal FOV fixed even when the observer resizes the window.
"""
from __future__ import annotations

import math
from pathlib import Path

from PIL import Image
from panda3d.core import Filename, PNMImage
from ursina import Vec3, camera

from shared.world import HFOV, IMG_H, IMG_W


class FrameGrabber:
    def __init__(self, hide_entities):
        # Either a list or a callable returning the current overlays; the route
        # preview and other entities are created after startup, so resolving
        # the list at capture time keeps them out of the frames.
        self._hide_source = hide_entities
        self._hidden: list = []
        self.busy = False
        self.armed = False
        self.last_path: Path | None = None
        self.error: str | None = None
        self._saved = None
        self._crop = None
        self._frame_size: tuple[int, int] | None = None
        self._path: Path | None = None

    def _collect_hidden(self) -> list:
        source = self._hide_source() if callable(self._hide_source) else self._hide_source
        return [entity for entity in source if entity is not None]

    def arm(self, pose: dict, path: str | Path) -> bool:
        """Point the camera and request a capture on the next rendered frame."""
        if self.busy:
            return False
        self.busy = True
        self.armed = True
        self.error = None
        self.last_path = None
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        ui = camera.ui
        self._hidden = self._collect_hidden()
        self._saved = (
            Vec3(camera.position), tuple(camera.rotation), camera.fov,
            ui.visible, [e.visible for e in self._hidden],
        )
        ui.visible = False
        for e in self._hidden:
            e.visible = False

        camera.position = (pose["x"], pose["alt"], pose["y"])
        camera.rotation = (90, 0, pose["yaw"])
        self._frame_size = None
        self._fit_capture(*base.win.getSize())
        return True

    def _fit_capture(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        self._frame_size = (width, height)
        target_aspect = IMG_W / IMG_H
        if width / height >= target_aspect:
            crop_w, crop_h = round(height * target_aspect), height
            x0, y0 = (width - crop_w) // 2, 0
        else:
            crop_w, crop_h = width, round(width / target_aspect)
            x0, y0 = 0, (height - crop_h) // 2
        self._crop = (x0, y0, x0 + crop_w, y0 + crop_h)
        camera.fov = math.degrees(2 * math.atan(math.tan(math.radians(HFOV / 2)) * width / crop_w))

    def update(self) -> None:
        """Called once per rendered frame, before mode updates."""
        if not self.armed:
            return
        if base.win.getProperties().getMinimized():
            self._frame_size = None
            return
        size = tuple(base.win.getSize())
        if self._frame_size != size:
            self._fit_capture(*size)
            return
        self.armed = False
        try:
            pnm = PNMImage()
            if not base.win.getScreenshot(pnm):
                raise RuntimeError("window screenshot failed")
            if (pnm.getXSize(), pnm.getYSize()) != size:
                self.armed = True
                return
            raw = self._path.with_suffix(".raw.png")
            if not pnm.write(Filename.fromOsSpecific(str(raw.resolve()))):
                raise RuntimeError("PNMImage.write failed")
            img = Image.open(raw).convert("RGB")
            img = img.crop(self._crop).resize((IMG_W, IMG_H), Image.LANCZOS)
            img.save(self._path)
            raw.unlink(missing_ok=True)
            self.last_path = self._path
        except Exception as exc:  # keep the app alive, report the reason
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            if not self.armed:
                self._restore()
                self.busy = False

    def _restore(self) -> None:
        if self._saved is None:
            return
        pos, rot, fov, ui_visible, vis = self._saved
        camera.position = pos
        camera.rotation = rot
        camera.fov = fov
        camera.ui.visible = ui_visible
        for e, v in zip(self._hidden, vis):
            e.visible = v
        self._hidden = []
        self._saved = None

    def cancel(self) -> None:
        if self._saved is not None:
            self._restore()
        self.armed = False
        self.busy = False
