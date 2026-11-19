from types import SimpleNamespace

import pytest
from panda3d.core import WindowProperties

from sim import ui
from sim.modes import free


@pytest.mark.parametrize(("aspect", "scale"), [(4 / 3, 0.75), (16 / 9, 1.0), (21 / 9, 1.0)])
def test_interface_canvas_fits_window(monkeypatch, aspect, scale):
    canvas = SimpleNamespace(scale=None)
    monkeypatch.setattr(ui, "canvas", canvas)
    ui.fit_canvas(aspect)
    assert canvas.scale == pytest.approx(scale)


class PointerWindow:
    def __init__(self):
        self.size = (1280, 720)
        self.pointer = (370, 480)
        self.properties = WindowProperties()
        self.properties.setForeground(True)
        self.requests = []

    def getSize(self):
        return self.size

    def getXSize(self):
        return self.size[0]

    def getYSize(self):
        return self.size[1]

    def getPointer(self, _device):
        x, y = self.pointer
        return SimpleNamespace(in_window=True, getX=lambda: float(x), getY=lambda: float(y))

    def movePointer(self, _device, x, y):
        assert isinstance(x, int) and isinstance(y, int)
        self.pointer = (x, y)
        return True

    def getProperties(self):
        return self.properties

    def requestProperties(self, properties):
        self.requests.append(properties)


def test_camera_reacquires_without_turning_or_requesting_window_geometry(monkeypatch):
    win = PointerWindow()
    monkeypatch.setattr(free.FreeMode, "_build_ui", lambda self: None)
    monkeypatch.setattr(free, "mouse", SimpleNamespace(right=True))
    monkeypatch.setattr(ui, "pointer_over_ui", lambda: False)
    mode = free.FreeMode(SimpleNamespace(ursina=SimpleNamespace(win=win)))

    mode.input("right mouse down")
    mode.update_frame()
    assert mode.obs_yaw == mode.obs_pitch == 0
    win.pointer = (660, 368)
    mode.update_frame()
    mode.update_frame()
    look = (20 * free.MOUSE_SENSITIVITY / 720, 8 * free.MOUSE_SENSITIVITY / 720)
    assert (mode.obs_yaw, mode.obs_pitch) == pytest.approx(look)

    mode.input("right mouse up")
    assert win.pointer == (370, 480)
    win.pointer = (920, 560)
    mode.input("right mouse down")
    mode.update_frame()
    win.pointer = (658, 401)
    mode.update_frame()
    win.size = (1000, 800)
    mode.update_frame()
    assert win.pointer == (500, 400)
    assert (mode.obs_yaw, mode.obs_pitch) == pytest.approx(look)

    win.properties.setForeground(False)
    mode.update_frame()
    assert not mode.looking
    assert win.pointer == (500, 400)
    assert [props.getCursorHidden() for props in win.requests] == [True, False, True, False]
    assert all(not props.hasOrigin() and not props.hasSize() for props in win.requests)
