"""Scene layout and a consistency check against the shared world model.

The plot coordinates are ground truth for rendering/evaluation.  They live in
``sim/field.py``; the CV pipeline must not import this module.
"""
from __future__ import annotations

from sim.field import (
    ALT_CORRIDOR, ALT_NOMINAL, FIELD_H, FIELD_W, HFOV, NO_FLY, PLOTS, PLOT_SIZE,
)
from shared import world


def validate_consistency() -> None:
    assert (FIELD_W, FIELD_H) == (world.FIELD_W, world.FIELD_H), "field size drift"
    assert (ALT_CORRIDOR[0], ALT_CORRIDOR[1]) == (world.ALT_MIN, world.ALT_MAX), "altitude drift"
    assert ALT_NOMINAL == world.ALT_NOMINAL, "nominal altitude drift"
    assert HFOV == world.HFOV, "camera HFOV drift"
    assert PLOT_SIZE == world.PLOT_SIZE, "plot size drift"
    assert [tuple(p) for p in NO_FLY] == [tuple(p) for p in world.NO_FLY], "no-fly zone drift"
    assert len(PLOTS) == 10, "plot count changed"


__all__ = ["ALT_CORRIDOR", "ALT_NOMINAL", "FIELD_H", "FIELD_W", "HFOV",
           "NO_FLY", "PLOTS", "PLOT_SIZE", "validate_consistency"]
