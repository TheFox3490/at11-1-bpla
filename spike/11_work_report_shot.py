"""Capture mode 3 in its real states: mission ready and execution finished."""
from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from panda3d.core import Filename, PNMImage

import sim.app as application

OUT = ROOT / "spike" / "out"


def update() -> None:
    application.update()


def input(key: str) -> None:  # noqa: A001 - Ursina hook
    application.input(key)


def shot(name: str) -> None:
    for _ in range(40):
        app.ursina.step()
    pnm = PNMImage()
    base.win.getScreenshot(pnm)
    path = OUT / name
    pnm.write(Filename.fromOsSpecific(str(path.resolve())))
    print("saved", path)


app = application.Application()
application.APP = app
app.set_mode("work")
work = app.modes["work"]
work.reset_work()
registry = ROOT / "state" / "probe_registry.json"
registry.unlink(missing_ok=True)
app.registry_path = registry
app.set_speed(5.0)
work.start_recon()

deadline = time.time() + 300
while time.time() < deadline:
    app.ursina.step()
    if work.phase in {"ready", "mission_invalid", "cv_failed", "recon_failed"}:
        break

print("phase", work.phase, "frames", work.recon.capture_count if work.recon else None,
      "error", (work.recon.error if work.recon else None) or work.cv_error)
if work.result is not None:
    print("counts", work.result.report.get("counts"))
    print("errors", work.result.errors)
shot("app_work_report.png")

if work.mission_valid:
    work.start_executor()
    deadline = time.time() + 300
    while time.time() < deadline:
        app.ursina.step()
        if app.execution is None and work.phase in {"exec_done", "exec_failed"}:
            break
    run = app.last_execution
    print("exec", work.phase, "actions", [(a.get("target_id"), a.get("status"))
                                          for a in (run.actions if run else [])])
    shot("app_work_done.png")

base.userExit()
