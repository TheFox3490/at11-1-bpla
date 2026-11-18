"""Spike 3: textured training field + recorded downward camera frames.

Design decision (learned the hard way): the field is NOT one giant textured
quad - it is built from separate plot entities at exact world coordinates.
That removes all UV/scale ambiguity and matches the physical setup: plots are
distinct objects the camera actually sees.

Output:
  spike/out/run_spike/textures/*.png
  spike/out/run_spike/frames/<id>.png + <id>.json
  spike/out/run_spike/run.json
  spike/out/run_spike/ground_truth.json   (eval only, never read by the pipeline)

Run:  .venv\\Scripts\\python.exe spike\\02_field_capture.py
"""
from __future__ import annotations

import hashlib
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
from panda3d.core import Filename, PNMImage
from ursina import Entity, Texture, Ursina, camera, color, scene, window

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.field import (  # noqa: E402
    ALT_CORRIDOR, ALT_NOMINAL, CALIB_MARKERS, CHECK_MARKERS, FIELD_H, FIELD_W,
    HFOV, MARKER_SIZE, NO_FLY, PLOTS, PLOT_SIZE, make_plot_texture,
    validate_layout,
)

# --------------------------------------------------------------------------- #
# Field in frame F: x along the 6 m side, y along the 4 m side, metres.
# Geometry, classes, markers and textures all live in sim/field.py so the
# recorder and the demo can never drift apart.
# --------------------------------------------------------------------------- #
IMG_W, IMG_H = 640, 480
SPEED = 0.4
SEED = 2026


def scan_path() -> list[tuple[float, float]]:
    xs = [0.9 + 0.6 * i for i in range(8)]
    lanes = [0.7, 2.0, 3.3]
    path: list[tuple[float, float]] = []
    for i, y in enumerate(lanes):
        lane = xs if i % 2 == 0 else list(reversed(xs))
        path.extend((x, y) for x in lane)
    return path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_scene(tex_dir: Path) -> list[Entity]:
    tex_dir.mkdir(parents=True, exist_ok=True)
    entities: list[Entity] = []

    # ground (plain colour on purpose: no mega-texture, no UV traps)
    entities.append(Entity(
        model="quad", color=color.rgb32(112, 86, 58),
        scale=(FIELD_W, FIELD_H), position=(FIELD_W / 2, 0, FIELD_H / 2),
        rotation_x=90, double_sided=True,
    ))

    for pid, cls, (x, y) in PLOTS:
        tex = make_plot_texture(cls, tex_dir / f"{pid}_{cls}.png")
        entities.append(Entity(
            model="quad", texture=Texture(tex),
            scale=PLOT_SIZE, position=(x, 0.002, y),
            rotation_x=90, double_sided=True, name=pid,
        ))

    for name, (x, y, rgb) in {**CALIB_MARKERS, **CHECK_MARKERS}.items():
        entities.append(Entity(
            model="quad",
            color=color.rgb32(*rgb), scale=MARKER_SIZE, position=(x, 0.004, y),
            rotation_x=90, double_sided=True, name=name,
        ))
    return entities


def main() -> None:
    layout = validate_layout()
    print(f"layout ok: {layout['plots']} plots, {layout['a_total']} A, "
          f"{layout['a_free']} A allowed, {layout['a_zone']} A in no-fly")

    out = Path(__file__).with_name("out") / "run_spike"
    frames_dir = out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    app = Ursina(
        title="BPLA field capture",
        size=(IMG_W, IMG_H),
        borderless=True,
        development_mode=False,
        editor_ui_enabled=False,
        show_ursina_splash=False,
    )
    window.color = color.black

    build_scene(out / "textures")
    camera.fov = HFOV
    camera.clip_plane_far = 100
    for _ in range(3):
        app.step()

    rng = random.Random(SEED)
    frames_meta: list[dict] = []

    def capture(name: str, pose: tuple[float, float, float, float], t: float) -> None:
        x, y, alt, yaw = pose
        camera.position = (x, alt, y)
        camera.rotation = (90, 0, yaw)
        for _ in range(2):
            app.step()

        img_path = frames_dir / f"{name}.png"
        pnm = PNMImage()
        base.win.getScreenshot(pnm)
        pnm.write(Filename.fromOsSpecific(str(img_path)))

        meta = {
            "frame_id": name,
            "run_id": "run_spike",
            "t_s": round(t, 3),
            "image": f"frames/{name}.png",
            "pose": {"x": round(x, 4), "y": round(y, 4),
                     "alt": round(alt, 4), "yaw": round(yaw, 6)},
            "camera": {"hfov_deg": HFOV, "width": IMG_W, "height": IMG_H,
                       "mount": "nadir, fixed, no gimbal"},
            "calib_id": "calib_spike_v1",
            "sha256": sha256(img_path),
        }
        (frames_dir / f"{name}.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        frames_meta.append(meta)

    capture("calib", (0.8, 0.8, ALT_NOMINAL, 0.0), 0.0)
    print("calibration frame done")

    t = 0.0
    prev = None
    for i, (x, y) in enumerate(scan_path(), start=1):
        if prev is not None:
            t += math.dist(prev, (x, y)) / SPEED
        prev = (x, y)
        alt = min(max(ALT_NOMINAL + rng.uniform(-0.04, 0.04), ALT_CORRIDOR[0]), ALT_CORRIDOR[1])
        yaw = math.radians(rng.uniform(-1.5, 1.5))
        capture(f"f{i:03d}", (x, y, alt, yaw), t)
    print(f"scan frames done: {len(frames_meta) - 1}")

    # wide shot of the whole field (demo / documentation only, not recon data)
    capture("overview", (3.0, 2.0, 6.5, 0.0), t)

    run = {
        "run_id": "run_spike",
        "generator": "spike/02_field_capture.py",
        "field": {"width_m": FIELD_W, "height_m": FIELD_H, "frame": "F",
                  "origin": "corner (0,0), x along 6 m, y along 4 m"},
        "camera": {"hfov_deg": HFOV, "resolution": [IMG_W, IMG_H],
                   "mount": "nadir fixed", "alt_nominal_m": ALT_NOMINAL,
                   "alt_corridor_m": list(ALT_CORRIDOR)},
        "motion": {"speed_m_s": SPEED, "pattern": "3 lanes, boustrophedon"},
        "calibration": {
            "calib_id": "calib_spike_v1",
            "markers": {k: {"x": v[0], "y": v[1]} for k, v in CALIB_MARKERS.items()},
            "check_points": {k: {"x": v[0], "y": v[1]} for k, v in CHECK_MARKERS.items()},
            "note": "visible in frames/calib.png only",
        },
        "frames": [m["frame_id"] for m in frames_meta],
        "seed": SEED,
    }
    (out / "run.json").write_text(json.dumps(run, indent=2, ensure_ascii=False), encoding="utf-8")

    gt = {
        "run_id": "run_spike",
        "warning": "evaluation only - the pipeline must never read this file",
        "plots": [{"id": pid, "class": cls, "x": x, "y": y} for pid, cls, (x, y) in PLOTS],
        "no_fly": [{"x": x, "y": y} for x, y in NO_FLY],
    }
    (out / "ground_truth.json").write_text(
        json.dumps(gt, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    print("frames:", len(frames_meta), "->", frames_dir)
    base.userExit()


if __name__ == "__main__":
    main()
