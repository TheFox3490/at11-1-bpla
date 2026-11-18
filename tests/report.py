"""Artifacts for the acceptance bench: field map, timeline, protocol, metrics.

Everything here is evaluation-only code.  It may read the simulator layout and
``runs/*`` because it judges the pipeline from the outside; the pipeline itself
still never sees ground truth.
"""
from __future__ import annotations

import json
import math
import platform
import sys
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from shared.contracts import iso, read_json, write_json
from shared.world import (
    FIELD_H, FIELD_W, NO_FLY, PLOT_SIZE, expand_polygon,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent

INK = (232, 238, 246)
MUTED = (150, 164, 182)
BG = (16, 22, 32)
GRID = (58, 70, 88)
PLOT = {"A": (206, 62, 62), "B": (56, 104, 206), "U": (176, 176, 176)}
ROUTE = (80, 230, 255)
NFZ_FILL = (170, 50, 50)
NFZ_EDGE = (238, 80, 80)
EXP_EDGE = (250, 170, 60)
GOOD = (96, 208, 132)
BAD = (238, 100, 100)
WARN = (240, 188, 78)


def _font(size: int, bold: bool = False):
    name = "arialbd.ttf" if bold else "arial.ttf"
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default()


class MapCanvas:
    """Top-down 6x4 m field with plots, NFZ, route and evaluation markers."""

    SCALE = 92
    PAD = 42

    def __init__(self, layout: list[dict], title: str = ""):
        self.layout = list(layout)
        self.title = title
        self.w = int(FIELD_W * self.SCALE + self.PAD * 2)
        self.h = int(FIELD_H * self.SCALE + self.PAD * 2 + 34)
        self.img = Image.new("RGB", (self.w, self.h), BG)
        self.d = ImageDraw.Draw(self.img)
        self._base()

    def _px(self, x: float, y: float) -> tuple[float, float]:
        # Field y grows "up" on the plan, image y grows down.
        return (self.PAD + x * self.SCALE,
                self.PAD + (FIELD_H - y) * self.SCALE)

    def _base(self) -> None:
        d = self.d
        d.text((self.PAD, 10), self.title, font=_font(15, True), fill=INK)
        x0, y0 = self._px(0, FIELD_H)
        x1, y1 = self._px(FIELD_W, 0)
        d.rectangle([x0, y0, x1, y1], fill=(58, 44, 32), outline=(210, 210, 210), width=2)
        for gx in range(1, int(FIELD_W)):
            gx0 = self._px(gx, 0)[0]
            d.line([(gx0, y0), (gx0, y1)], fill=GRID, width=1)
        for gy in range(1, int(FIELD_H)):
            gy0 = self._px(0, gy)[1]
            d.line([(x0, gy0), (x1, gy0)], fill=GRID, width=1)
        # coordinates hint
        d.text((x1 + 4, y1 - 14), "x", font=_font(12), fill=(230, 120, 120))
        d.text((x0 - 16, y0 - 16), "y", font=_font(12), fill=(120, 220, 150))

    def nfz(self, expanded: list | None = None) -> "MapCanvas":
        d = self.d
        pts = [self._px(x, y) for x, y in NO_FLY]
        d.polygon(pts, fill=(120, 36, 36), outline=NFZ_EDGE)
        if expanded:
            d.line([self._px(float(p["x"]), float(p["y"])) for p in expanded] + [self._px(float(expanded[0]["x"]), float(expanded[0]["y"]))],
                   fill=EXP_EDGE, width=2)
        return self

    def plots(self) -> "MapCanvas":
        d = self.d
        half = PLOT_SIZE / 2
        for obj in self.layout:
            x, y = float(obj["x"]), float(obj["y"])
            p0 = self._px(x - half, y + half)
            p1 = self._px(x + half, y - half)
            d.rectangle([p0[0], p0[1], p1[0], p1[1]],
                        fill=PLOT.get(str(obj.get("klass")), PLOT["U"]),
                        outline=(245, 245, 245), width=2)
            d.text(((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2),
                   str(obj.get("klass", "?")), font=_font(13, True),
                   fill=(20, 20, 20), anchor="mm")
        return self

    def route(self, points: list[dict], color=ROUTE, width: int = 3,
              dashed: bool = False) -> "MapCanvas":
        if len(points) < 2:
            return self
        px = [self._px(float(p["x"]), float(p["y"])) for p in points]
        if dashed:
            for a, b in zip(px, px[1:]):
                self._dashed(a, b, color, width)
        else:
            self.d.line(px, fill=color, width=width, joint="curve")
        for p in px:
            self.d.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], fill=color)
        return self

    def _dashed(self, a, b, color, width=2, dash=9, gap=6) -> None:
        length = math.dist(a, b)
        if length == 0:
            return
        ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
        pos = 0.0
        while pos < length:
            end = min(pos + dash, length)
            self.d.line([(a[0] + ux * pos, a[1] + uy * pos),
                         (a[0] + ux * end, a[1] + uy * end)], fill=color, width=width)
            pos = end + gap

    def markers(self, points: list[tuple[float, float, str, str]],
                color=GOOD) -> "MapCanvas":
        for x, y, label, _kind in points:
            px, py = self._px(float(x), float(y))
            self.d.ellipse([px - 9, py - 9, px + 9, py + 9], outline=color, width=2)
            self.d.text((px, py), label, font=_font(11, True), fill=color, anchor="mm")
        return self

    def cross(self, x: float, y: float, label: str, color=BAD) -> "MapCanvas":
        px, py = self._px(float(x), float(y))
        self.d.line([px - 12, py - 12, px + 12, py + 12], fill=color, width=3)
        self.d.line([px - 12, py + 12, px + 12, py - 12], fill=color, width=3)
        if label:
            self.d.text((px + 15, py - 8), label, font=_font(12, True), fill=color)
        return self

    def skipped(self, x: float, y: float, label: str) -> "MapCanvas":
        px, py = self._px(float(x), float(y))
        self.d.rectangle([px - 9, py - 9, px + 9, py + 9], outline=WARN, width=2)
        self.d.text((px + 12, py - 8), label, font=_font(11), fill=WARN)
        return self

    def legend(self, rows: list[tuple[str, str]]) -> "MapCanvas":
        x = self.PAD
        y = self.h - 30
        for text, color in rows:
            self.d.rectangle([x, y + 3, x + 10, y + 13], fill=color)
            self.d.text((x + 15, y), text, font=_font(11), fill=INK)
            x += 16 + int(self.d.textlength(text, font=_font(11))) + 14
        return self

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.img.save(path)
        return path


def draw_timeline(events: list[dict], path: str | Path,
                  duration_s: float, title: str = "") -> Path:
    """Horizontal event strip: stabilise, gate, LED, abort, landing."""
    path = Path(path)
    width, height = 900, 132
    img = Image.new("RGB", (width, height), BG)
    d = ImageDraw.Draw(img)
    d.text((14, 8), title or "ТАЙМЛАЙН ИСПОЛНИТЕЛЯ", font=_font(14, True), fill=INK)

    left, right = 40, width - 30
    y = 58
    d.line([(left, y), (right, y)], fill=GRID, width=2)
    span = max(duration_s, 0.1)

    def xpos(t: float) -> float:
        return left + (right - left) * max(0.0, min(1.0, t / span))

    colors = {
        "mission_loaded": MUTED,
        "stabilize_start": (120, 160, 220),
        "led_command": (250, 220, 90),
        "led_confirmed": GOOD,
        "led_interrupted": BAD,
        "action_blocked": WARN,
        "action_skipped": WARN,
        "waypoint": (90, 130, 170),
        "mission_aborted": BAD,
        "emergency_land_started": BAD,
        "abort_home_arrived": (250, 170, 60),
        "landing_started": MUTED,
        "mission_safely_ended": GOOD,
        "mission_done": GOOD,
        "localization_lost": BAD,
        "link_lost": BAD,
        "forbidden_zone_violation": BAD,
    }
    labels = {
        "led_command": "LED вкл",
        "led_confirmed": "LED 2с",
        "led_interrupted": "LED стоп",
        "mission_aborted": "ПРЕРЫВАНО",
        "emergency_land_started": "авар. посадка",
        "abort_home_arrived": "дома",
        "localization_lost": "нет локализации",
        "link_lost": "нет связи",
        "mission_safely_ended": "посадка",
        "mission_done": "посадка",
    }
    for event in events:
        t = float(event.get("t_s", 0.0))
        name = str(event.get("event", ""))
        color = colors.get(name, GRID)
        x = xpos(t)
        d.line([(x, y - 16), (x, y + 16)], fill=color, width=2)
        text = labels.get(name)
        if text:
            d.text((x + 3, y + 20), text, font=_font(10), fill=color)
        if name in {"mission_aborted", "led_interrupted"} and event.get("reason"):
            d.text((x + 3, y - 34), str(event["reason"])[:34], font=_font(10), fill=color)
    d.text((left, y + 44), "0 с", font=_font(10), fill=MUTED)
    d.text((right - 44, y + 44), f"{span:.0f} с", font=_font(10), fill=MUTED)
    img.save(path)
    return path


def collect_events(journal_path: str | Path) -> list[dict]:
    path = Path(journal_path)
    if not path.exists():
        return []
    events = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def _fmt_float(value, digits: int = 3) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return "—"


def _rel(path) -> str:
    try:
        return str(Path(path).resolve().relative_to(PROJECT_ROOT))
    except (ValueError, OSError):
        return str(path)


def write_protocol(bench_dir: str | Path, results: list, metrics: dict,
                   title: str = "Протокол испытаний АТ11-2") -> Path:
    """B1 protocol: expected / received / duration / proof / retest."""
    bench_dir = Path(bench_dir)
    bench_dir.mkdir(parents=True, exist_ok=True)
    lines = [
        f"# {title}",
        "",
        f"- дата: {iso()}",
        f"- python: {platform.python_version()} ({sys.platform})",
        f"- режим: автоматический стенд приложения, сценарии C1–C8",
        "",
        "| № | Сценарий | Ожидалось | Получено | Задержка | Результат | Доказательство |",
        "|---|---|---|---|---|---|---|",
    ]
    for result in results:
        proof = ", ".join(f"`{_rel(p).split(chr(92))[-1]}`"
                          for p in result.proof.values() if p)
        lines.append(
            f"| {result.cid} | {result.title} | {result.expected} | "
            f"{result.received} | {result.duration_s:.1f} с | "
            f"{'PASS' if result.passed else 'FAIL'} | {proof} |")
    lines += ["", "## Подробности", ""]
    for result in results:
        lines += [f"### {result.cid} · {result.title}", ""]
        lines += [f"- ожидалось: {result.expected}",
                  f"- получено: {result.received}",
                  f"- задержка: {result.duration_s:.2f} с",
                  f"- результат: {'PASS' if result.passed else 'FAIL'}"]
        if result.details:
            lines.append("- шаги:")
            lines += [f"  - {item}" for item in result.details]
        if result.proof:
            lines.append("- доказательства:")
            lines += [f"  - {name}: `{_rel(p)}`" for name, p in result.proof.items()]
        lines.append("")
    if metrics:
        lines += ["## Показатели (Б2)", "", "| Показатель | Значение |", "|---|---|"]
        for name, value in metrics.items():
            lines.append(f"| {name} | {value} |")
        lines.append("")
    path = bench_dir / "protocol.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    write_json(bench_dir / "protocol.json", {
        "title": title,
        "generated_at": iso(),
        "python": platform.python_version(),
        "results": [
            {"id": r.cid, "title": r.title, "expected": r.expected,
             "received": r.received, "passed": r.passed,
             "duration_s": round(r.duration_s, 3),
             "details": r.details,
             "proof": {k: _rel(v) for k, v in r.proof.items()}}
            for r in results
        ],
        "metrics": metrics,
    })
    return path
