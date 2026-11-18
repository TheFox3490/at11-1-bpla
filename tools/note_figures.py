"""Draw the figures of the technical note (PIL only, no matplotlib).

Everything is generated from the same constants the application uses
(``shared.world``, ``sim.scenario``, ``cv.planner``), so a figure can never
show a field, a zone or a route that differs from the program.

Run:  .venv\\Scripts\\python tools\\note_figures.py
Out:  docs/figures/*.png
"""
from __future__ import annotations

import json
import math
import shutil
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from shared.world import (  # noqa: E402
    ALT_MAX, ALT_MIN, FIELD_H, FIELD_W, FORBIDDEN_EXPANSION, HOME, LOCAL_FRAMES,
    NO_FLY, PLOT_SIZE, TARGET_RADIUS, distance_to_polygon, expand_polygon,
)
from cv.planner import plan  # noqa: E402
from sim import scenario  # noqa: E402

OUT = ROOT / "docs" / "figures"

# document palette (light, prints well)
BG = (255, 255, 255)
INK = (26, 31, 40)
MUTED = (105, 114, 128)
LINE = (170, 178, 190)
GRID = (225, 229, 235)
ACCENT = (32, 88, 156)
ACCENT_SOFT = (228, 238, 250)
GOOD = (38, 132, 74)
GOOD_SOFT = (226, 244, 232)
WARN = (176, 118, 16)
WARN_SOFT = (253, 243, 220)
BAD = (184, 52, 52)
BAD_SOFT = (252, 230, 230)
GRAY_SOFT = (241, 243, 246)
PURPLE = (128, 62, 140)


def font(size: int, bold: bool = False):
    name = "arialbd.ttf" if bold else "arial.ttf"
    for candidate in (name, r"C:\Windows\Fonts\\" + name):
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            continue
    return ImageFont.load_default()


def canvas(w: int, h: int):
    img = Image.new("RGB", (w, h), BG)
    return img, ImageDraw.Draw(img)


def center(d, xy, text, fnt, fill=INK):
    d.text(xy, text, font=fnt, fill=fill, anchor="mm")


def wrap(d, text, fnt, width: int) -> list[str]:
    words, lines, line = text.split(), [], ""
    for word in words:
        probe = f"{line} {word}".strip()
        if d.textlength(probe, font=fnt) <= width:
            line = probe
        else:
            if line:
                lines.append(line)
            line = word
    if line:
        lines.append(line)
    return lines


def text_lines(d, text, fnt, width: int) -> list[str]:
    out: list[str] = []
    for chunk in str(text).split("\n"):
        out.extend(wrap(d, chunk, fnt, width) or [""])
    return out


def arrow(d, a, b, color=ACCENT, width=3, head=13, dashed=False):
    if dashed:
        _dashed(d, a, b, color, width)
    else:
        d.line([a, b], fill=color, width=width)
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    p1 = (b[0] - head * math.cos(ang - 0.42), b[1] - head * math.sin(ang - 0.42))
    p2 = (b[0] - head * math.cos(ang + 0.42), b[1] - head * math.sin(ang + 0.42))
    d.polygon([b, p1, p2], fill=color)


def _dashed(d, a, b, color, width=2, dash=10, gap=7):
    length = math.dist(a, b)
    if length == 0:
        return
    ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
    pos = 0.0
    while pos < length:
        end = min(pos + dash, length)
        d.line([(a[0] + ux * pos, a[1] + uy * pos),
                (a[0] + ux * end, a[1] + uy * end)], fill=color, width=width)
        pos = end + gap


def box(d, xy, size, fill=GRAY_SOFT, outline=LINE, radius=12, width=2):
    x, y = xy
    w, h = size
    d.rounded_rectangle([x, y, x + w, y + h], radius=radius, fill=fill,
                        outline=outline, width=width)


def label_box(d, xy, size, title, subtitle="", fill=GRAY_SOFT, outline=LINE,
              title_size=24, sub_size=17, title_fill=INK, sub_fill=MUTED, pad=14):
    box(d, xy, size, fill=fill, outline=outline)
    x, y = xy
    w, h = size
    tlines = text_lines(d, title, font(title_size, True), w - 2 * pad)
    slines = text_lines(d, subtitle, font(sub_size), w - 2 * pad) if subtitle else []
    total = (len(tlines) * (title_size + 6)
             + (10 + len(slines) * (sub_size + 4) if slines else 0))
    cy = y + h / 2 - total / 2 + title_size / 2
    for line in tlines:
        center(d, (x + w / 2, cy), line, font(title_size, True), title_fill)
        cy += title_size + 6
    if slines:
        cy += 8
        for line in slines:
            center(d, (x + w / 2, cy), line, font(sub_size), sub_fill)
            cy += sub_size + 4


# --------------------------------------------------------------------------- #
# field drawing (shared by several figures)
# --------------------------------------------------------------------------- #
class Field:
    def __init__(self, width: int, height: int, pad: int = 110,
                 title: str = "", caption: int = 90):
        self.pad = pad
        self.scale = min((width - 2 * pad) / FIELD_W,
                         (height - 2 * pad - caption) / FIELD_H)
        self.w = int(FIELD_W * self.scale + pad * 2)
        self.h = int(FIELD_H * self.scale + pad * 2 + caption)
        self.img, self.d = canvas(self.w, self.h)
        self.title = title
        self._base()

    def px(self, x: float, y: float) -> tuple[float, float]:
        return (self.pad + x * self.scale,
                self.pad + 46 + (FIELD_H - y) * self.scale)

    def _base(self):
        d = self.d
        if self.title:
            d.text((self.pad, 14), self.title, font=font(26, True), fill=INK)
        x0, y0 = self.px(0, FIELD_H)
        x1, y1 = self.px(FIELD_W, 0)
        d.rectangle([x0, y0, x1, y1], fill=(250, 246, 238), outline=(120, 120, 120),
                    width=2)
        for gx in range(1, int(FIELD_W)):
            gx0 = self.px(gx, 0)[0]
            d.line([(gx0, y0), (gx0, y1)], fill=GRID, width=1)
        for gy in range(1, int(FIELD_H)):
            gy0 = self.px(0, gy)[1]
            d.line([(x0, gy0), (x1, gy0)], fill=GRID, width=1)
        for gx in range(0, int(FIELD_W) + 1):
            gx0 = self.px(gx, 0)[0]
            d.text((gx0, y1 + 8), f"{gx}", font=font(16), fill=MUTED, anchor="ma")
        for gy in range(0, int(FIELD_H) + 1):
            gy0 = self.px(0, gy)[1]
            d.text((x0 - 10, gy0), f"{gy}", font=font(16), fill=MUTED, anchor="rm")
        d.text((x1 + 8, y1 - 12), "x, м", font=font(17), fill=MUTED)
        d.text((x0 - 10, y0 - 30), "y, м", font=font(17), fill=MUTED, anchor="rm")

    def nfz(self, expanded: bool = True, labels: bool = True):
        d = self.d
        pts = [self.px(x, y) for x, y in NO_FLY]
        d.polygon(pts, fill=(252, 214, 214), outline=BAD)
        if expanded:
            exp = expand_polygon(NO_FLY, FORBIDDEN_EXPANSION)
            ep = [self.px(x, y) for x, y in exp]
            d.line(ep + [ep[0]], fill=WARN, width=3)
            if labels:
                mx = sum(p[0] for p in ep) / len(ep)
                my = min(p[1] for p in ep)
                d.text((mx, my - 16), f"NFZ + {FORBIDDEN_EXPANSION:.2f} м",
                       font=font(17, True), fill=WARN, anchor="ma")
        if labels:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            center(d, (cx, cy), "NFZ", font(22, True), BAD)
        return self

    def plots(self, objects, labels: bool = True):
        d = self.d
        half = PLOT_SIZE / 2
        colors = {"A": (206, 84, 84), "B": (74, 116, 196), "U": (150, 150, 150)}
        for obj in objects:
            x, y = float(obj["x"]), float(obj["y"])
            p0 = self.px(x - half, y + half)
            p1 = self.px(x + half, y - half)
            d.rectangle([p0[0], p0[1], p1[0], p1[1]],
                        fill=colors.get(str(obj["klass"]), colors["U"]),
                        outline=(90, 90, 90), width=2)
            center(d, ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2),
                   str(obj["klass"]), font(20, True), (255, 255, 255))
            if labels:
                d.text((p0[0], p0[1] - 24), str(obj.get("id", "")),
                       font=font(15), fill=MUTED)
        return self

    def home(self, label: str = "H"):
        px, py = self.px(*HOME)
        d = self.d
        d.rectangle([px - 13, py - 13, px + 13, py + 13], fill=GOOD,
                    outline=(20, 80, 40), width=2)
        d.text((px + 20, py), label, font=font(20, True), fill=GOOD, anchor="lm")
        return self

    def route(self, points, color=ACCENT, width=4, badges=None):
        d = self.d
        px = [self.px(float(p["x"]), float(p["y"])) for p in points]
        d.line(px, fill=color, width=width, joint="curve")
        if badges:
            for point, index in badges:
                x, y = self.px(float(point["x"]), float(point["y"]))
                d.ellipse([x - 17, y - 17, x + 17, y + 17], fill=BG,
                          outline=color, width=3)
                center(d, (x, y), str(index), font(18, True), color)
        return self

    def scalebar(self, metres: float = 1.0):
        d = self.d
        x0 = self.pad
        y0 = self.px(0, 0)[1] + 40
        x1 = x0 + metres * self.scale
        d.line([(x0, y0), (x1, y0)], fill=INK, width=3)
        d.line([(x0, y0 - 6), (x0, y0 + 6)], fill=INK, width=2)
        d.line([(x1, y0 - 6), (x1, y0 + 6)], fill=INK, width=2)
        d.text(((x0 + x1) / 2, y0 + 6), f"{metres:g} м", font=font(16),
               fill=INK, anchor="ma")

    def legend(self, rows, y: int | None = None, scale: int = 17):
        d = self.d
        x = self.pad
        y = y if y is not None else self.h - 26
        fnt = font(scale)
        for text, style in rows:
            if style == "dash":
                d.line([(x, y + 10), (x + 26, y + 10)], fill=ACCENT, width=4)
                x += 32
            elif style == "x":
                d.line([(x, y + 3), (x + 14, y + 17)], fill=BAD, width=3)
                d.line([(x + 14, y + 3), (x, y + 17)], fill=BAD, width=3)
                x += 20
            else:
                d.rectangle([x, y + 3, x + 14, y + 17], fill=style,
                            outline=(90, 90, 90))
                x += 20
            d.text((x, y), text, font=fnt, fill=INK)
            x += int(d.textlength(text, font=fnt)) + 22

    def save(self, name: str):
        OUT.mkdir(parents=True, exist_ok=True)
        path = OUT / name
        self.img.save(path)
        print("figure:", path.name)
        return path


# --------------------------------------------------------------------------- #
# 1. field with classes and the expanded NFZ
# --------------------------------------------------------------------------- #
def fig_field():
    f = Field(1300, 950, title="Полигон 6 × 4 м: участки, запретная зона, точка старта")
    f.nfz().plots(scenario.default_objects(), labels=False).home().scalebar()
    f.legend([("класс A — операция разрешена", (206, 84, 84)),
              ("класс B — только наблюдение", (74, 116, 196)),
              ("класс U — класс не определён", (150, 150, 150)),
              (f"NFZ + {FORBIDDEN_EXPANSION:.2f} м", WARN)])
    f.save("fig_field.png")


# --------------------------------------------------------------------------- #
# 2. data flow / responsibility boundary
# --------------------------------------------------------------------------- #
def fig_dataflow():
    img, d = canvas(1800, 720)
    d.text((50, 34), "Обмен данными и границы ответственности",
           font=font(30, True), fill=INK)
    W, H = 290, 140
    top, bot = 150, 430
    xs_top = [50, 400, 750, 1100]
    xs_bot = [400, 750, 1100]

    top_blocks = [
        ("БПЛА-разведчик", "облёт «змейкой»\n25 кадров + позы\nкалибровочный кадр"),
        ("Обработка (ПК)", "калибровка L→F\nдетекция и слияние\nкарта наблюдений"),
        ("Валидатор", "checksum, единицы, TTL\nверсия калибровки,\nNFZ + 0.23 м, ошибка"),
        ("Планировщик", "2 метода обхода,\nбюджет и приоритеты,\nфайл + checksum"),
    ]
    for i, (title, sub) in enumerate(top_blocks):
        label_box(d, (xs_top[i], top), (W, H), title, sub,
                  fill=GRAY_SOFT if i == 0 else ACCENT_SOFT,
                  title_size=24, sub_size=16)

    gap_labels = ["кадры + позы\n+ время", "карта\nнаблюдений", "только\nкласс A"]
    for i, text in enumerate(gap_labels):
        x = (xs_top[i] + W + xs_top[i + 1]) / 2
        arrow(d, (xs_top[i] + W + 6, top + H / 2),
              (xs_top[i + 1] - 6, top + H / 2))
        for j, line in enumerate(text.split("\n")):
            d.text((x, top + H + 12 + j * 21), line, font=font(15, True),
                   fill=ACCENT, anchor="ma")

    # file crosses the responsibility boundary
    d.line([(40, 370), (1400, 370)], fill=BAD, width=2)
    d.text((44, 378), "Граница ответственности: между ролями передаётся только файл миссии, а не координаты в памяти",
           font=font(18, True), fill=BAD)

    label_box(d, (xs_bot[2], bot), (W, H), "БПЛА-исполнитель",
              "полёт, стабилизация 1 с,\nшлюз, LED 2 с,\nвозврат и посадка",
              fill=GRAY_SOFT, title_size=24, sub_size=16)
    label_box(d, (xs_bot[1], bot), (W, H), "Журнал и реестр",
              "journal.jsonl, summary.json,\naction_id → confirmed /\nuncertain / aborted",
              fill=WARN_SOFT, outline=WARN, title_size=24, sub_size=16)
    label_box(d, (xs_bot[0], bot), (W, H), "Протокол испытаний",
              "C1–C8, метрики, карты,\nтаймлайны, снимки",
              fill=GOOD_SOFT, outline=GOOD, title_size=24, sub_size=16)

    arrow(d, (xs_bot[2] + W // 2, top + H + 6),
          (xs_bot[2] + W // 2, bot - 6), ACCENT, 3, 14)
    d.text((xs_bot[2] + W // 2 + 12, 300), "миссия.json", font=font(17, True),
           fill=ACCENT)
    d.text((xs_bot[2] + W // 2 + 12, 322), "(F, метры, checksum)", font=font(16),
           fill=ACCENT)

    arrow(d, (xs_bot[2] - 6, bot + H / 2), (xs_bot[1] + W + 6, bot + H / 2))
    d.text((xs_bot[1] + W / 2 + 20, bot + H + 12), "каждое решение",
           font=font(15), fill=MUTED)
    arrow(d, (xs_bot[1] - 6, bot + H / 2), (xs_bot[0] + W + 6, bot + H / 2))
    d.text((xs_bot[0] + W / 2 + 20, bot + H + 12), "сводка",
           font=font(15), fill=MUTED)

    label_box(d, (1430, 130), (330, 210), "Эталонная раскладка",
              "не передаётся ни в CV,\nни в валидатор, ни\nв планировщик:\nдоступна только\nпротоколу",
              fill=BAD_SOFT, outline=BAD, title_size=22, sub_size=16)
    d.text((50, 630), "Один и тот же файл миссии — вход исполнителя и доказательство "
                      "для жюри: его можно проверить отдельно от программы.",
           font=font(19), fill=MUTED)
    d.text((50, 662), "Ручной ввод координат целей отсутствует: режим 2 меняет только "
                      "полигон, координаты считает CV.", font=font(19), fill=MUTED)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_dataflow.png")
    print("figure: fig_dataflow.png")


# --------------------------------------------------------------------------- #
# 3. local frames
# --------------------------------------------------------------------------- #
def fig_frames():
    f = Field(1200, 930, title="Две локальные системы и общее поле F", caption=130)
    f.nfz(expanded=False).plots(scenario.default_objects(), labels=False).home()
    d = f.d
    for name, color, tag in (("L_recon", ACCENT, "R"), ("L_exec", PURPLE, "E")):
        spec = LOCAL_FRAMES[name]
        ox, oy = spec["origin"]
        yaw = math.radians(spec["yaw_deg"])
        px, py = f.px(ox, oy)
        ln = 84
        for delta, wid in ((0.0, 4), (math.pi / 2, 3)):
            end = (px + ln * math.cos(yaw + delta), py - ln * math.sin(yaw + delta))
            arrow(d, (px, py), end, color, wid, 11)
        d.ellipse([px - 10, py - 10, px + 10, py + 10], fill=BG, outline=color,
                  width=3)
        center(d, (px, py), tag, font(19, True), color)

    cap_y = f.h - 118
    caps = [
        ("R — L_recon: начало (0.20, 0.30) м, курс +7°   ·   "
         "E — L_exec: начало (5.55, 3.70) м, курс −160°", font(19, True), ACCENT),
        ("Это начала координатных систем, а не позиции дронов. "
         "Перевод L → F: поворот, масштаб и сдвиг по калибровочным маркерам.",
         font(18), MUTED),
        ("Оси показаны стрелками: x — вперёд по курсу, y — влево; "
         "системы не считаются совпадающими.", font(18), MUTED),
    ]
    for text, fnt, color in caps:
        for line in text_lines(d, text, fnt, f.w - 2 * f.pad):
            d.text((f.pad, cap_y), line, font=fnt, fill=color)
            cap_y += 27
    f.save("fig_frames.png")


# --------------------------------------------------------------------------- #
# 4. CV pipeline
# --------------------------------------------------------------------------- #
def fig_cv():
    img, d = canvas(1800, 400)
    d.text((50, 34), "Обработка кадров: от снимка до карты наблюдений",
           font=font(30, True), fill=INK)
    steps = [
        ("1. Кадр", "640×480, надир,\nпоза L и время"),
        ("2. Выпрямление", "перспектива участка\n→ квадрат в метрах"),
        ("3. Признаки", "белая карточка,\nцвет + форма"),
        ("4. Оценка", "score = f(цвет, форма,\nконтраст) — не вероятность"),
        ("5. Слияние", "кластер 0.20 м,\nголоса по кадрам"),
        ("6. Класс", "A / B, конфликт → U,\nразброс → ошибка"),
    ]
    w, h, y = 260, 155, 120
    for i, (title, sub) in enumerate(steps):
        x = 40 + i * 290
        label_box(d, (x, y), (w, h), title, sub,
                  fill=ACCENT_SOFT if i < 4 else GOOD_SOFT,
                  title_size=22, sub_size=16)
        if i:
            arrow(d, (x - 27, y + h / 2), (x - 5, y + h / 2), ACCENT, 3, 11)
    d.text((50, 320), "Координаты в F получаются только пересчётом L→F по калибровке; "
                      "ручного ввода координат в конвейере нет.",
           font=font(19), fill=MUTED)
    d.text((50, 352), "Разные классы для одного места → объект U и пропуск; "
                      "разброс наблюдений → вклад в ошибку цели.",
           font=font(19), fill=MUTED)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_cv.png")
    print("figure: fig_cv.png")


# --------------------------------------------------------------------------- #
# 5. route with numbered targets
# --------------------------------------------------------------------------- #
def _distance(x, y):
    from shared.world import distance_to_polygon
    return distance_to_polygon(x, y, NO_FLY)


def _plan_for_default_field():
    objects = scenario.default_objects()
    targets = []
    for obj in objects:
        if obj["klass"] != "A" or _distance(obj["x"], obj["y"]) <= FORBIDDEN_EXPANSION:
            continue
        targets.append({
            "target_id": f"T{len(targets) + 1:02d}",
            "action_id": f"note-{obj['id']}",
            "klass": "A",
            "point": {"x": obj["x"], "y": obj["y"]},
            "position_error_m": 0.07,
            "priority": len(targets) + 1,
            "source_object_id": obj["id"],
        })
    return plan(HOME, targets, forbidden=NO_FLY)


def fig_route():
    result = _plan_for_default_field()
    f = Field(1300, 950, title=(
        f"Маршрут: {len(result['targets'])} целей, {result['length_m']:.2f} м, "
        f"оценка {result['estimated_s']:.0f} с + резерв {result['reserve_s']:.0f} с"))
    f.nfz()
    f.plots(scenario.default_objects(), labels=False)
    # class A targets inside the forbidden zone are excluded by the planner
    for obj in scenario.default_objects():
        if obj["klass"] == "A" and distance_to_polygon(
                obj["x"], obj["y"], NO_FLY) <= FORBIDDEN_EXPANSION:
            x, y = f.px(obj["x"], obj["y"])
            d = f.d
            d.line([(x - 16, y - 16), (x + 16, y + 16)], fill=BAD, width=5)
            d.line([(x + 16, y - 16), (x - 16, y + 16)], fill=BAD, width=5)
    f.route(result["route"], badges=[(t["point"], i)
                                     for i, t in enumerate(result["targets"], 1)])
    f.home().scalebar()
    f.legend([("маршрут", "dash"),
              ("цель класса A", (206, 84, 84)),
              (f"NFZ + {FORBIDDEN_EXPANSION:.2f} м", WARN),
              ("старт и посадка", GOOD),
              ("A в NFZ — исключена", "x")])
    f.save("fig_route.png")


# --------------------------------------------------------------------------- #
# 6. executor state machine
# --------------------------------------------------------------------------- #
def fig_executor():
    img, d = canvas(1680, 1180)
    d.text((50, 30), "Автомат исполнителя", font=font(30, True), fill=INK)
    W, H = 300, 76
    xm = 200
    ys = [90, 190, 290, 390, 490, 590]

    steps = [("загрузка", "чтение файла и checksum"),
             ("проверка", "7 условий карты"),
             ("перелёт", "к точке маршрута"),
             ("стабилизация", "1 с в радиусе 0.25 м"),
             ("шлюз", "допуск операции"),
             ("LED 2 с", "перепроверка каждые 0.1 с")]
    for i, (title, sub) in enumerate(steps):
        label_box(d, (xm, ys[i]), (W, H), title, sub,
                  title_size=20, sub_size=14)
        if i:
            arrow(d, (xm + W / 2, ys[i] - 24), (xm + W / 2, ys[i] - 4))

    # right column: confirm / skip
    xr = 620
    label_box(d, (xr, 590), (W, H), "подтверждение", "2 с выдержаны",
              fill=GOOD_SOFT, outline=GOOD, title_size=20, sub_size=14)
    arrow(d, (xm + W + 4, 628), (xr - 4, 628), ACCENT, 3, 12)
    d.text((xm + W + 12, 606), "разрешено", font=font(13), fill=GOOD)

    label_box(d, (xr, 490), (W, H), "пропуск", "запрет или дубль",
              fill=WARN_SOFT, outline=WARN, title_size=20, sub_size=14)
    arrow(d, (xm + W + 4, 528), (xr - 4, 528), WARN, 3, 12)
    d.text((xm + W + 12, 506), "запрещено", font=font(13), fill=WARN)

    # both lead to the "any targets left?" decision
    label_box(d, (xr, 700), (W, H), "остались цели?", "индекс + 1",
              fill=ACCENT_SOFT, title_size=20, sub_size=14)
    arrow(d, (xr + W / 2, 666), (xr + W / 2, 700))
    d.text((xr + W / 2 + 8, 678), "засчитано", font=font(13), fill=MUTED)

    # skip bypasses the confirmation box on the right lane
    d.line([(xr + W + 4, 528), (960, 528)], fill=WARN, width=3)
    d.line([(960, 528), (960, 760)], fill=WARN, width=3)
    arrow(d, (960, 760), (xr + W + 4, 760), WARN, 3, 12)

    # loop back to cruise
    d.line([(xr - 4, 738), (120, 738), (120, 328)], fill=ACCENT, width=3)
    arrow(d, (120, 328), (xm - 4, 328), ACCENT, 3, 12)
    d.text((150, 716), "да, есть ещё цели", font=font(15, True), fill=ACCENT)

    # normal finish
    label_box(d, (xr, 830), (W, H), "возврат", "на точку старта",
              title_size=20, sub_size=14)
    arrow(d, (xr + W / 2, 776), (xr + W / 2, 830))
    d.text((xr + W / 2 + 8, 794), "цели кончились", font=font(13), fill=MUTED)
    label_box(d, (xr, 930), (W, H), "посадка", "снижение до 0.05 м",
              fill=GOOD_SOFT, outline=GOOD, title_size=20, sub_size=14)
    arrow(d, (xr + W / 2, 906), (xr + W / 2, 930))
    label_box(d, (xr, 1030), (W, H), "готово", "сводка и журнал",
              fill=GOOD_SOFT, outline=GOOD, title_size=20, sub_size=14)
    arrow(d, (xr + W / 2, 1006), (xr + W / 2, 1030))

    # abort branch
    xa = 990
    label_box(d, (xa, 90), (300, 76), "аномалия",
              "карта, локализация, связь,\nбюджет, NFZ",
              fill=BAD_SOFT, outline=BAD, title_size=20, sub_size=14)
    label_box(d, (xa, 190), (300, 76), "LED выключен", "немедленно",
              fill=BAD_SOFT, outline=BAD, title_size=20, sub_size=14)
    arrow(d, (xa + 150, 166), (xa + 150, 190), BAD, 3, 12)
    label_box(d, (xa, 300), (300, 84), "навигация достоверна?",
              "свежая локализация и связь",
              fill=ACCENT_SOFT, title_size=19, sub_size=14)
    arrow(d, (xa + 150, 266), (xa + 150, 300), BAD, 3, 12)
    label_box(d, (xa, 410), (300, 76), "возврат на базу",
              "abort_home", fill=GOOD_SOFT, outline=GOOD,
              title_size=20, sub_size=14)
    arrow(d, (xa + 90, 384), (xa + 110, 410), GOOD, 3, 12)
    d.text((xa + 30, 390), "да", font=font(15, True), fill=GOOD)
    label_box(d, (xa + 340, 410), (300, 76), "посадка на месте",
              "emergency_land", fill=WARN_SOFT, outline=WARN,
              title_size=20, sub_size=14)
    d.line([(xa + W + 4, 342), (xa + 490, 342)], fill=WARN, width=3)
    arrow(d, (xa + 490, 342), (xa + 490, 410), WARN, 3, 12)
    d.text((xa + W + 12, 318), "нет", font=font(15, True), fill=WARN)

    label_box(d, (xa + 160, 620), (340, 80), "посадка выполнена",
              "статус aborted, причина в журнале",
              fill=GRAY_SOFT, title_size=18, sub_size=13)
    arrow(d, (xa + 140, 486), (xa + 240, 620), GOOD, 2, 11)
    arrow(d, (xa + 490, 486), (xa + 420, 620), WARN, 2, 11)

    # aborted mission still ends on the landing box
    d.line([(xa + 160, 660), (xa, 660), (xa, 968)], fill=MUTED, width=2)
    arrow(d, (xa, 968), (xr + W + 4, 968), MUTED, 2, 10)
    d.text((xa + 6, 812), "после\nпосадки", font=font(12), fill=MUTED)

    # "in any state" bracket
    d.line([(596, 82), (596, 672)], fill=BAD, width=2)
    for yy in (82, 672):
        d.line([(596, yy), (608, yy)], fill=BAD, width=2)
    _dashed(d, (600, 300), (xa - 6, 128), BAD, 2, 10, 8)
    d.text((660, 246), "в любом состоянии", font=font(15), fill=BAD)

    d.text((50, 1140), "aborted — безопасное завершение, а не сбой: дрон сел, "
                      "причина записана, повтор автоматически не выполняется.",
           font=font(17, True), fill=BAD)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_executor.png")
    print("figure: fig_executor.png")


# --------------------------------------------------------------------------- #
# 7. operation gate
# --------------------------------------------------------------------------- #
def fig_gate():
    img, d = canvas(1600, 900)
    d.text((50, 30), "Шлюз допуска операции: все условия одновременно",
           font=font(30, True), fill=INK)
    rows = [
        ("1. Карта валидна и не просрочена", "checksum, версия калибровки, TTL 15 мин"),
        ("2. Цель класса A и вне NFZ + 0.23 м", "и сам дрон вне расширенной зоны"),
        (f"3. Дрон в радиусе {TARGET_RADIUS:.2f} м, высота {ALT_MIN:.1f}–{ALT_MAX:.1f} м",
         "область воздействия вокруг цели"),
        ("4. Устойчиво ≥ 1 с внутри области", "счётчик стабилизации"),
        ("5. Бюджет точности выполнен", "e_цели + 0.06 + 0.02 + 0.04 ≤ 0.25 м"),
        ("6. Действие ещё не подтверждено", "реестр: нет confirmed и uncertain"),
        ("7. Локализация ≤ 0.3 с, связь есть", "свежесть решения и канал управления"),
    ]
    y = 100
    for title, sub in rows:
        label_box(d, (60, y), (700, 88), title, sub, title_size=21, sub_size=16,
                  fill=ACCENT_SOFT)
        y += 104
    gx, gy = 880, 430
    d.polygon([(gx, gy - 100), (gx + 130, gy - 65), (gx + 130, gy + 65),
               (gx, gy + 100)], fill=ACCENT_SOFT, outline=ACCENT, width=2)
    center(d, (gx + 55, gy), "И", font(44, True), ACCENT)
    for i in range(7):
        yy = 144 + i * 104
        _dashed(d, (762, yy), (gx, gy - 92 + i * 30.6), ACCENT, 2, 8, 6)
    label_box(d, (1080, 330), (470, 110), "LED включается на 2 с",
              "успех = 2 с непрерывного разрешённого действия",
              fill=GOOD_SOFT, outline=GOOD, title_size=25, sub_size=17)
    arrow(d, (1012, 408), (1080, 380), GOOD, 4, 14)
    label_box(d, (1080, 500), (470, 130), "операция запрещена",
              "LED не включается; цель пропускается\nили миссия завершается безопасно",
              fill=BAD_SOFT, outline=BAD, title_size=25, sub_size=17)
    arrow(d, (1012, 450), (1080, 540), BAD, 3, 13)
    d.text((1080, 470), "хотя бы одно «нет»", font=font(17, True), fill=BAD)
    d.text((60, 848), "Условия проверяются до включения и во время свечения: "
                      "нарушение любого — немедленное выключение.",
           font=font(18), fill=MUTED)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_gate.png")
    print("figure: fig_gate.png")


# --------------------------------------------------------------------------- #
# 8. economics
# --------------------------------------------------------------------------- #
def fig_economics():
    img, d = canvas(1250, 900)
    d.text((50, 30), "Стоимость цикла: сплошная обработка и разделение ролей",
           font=font(26, True), fill=INK)
    left, right, top, bottom = 120, 1180, 130, 760
    max_cost = 320.0

    def xpos(s):
        return left + (right - left) * s

    def ypos(cost):
        return bottom - (bottom - top) * cost / max_cost

    for cost in range(0, 321, 40):
        y = ypos(cost)
        d.line([(left, y), (right, y)], fill=GRID, width=1)
        d.text((left - 12, y), f"{cost}", font=font(17), fill=MUTED, anchor="rm")
    for s in range(0, 101, 10):
        x = xpos(s / 100)
        d.line([(x, bottom), (x, bottom + 6)], fill=LINE, width=2)
        d.text((x, bottom + 12), f"{s}", font=font(17), fill=MUTED, anchor="ma")
    d.line([(left, bottom), (right, bottom)], fill=INK, width=2)
    d.line([(left, top), (left, bottom)], fill=INK, width=2)
    d.text(((left + right) / 2, bottom + 46), "доля целевых участков, %",
           font=font(19), fill=INK, anchor="ma")
    d.text((left - 8, top - 24), "тыс. ₽", font=font(18), fill=INK, anchor="ls")

    d.line([(xpos(0), ypos(216)), (xpos(1), ypos(216))], fill=BAD, width=4)
    d.text((xpos(0.02), ypos(216) - 28), "сплошная операция: 1800 × 120 = 216",
           font=font(18, True), fill=BAD)

    pts = [(xpos(s / 100), ypos(45 + 264 * s / 100)) for s in range(0, 101, 2)]
    d.line(pts, fill=GOOD, width=4)
    d.text((xpos(0.02), ypos(196)), "разделение: 45 + 264 × s",
           font=font(18, True), fill=GOOD)
    arrow(d, (xpos(0.05), ypos(196) + 32), (xpos(0.253), ypos(113)), GOOD, 2, 9,
          dashed=True)

    sx, sy = xpos(0.6477), ypos(216)
    _dashed(d, (sx, sy), (sx, bottom), MUTED, 2, 8, 6)
    d.ellipse([sx - 8, sy - 8, sx + 8, sy + 8], fill=WARN, outline=INK)
    d.text((sx + 16, sy + 34), "безубыточность 64.8 %", font=font(19, True),
           fill=WARN)

    x20, y20 = xpos(0.20), ypos(97.8)
    d.ellipse([x20 - 7, y20 - 7, x20 + 7, y20 + 7], fill=GOOD)
    arrow(d, (xpos(0.225), ypos(86)), (x20 + 7, y20 + 8), GOOD, 2, 10)
    d.text((xpos(0.235), ypos(76)), "20 % → 97.8 тыс. ₽", font=font(18, True),
           fill=GOOD)
    d.text((xpos(0.235), ypos(76) + 24), "экономия 118.2 тыс. ₽ (54.7 %)",
           font=font(17), fill=GOOD)

    d.rectangle([left + 20, top + 16, left + 44, top + 28], fill=BAD)
    d.text((left + 52, top + 10), "сплошная операция", font=font(18), fill=INK)
    d.rectangle([left + 20, top + 48, left + 44, top + 60], fill=GOOD)
    d.text((left + 52, top + 42), "разведка + локальная операция",
           font=font(18), fill=INK)
    d.text((50, 840), "Ниже 64.8 % целей разделение дешевле; выше — дешевле обработать всё поле.",
           font=font(18), fill=MUTED)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_economics.png")
    print("figure: fig_economics.png")


# --------------------------------------------------------------------------- #
# 9. abort decision
# --------------------------------------------------------------------------- #
def fig_abort():
    img, d = canvas(1600, 700)
    d.text((50, 30), "Безопасное завершение: что происходит после аномалии",
           font=font(28, True), fill=INK)
    label_box(d, (50, 150), (350, 390), "Триггеры",
              "карта просрочена\nпотеря локализации\n> 0.3 с\nпотеря связи\n"
              "бюджет исчерпан\nвход в NFZ\nотмена оператором",
              fill=BAD_SOFT, outline=BAD, title_size=24, sub_size=18)
    label_box(d, (470, 250), (300, 120), "LED выключен",
              "немедленно, до любых\nдругих действий", fill=WARN_SOFT,
              outline=WARN, title_size=23, sub_size=17)
    arrow(d, (402, 345), (470, 310), BAD, 4, 14)
    label_box(d, (850, 250), (340, 120), "навигация достоверна?",
              "свежая локализация и связь", fill=ACCENT_SOFT,
              title_size=22, sub_size=17)
    arrow(d, (770, 310), (850, 310), ACCENT, 3, 13)
    label_box(d, (1270, 120), (290, 120), "возврат на базу",
              "abort_home → посадка", fill=GOOD_SOFT, outline=GOOD,
              title_size=22, sub_size=17)
    arrow(d, (1190, 285), (1300, 240), GOOD, 3, 13)
    d.text((1204, 216), "да", font=font(19, True), fill=GOOD)
    label_box(d, (1270, 390), (290, 130), "посадка на месте",
              "emergency_land:\nнавигация ненадёжна", fill=WARN_SOFT,
              outline=WARN, title_size=22, sub_size=16)
    arrow(d, (1190, 340), (1300, 420), WARN, 3, 13)
    d.text((1198, 382), "нет", font=font(19, True), fill=WARN)

    d.text((470, 500), "В обоих случаях:", font=font(19, True), fill=INK)
    d.text((470, 532),
           "в журнал пишется mission_aborted (причина, время, координаты, возраст "
           "измерения);", font=font(18), fill=MUTED)
    d.text((470, 560),
           "незавершённая операция получает статус aborted и не повторяется "
           "автоматически;", font=font(18), fill=MUTED)
    d.text((470, 588),
           "миссия заканчивается посадкой, сводка сохраняется, статус — aborted, "
           "а не failed.", font=font(18), fill=MUTED)
    d.text((50, 650), "Прерывание — это штатный сценарий, а не сбой программы.",
           font=font(19), fill=MUTED)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_abort.png")
    print("figure: fig_abort.png")


# --------------------------------------------------------------------------- #
# 10. three modes
# --------------------------------------------------------------------------- #
def fig_modes():
    img, d = canvas(1500, 400)
    d.text((50, 30), "Три режима приложения", font=font(30, True), fill=INK)
    cards = [
        ("1 · Свободный полёт", "F1", "ручное управление,\nпроверка сцены и камеры",
         GRAY_SOFT, LINE),
        ("2 · Конструктор поля", "F2", "только полигон и классы;\nкоординаты целей не вводятся",
         WARN_SOFT, WARN),
        ("3 · Рабочий сценарий", "F3", "разведка → CV → миссия →\nисполнитель → протокол",
         ACCENT_SOFT, ACCENT),
    ]
    for i, (title, key, sub, fill, edge) in enumerate(cards):
        x = 50 + i * 490
        label_box(d, (x, 110), (440, 190), title, sub, fill=fill, outline=edge,
                  title_size=26, sub_size=18)
        d.rounded_rectangle([x + 358, 124, x + 418, 164], radius=8, fill=BG,
                            outline=edge, width=2)
        center(d, (x + 388, 144), key, font(21, True), edge)
    d.text((50, 330), "Стенд испытаний C1–C8 (F9) — сервисный оверлей над этими "
                      "режимами, а не четвёртый режим.", font=font(19), fill=MUTED)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_modes.png")
    print("figure: fig_modes.png")


# --------------------------------------------------------------------------- #
# 11. real timeline from the C3 journal
# --------------------------------------------------------------------------- #
def _latest_bench() -> Path | None:
    runs = ROOT / "runs" / "tests"
    if not runs.exists():
        return None
    dirs = [p for p in runs.iterdir() if p.is_dir()]
    return max(dirs, key=lambda p: p.stat().st_mtime) if dirs else None


def _read_events(path: Path) -> list[dict]:
    events = []
    if not path.exists():
        return events
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def _abort_run():
    """Take the C3 run referenced by the last protocol (stable, no mtime games)."""
    bench = _latest_bench()
    if bench is None:
        return None
    protocol = bench / "protocol.json"
    if not protocol.exists():
        return None
    try:
        results = json.loads(protocol.read_text(encoding="utf-8")).get("results", [])
    except (OSError, ValueError):
        return None
    for result in results:
        if result.get("id") != "C3":
            continue
        rel = (result.get("proof") or {}).get("сводка")
        if not rel:
            continue
        summary_path = ROOT / rel
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        events = _read_events(summary_path.parent / "journal.jsonl")
        if events:
            return events, summary
    return None


def fig_timeline():
    data = _abort_run()
    if data is None:
        print("figure: fig_timeline.png skipped (no abort run)")
        return
    events, summary = data
    total = max(float(summary.get("t_end_s") or 0.0), 1.0)
    img, d = canvas(1500, 340)
    d.text((50, 26), "Таймлайн реального прогона C3: просрочка карты в полёте",
           font=font(24, True), fill=INK)
    left, right, bar_y, bar_h = 90, 1440, 120, 48

    def xpos(t):
        return left + (right - left) * min(1.0, max(0.0, t / total))

    bands, phase, start = [], "перелёт", 0.0
    for event in events:
        name, t = event.get("event", ""), float(event.get("t_s") or 0.0)
        new = None
        if name == "stabilize_start":
            new = "стабилизация"
        elif name == "led_command":
            new = "LED"
        elif name in ("led_confirmed", "waypoint"):
            new = "перелёт"
        elif name == "mission_aborted":
            new = "возврат" if event.get("mode") != "land" else "аварийная посадка"
        elif name in ("abort_home_arrived", "landing_started", "emergency_land_started"):
            new = "посадка"
        if new and new != phase:
            bands.append((start, t, phase))
            phase, start = new, t
    bands.append((start, total, phase))
    palette = {"перелёт": ACCENT_SOFT, "стабилизация": (255, 244, 214),
               "LED": (255, 232, 150), "возврат": (255, 224, 205),
               "аварийная посадка": BAD_SOFT, "посадка": GOOD_SOFT}
    for a, b, name in bands:
        x0, x1 = xpos(a), xpos(b)
        if x1 - x0 < 1:
            continue
        d.rectangle([x0, bar_y, x1, bar_y + bar_h],
                    fill=palette.get(name, GRAY_SOFT), outline=LINE)
        if x1 - x0 > 74:
            name_font = font(17, True)
            if x1 - x0 > d.textlength(name, font=name_font) + 16:
                center(d, ((x0 + x1) / 2, bar_y + bar_h / 2), name,
                       name_font, INK)

    marked = {
        "waypoint": ("точка маршрута", ACCENT),
        "stabilize_start": ("стабилизация", WARN),
        "led_command": ("LED вкл", (140, 110, 10)),
        "led_confirmed": ("LED 2 с", GOOD),
        "mission_aborted": ("ПРЕРВАНО", BAD),
        "abort_home_arrived": ("на базе", GOOD),
        "landing_started": ("посадка", MUTED),
        "mission_safely_ended": ("сели", GOOD),
    }
    fnt = font(15)
    taken: list[tuple[float, float, float, float]] = []
    seen: set[str] = set()
    for event in events:
        name = event.get("event", "")
        if name not in marked:
            continue
        if name in seen:
            continue
        seen.add(name)
        t = float(event.get("t_s") or 0.0)
        label, color = marked[name]
        px = xpos(t)
        width = d.textlength(label, font=fnt)
        tx = min(px + 4, right - width)
        level = 0
        while level < 6:
            rect = (tx - 3, bar_y + bar_h + 18 + level * 21,
                    tx + width + 3, bar_y + bar_h + 18 + level * 21 + 16)
            clash = any(not (rect[2] < r[0] or r[2] < rect[0]
                             or rect[3] < r[1] or r[3] < rect[1])
                        for r in taken)
            if not clash:
                break
            level += 1
        taken.append((tx - 3, bar_y + bar_h + 18 + level * 21,
                      tx + width + 3, bar_y + bar_h + 18 + level * 21 + 16))
        d.line([(px, bar_y - 12), (px, bar_y + bar_h + 12)], fill=color, width=2)
        d.text((tx, bar_y + bar_h + 18 + level * 21), label,
               font=fnt, fill=color)

    d.text((left, bar_y - 46), "0 с", font=font(17), fill=MUTED)
    d.text((right, bar_y - 46), f"{total:.0f} с", font=font(17),
           fill=MUTED, anchor="ra")
    confirmed = sum(1 for a in summary.get("actions", [])
                    if a.get("status") == "confirmed")
    d.text((50, 290), f"Причина: {summary.get('abort_reason', '—')}. "
                      f"Подтверждено операций: {confirmed}. "
                      f"Статус: {summary.get('status')}. Посадка выполнена.",
           font=font(18), fill=MUTED)
    OUT.mkdir(parents=True, exist_ok=True)
    img.save(OUT / "fig_timeline.png")
    print("figure: fig_timeline.png")


# --------------------------------------------------------------------------- #
# 12. copy real screenshots / maps from the last bench run
# --------------------------------------------------------------------------- #
def copy_evidence():
    OUT.mkdir(parents=True, exist_ok=True)
    bench = _latest_bench()
    pairs = []
    if bench is not None:
        pairs = [
            (bench / "map_C1.png", "fig_map_c1.png"),
            (bench / "shots" / "C3_result.png", "fig_bench_c3.png"),
        ]
    for src, name in pairs:
        if src.exists():
            shutil.copyfile(src, OUT / name)
            print("copy:", name)


# --------------------------------------------------------------------------- #
def main():
    fig_field()
    fig_dataflow()
    fig_frames()
    fig_cv()
    fig_route()
    fig_executor()
    fig_gate()
    fig_economics()
    fig_abort()
    fig_modes()
    fig_timeline()
    copy_evidence()


if __name__ == "__main__":
    main()
