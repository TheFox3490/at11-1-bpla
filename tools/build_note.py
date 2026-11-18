"""Build the DOCX technical note from ``docs/07_technical_note.md``.

Only python-docx is used: no Word, no LibreOffice, no pandoc.  The markdown
subset is deliberately small and explicit:

    [COVER] ... [/COVER]   title page (@BIG / @MID / @SMALL / @SPACE lines)
    [PAGEBREAK]            hard page break
    [TOC]                  table of contents (Word field, updated on open)
    # / ## / ###           headings (built-in styles, so the TOC sees them)
    - item                 bullet
    1. item                numbered item (numbers are kept as typed)
    | a | b |              table; a preceding ``TABLE: caption`` sets the caption
    ![caption](path, 15)   figure with automatic number; width in cm
    ```...```              code block
    [[C1C8_TABLE]]         protocol table from the latest bench run
    [[METRICS_TABLE]]      metrics table from the latest bench run
    [[C1C8_DETAILS]]       per-scenario details and proof paths
    [[PROTOCOL_INFO]]      "date, run id" inline
    [[FIG:name]]           inline "рисунок N"
    [[TAB:name]]           inline "таблица N"

Run:  .venv\\Scripts\\python tools\\build_note.py
Out:  docs/Техническая_записка_Агрокидсы_АТ11-2.docx
"""
from __future__ import annotations

import json
import re
import sys
from datetime import datetime
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "docs" / "07_technical_note.md"
OUTPUT = ROOT / "docs" / "Техническая_записка_Агрокидсы_АТ11-2.docx"

BODY_FONT = "Times New Roman"
CODE_FONT = "Consolas"
BODY_SIZE = 14
TABLE_SIZE = 12
CAPTION_SIZE = 12

FIGURES: dict[str, int] = {}
TABLES: dict[str, int] = {}
figure_counter = 0
table_counter = 0


# --------------------------------------------------------------------------- #
# low level helpers
# --------------------------------------------------------------------------- #
def set_run_font(run, name: str = BODY_FONT, size: int = BODY_SIZE,
                 bold: bool | None = None, italic: bool | None = None,
                 color: RGBColor | None = None):
    run.font.name = name
    run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if italic is not None:
        run.font.italic = italic
    if color is not None:
        run.font.color.rgb = color
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    for attr in ("w:ascii", "w:hAnsi", "w:cs", "w:eastAsia"):
        rfonts.set(qn(attr), name)
    lang = rpr.find(qn("w:lang"))
    if lang is None:
        lang = OxmlElement("w:lang")
        rpr.append(lang)
    lang.set(qn("w:val"), "ru-RU")
    lang.set(qn("w:cs"), "ru-RU")


def shade(element, fill: str):
    if hasattr(element, "_p"):
        pr = element._p.get_or_add_pPr()
    elif hasattr(element, "get_or_add_pPr"):
        pr = element.get_or_add_pPr()
    else:
        pr = element
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    pr.append(shd)


def shade_cell(cell, fill: str):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    tc_pr.append(shd)


def repeat_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    el = OxmlElement("w:tblHeader")
    el.set(qn("w:val"), "true")
    tr_pr.append(el)


def no_split(row):
    tr_pr = row._tr.get_or_add_trPr()
    el = OxmlElement("w:cantSplit")
    el.set(qn("w:val"), "true")
    tr_pr.append(el)


def add_field(paragraph, instruction: str, placeholder: str = "",
              size: int = 12):
    run = paragraph.add_run()
    set_run_font(run, size=size)
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = instruction
    sep = OxmlElement("w:fldChar")
    sep.set(qn("w:fldCharType"), "separate")
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.append(begin)
    run._r.append(instr)
    run._r.append(sep)
    if placeholder:
        text_run = paragraph.add_run(placeholder)
        set_run_font(text_run, size=size)
    tail = paragraph.add_run()
    set_run_font(tail, size=size)
    tail._r.append(end)


# --------------------------------------------------------------------------- #
# document setup
# --------------------------------------------------------------------------- #
def setup(document: Document):
    section = document.sections[0]
    section.page_width = Cm(21.0)
    section.page_height = Cm(29.7)
    section.top_margin = Cm(2.0)
    section.bottom_margin = Cm(2.0)
    section.left_margin = Cm(2.5)
    section.right_margin = Cm(1.5)
    section.different_first_page_header_footer = True

    normal = document.styles["Normal"]
    normal.font.name = BODY_FONT
    normal.font.size = Pt(BODY_SIZE)
    rpr = normal.element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    for attr in ("w:ascii", "w:hAnsi", "w:cs", "w:eastAsia"):
        rfonts.set(qn(attr), BODY_FONT)
    pf = normal.paragraph_format
    pf.line_spacing = 1.5
    pf.space_before = Pt(0)
    pf.space_after = Pt(0)
    pf.first_line_indent = Cm(1.25)
    pf.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    pf.widow_control = True

    for name, size, before, after in (("Heading 1", 16, 0, 18),
                                      ("Heading 2", 14, 18, 8),
                                      ("Heading 3", 14, 12, 6)):
        style = document.styles[name]
        style.font.name = BODY_FONT
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.italic = name == "Heading 3"
        style.font.color.rgb = RGBColor(0, 0, 0)
        s_rpr = style.element.get_or_add_rPr()
        s_rfonts = s_rpr.find(qn("w:rFonts"))
        if s_rfonts is None:
            s_rfonts = OxmlElement("w:rFonts")
            s_rpr.append(s_rfonts)
        for attr in ("w:ascii", "w:hAnsi", "w:cs", "w:eastAsia"):
            s_rfonts.set(qn(attr), BODY_FONT)
        s_pf = style.paragraph_format
        s_pf.first_line_indent = Cm(0)
        s_pf.space_before = Pt(before)
        s_pf.space_after = Pt(after)
        s_pf.line_spacing = 1.5
        s_pf.keep_with_next = True
        s_pf.alignment = WD_ALIGN_PARAGRAPH.LEFT

    # page number in the footer (the cover uses "different first page")
    footer = section.footer
    paragraph = footer.paragraphs[0]
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.first_line_indent = Cm(0)
    add_field(paragraph, " PAGE ", "2", size=12)

    # update fields (TOC, page numbers) when the file is opened
    settings = document.settings.element
    update = OxmlElement("w:updateFields")
    update.set(qn("w:val"), "true")
    settings.append(update)


def body(document: Document) -> object:
    return document.styles["Normal"]


def add_paragraph(document: Document, align=None, indent=True,
                  space_after=0, size=BODY_SIZE, keep=False):
    paragraph = document.add_paragraph()
    pf = paragraph.paragraph_format
    pf.first_line_indent = Cm(1.25) if indent else Cm(0)
    pf.space_after = Pt(space_after)
    pf.line_spacing = 1.5
    pf.keep_with_next = keep
    if align is not None:
        pf.alignment = align
    return paragraph


def add_heading(document: Document, level: int, text: str, page_break: bool):
    if page_break:
        document.add_paragraph().add_run().add_break(WD_BREAK.PAGE)
    paragraph = document.add_paragraph(style=f"Heading {level}")
    run = paragraph.add_run(text)
    set_run_font(run, size={1: 16, 2: 14, 3: 14}[level],
                 bold=True, italic=(level == 3),
                 color=RGBColor(0, 0, 0))
    return paragraph


def add_caption(document: Document, text: str, above: bool = False):
    paragraph = add_paragraph(document, align=WD_ALIGN_PARAGRAPH.CENTER,
                              indent=False,
                              space_after=12 if not above else 6)
    paragraph.paragraph_format.space_before = Pt(6)
    paragraph.paragraph_format.keep_with_next = above
    paragraph.paragraph_format.line_spacing = 1.0
    run = paragraph.add_run(text)
    set_run_font(run, size=CAPTION_SIZE)
    return paragraph


# --------------------------------------------------------------------------- #
# inline formatting
# --------------------------------------------------------------------------- #
INLINE = re.compile(
    r"(\*\*[^*]+\*\*|\*[^*]+\*|`[^`]+`|\[\[FIG:[a-zA-Z0-9_]+\]\]"
    r"|\[\[TAB:[a-zA-Z0-9_]+\]\]|\[\[PROTOCOL_INFO\]\])")


def add_inline(paragraph, text: str, size: int = BODY_SIZE,
               base_bold: bool = False, base_italic: bool = False):
    for chunk in INLINE.split(text):
        if not chunk:
            continue
        if chunk.startswith("**") and chunk.endswith("**") and len(chunk) > 4:
            run = paragraph.add_run(chunk[2:-2])
            set_run_font(run, size=size, bold=True, italic=base_italic)
        elif chunk.startswith("*") and chunk.endswith("*") and len(chunk) > 2:
            run = paragraph.add_run(chunk[1:-1])
            set_run_font(run, size=size, bold=base_bold, italic=True)
        elif chunk.startswith("`") and chunk.endswith("`") and len(chunk) > 2:
            run = paragraph.add_run(chunk[1:-1])
            set_run_font(run, name=CODE_FONT, size=size - 2,
                         bold=base_bold, italic=False)
        elif chunk.startswith("[[FIG:"):
            name = chunk[6:-2]
            run = paragraph.add_run(f"рисунок {FIGURES.get(name, 0)}")
            set_run_font(run, size=size, bold=base_bold, italic=True)
        elif chunk.startswith("[[TAB:"):
            name = chunk[6:-2]
            run = paragraph.add_run(f"таблица {TABLES.get(name, 0)}")
            set_run_font(run, size=size, bold=base_bold, italic=True)
        elif chunk.startswith("[[PROTOCOL_INFO]]"):
            run = paragraph.add_run(PROTOCOL_INFO)
            set_run_font(run, size=size, bold=base_bold, italic=base_italic)
        else:
            run = paragraph.add_run(chunk)
            set_run_font(run, size=size, bold=base_bold, italic=base_italic)


# --------------------------------------------------------------------------- #
# tables
# --------------------------------------------------------------------------- #
def auto_widths(header: list[str], rows: list[list[str]],
                available: float = 16.9) -> list[float]:
    """Split the text width by content length, keeping a readable minimum."""
    weights = []
    for index, title in enumerate(header):
        longest = len(title)
        for values in rows:
            if index < len(values):
                longest = max(longest, len(str(values[index])))
        weights.append(min(max(longest, 6), 70))
    total = sum(weights) or 1
    minimum = 1.5
    remainder = available - minimum * len(header)
    return [minimum + remainder * weight / total for weight in weights]


def build_table(document: Document, header: list[str], rows: list[list[str]],
                widths: list[float] | None = None, caption: str | None = None,
                name: str | None = None):
    global table_counter
    table_counter += 1
    if name:
        TABLES[name] = table_counter
    if caption:
        add_caption(document, f"Таблица {table_counter} — {caption}", above=True)

    table = document.add_table(rows=1, cols=len(header))
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.CENTER

    head = table.rows[0]
    repeat_header(head)
    no_split(head)
    for index, text in enumerate(header):
        cell = head.cells[index]
        paragraph = cell.paragraphs[0]
        paragraph.paragraph_format.first_line_indent = Cm(0)
        paragraph.paragraph_format.line_spacing = 1.0
        paragraph.paragraph_format.space_after = Pt(2)
        paragraph.paragraph_format.space_before = Pt(2)
        add_inline(paragraph, text, size=TABLE_SIZE, base_bold=True)
        shade_cell(cell, "EDEDED")

    for values in rows:
        row = table.add_row()
        no_split(row)
        for index, text in enumerate(values):
            cell = row.cells[index]
            paragraph = cell.paragraphs[0]
            paragraph.paragraph_format.first_line_indent = Cm(0)
            paragraph.paragraph_format.line_spacing = 1.0
            paragraph.paragraph_format.space_after = Pt(2)
            paragraph.paragraph_format.space_before = Pt(2)
            add_inline(paragraph, str(text), size=TABLE_SIZE)

    if not widths:
        widths = auto_widths(header, rows)
    table.autofit = False
    for row in table.rows:
        for index, width in enumerate(widths):
            if index < len(row.cells):
                row.cells[index].width = Cm(width)
    add_paragraph(document, indent=False, space_after=6).paragraph_format.line_spacing = 0.6
    return table


def parse_pipe_table(lines: list[str]) -> tuple[list[str], list[list[str]]]:
    parsed = []
    for line in lines:
        text = line.strip()
        if text.startswith("|"):
            text = text[1:]
        if text.endswith("|"):
            text = text[:-1]
        cells = [cell.strip() for cell in text.split("|")]
        if all(re.fullmatch(r":?-{2,}:?", cell) for cell in cells):
            continue
        parsed.append(cells)
    header = parsed[0]
    rows = []
    for cells in parsed[1:]:
        if len(cells) < len(header):
            cells = cells + [""] * (len(header) - len(cells))
        rows.append(cells[:len(header)])
    return header, rows


# --------------------------------------------------------------------------- #
# figures
# --------------------------------------------------------------------------- #
def add_figure(document: Document, caption: str, path: Path,
               width_cm: float = 16.0):
    global figure_counter
    figure_counter += 1
    name = path.stem
    FIGURES[name] = figure_counter
    if path.exists():
        paragraph = document.add_paragraph()
        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
        paragraph.paragraph_format.first_line_indent = Cm(0)
        paragraph.paragraph_format.space_before = Pt(6)
        paragraph.paragraph_format.space_after = Pt(0)
        paragraph.paragraph_format.keep_with_next = True
        run = paragraph.add_run()
        run.add_picture(str(path), width=Cm(min(width_cm, 16.8)))
    else:
        print("missing figure:", path)
        add_paragraph(document).add_run(f"[нет файла {path.name}]")
    add_caption(document, f"Рисунок {figure_counter} — {caption}")


# --------------------------------------------------------------------------- #
# protocol data
# --------------------------------------------------------------------------- #
def latest_bench() -> tuple[Path, dict]:
    runs = ROOT / "runs" / "tests"
    dirs = sorted((p for p in runs.glob("bench_*") if p.is_dir()),
                  key=lambda p: p.name)
    if not dirs:
        raise SystemExit("нет прогонов стенда в runs/tests")
    bench = dirs[-1]
    protocol = bench / "protocol.json"
    if not protocol.exists():
        raise SystemExit(f"нет protocol.json в {bench}")
    return bench, json.loads(protocol.read_text(encoding="utf-8"))


BENCH_DIR, PROTOCOL = latest_bench()
PROTOCOL_INFO = ""


def protocol_info(bench: Path, protocol: dict) -> str:
    stamp = protocol.get("generated_at", "")
    try:
        when = datetime.fromisoformat(stamp.replace("Z", "+00:00"))
        date = when.strftime("%d.%m.%Y")
    except ValueError:
        date = ""
    return f"{date}, прогон {bench.name}".strip(", ")


def metric_value(keys: list[str]) -> str:
    metrics = PROTOCOL.get("metrics", {})
    for key in keys:
        if key in metrics:
            return str(metrics[key])
    for name, value in metrics.items():
        low = name.lower()
        if all(part.lower() in low for part in keys):
            return str(value)
    return "—"


def c1c8_rows() -> list[list[str]]:
    rows = []
    for result in PROTOCOL["results"]:
        rows.append([
            result["id"],
            result.get("title", ""),
            result.get("expected", ""),
            result.get("received", ""),
            f"{float(result.get('duration_s', 0.0)):.1f}",
            "PASS" if result.get("passed") else "FAIL",
        ])
    return rows


def insert_c1c8_table(document: Document):
    caption = (f"Протокол испытаний C1–C8, прогон {BENCH_DIR.name}: "
               f"ожидалось, получено и вердикт")
    build_table(document,
                ["№", "Сценарий", "Ожидалось", "Получено", "t, с", "Вердикт"],
                c1c8_rows(),
                widths=[1.1, 3.2, 4.2, 6.0, 1.2, 1.3],
                caption=caption, name="c1c8")


def insert_metrics_table(document: Document):
    metrics = PROTOCOL.get("metrics", {})
    rows = [[name, str(value)] for name, value in metrics.items()]
    build_table(document, ["Показатель", "Значение"], rows,
                widths=[12.0, 4.0],
                caption=f"Показатели прогона {BENCH_DIR.name}",
                name="metrics")


def relative_proof(value: str) -> str:
    text = str(value).replace("/", "\\")
    prefix = str(BENCH_DIR).replace("/", "\\") + "\\"
    if text.lower().startswith(prefix.lower()):
        text = text[len(prefix):]
    return text.replace("\\", "/")


def insert_details(document: Document):
    for result in PROTOCOL["results"]:
        verdict = "PASS" if result.get("passed") else "FAIL"
        paragraph = add_paragraph(document, indent=False, space_after=4)
        paragraph.paragraph_format.space_before = Pt(12)
        paragraph.paragraph_format.keep_with_next = True
        add_inline(paragraph,
                   f"**{result['id']}. {result.get('title', '')}** — "
                   f"{verdict}, {float(result.get('duration_s', 0.0)):.1f} с",
                   size=13)
        for item in result.get("details", []):
            bullet = add_paragraph(document, indent=False, space_after=2)
            bullet.paragraph_format.left_indent = Cm(1.0)
            bullet.paragraph_format.first_line_indent = Cm(-0.4)
            bullet.paragraph_format.line_spacing = 1.15
            run = bullet.add_run("— ")
            set_run_font(run, size=12)
            add_inline(bullet, item, size=12)
        proof = result.get("proof") or {}
        if proof:
            paths = [f"{key}: {relative_proof(value)}"
                     for key, value in proof.items()]
            path_paragraph = add_paragraph(document, indent=False, space_after=2)
            path_paragraph.paragraph_format.left_indent = Cm(1.0)
            path_paragraph.paragraph_format.first_line_indent = Cm(-0.4)
            path_paragraph.paragraph_format.line_spacing = 1.0
            run = path_paragraph.add_run("Доказательства: " + "  ·  ".join(paths))
            set_run_font(run, name=CODE_FONT, size=9)
        expected = result.get("expected")
        if expected:
            note = add_paragraph(document, indent=False, space_after=6)
            note.paragraph_format.left_indent = Cm(1.0)
            note.paragraph_format.line_spacing = 1.15
            run = note.add_run("Требование: ")
            set_run_font(run, size=12, italic=True)
            add_inline(note, expected, size=12, base_italic=True)


# --------------------------------------------------------------------------- #
# main parser
# --------------------------------------------------------------------------- #
def add_code(document: Document, lines: list[str]):
    paragraph = document.add_paragraph()
    paragraph.paragraph_format.first_line_indent = Cm(0)
    paragraph.paragraph_format.left_indent = Cm(0.6)
    paragraph.paragraph_format.line_spacing = 1.0
    paragraph.paragraph_format.space_before = Pt(6)
    paragraph.paragraph_format.space_after = Pt(6)
    shade(paragraph, "F4F4F4")
    for index, line in enumerate(lines):
        run = paragraph.add_run(line)
        set_run_font(run, name=CODE_FONT, size=11)
        if index < len(lines) - 1:
            run.add_break()


def add_cover(document: Document, lines: list[str]):
    for line in lines:
        text = line.rstrip()
        if text.startswith("@SPACE"):
            paragraph = document.add_paragraph()
            paragraph.paragraph_format.space_after = Pt(0)
            continue
        paragraph = document.add_paragraph()
        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
        paragraph.paragraph_format.first_line_indent = Cm(0)
        paragraph.paragraph_format.line_spacing = 1.3
        if text.startswith("@BIG:"):
            run = paragraph.add_run(text[5:].strip())
            set_run_font(run, size=30, bold=True)
            paragraph.paragraph_format.space_before = Pt(40)
            paragraph.paragraph_format.space_after = Pt(16)
        elif text.startswith("@MID:"):
            run = paragraph.add_run(text[5:].strip())
            set_run_font(run, size=16)
            paragraph.paragraph_format.space_after = Pt(10)
        else:
            value = text[7:].strip() if text.startswith("@SMALL:") else text
            run = paragraph.add_run(value)
            set_run_font(run, size=13)
            paragraph.paragraph_format.space_after = Pt(6)


def add_toc(document: Document):
    paragraph = add_paragraph(document, align=WD_ALIGN_PARAGRAPH.CENTER,
                              indent=False, space_after=12)
    run = paragraph.add_run("Содержание")
    set_run_font(run, size=16, bold=True, color=RGBColor(0, 0, 0))

    toc = document.add_paragraph()
    toc.paragraph_format.first_line_indent = Cm(0)
    add_field(toc, ' TOC \\o "1-3" \\h \\z \\u ',
              "Оглавление обновится при открытии документа "
              "(в Word: Ctrl+A, затем F9).", size=12)


def build():
    global PROTOCOL_INFO
    PROTOCOL_INFO = protocol_info(BENCH_DIR, PROTOCOL)

    document = Document()
    setup(document)
    document.core_properties.title = "Техническая записка АТ11-2"
    document.core_properties.author = "Агрокидсы"
    document.core_properties.subject = ("Разведка → карта → адресная операция")

    lines = SOURCE.read_text(encoding="utf-8").splitlines()
    index = 0
    paragraph_lines: list[str] = []
    pending_caption: str | None = None
    body_started = False
    last_was_break = False
    in_cover = False
    cover_lines: list[str] = []

    def flush_paragraph():
        nonlocal paragraph_lines
        if not paragraph_lines:
            return
        text = " ".join(part.strip() for part in paragraph_lines)
        paragraph_lines = []
        paragraph = add_paragraph(document)
        add_inline(paragraph, text)

    while index < len(lines):
        line = lines[index].rstrip()

        if in_cover:
            if line.strip() == "[/COVER]":
                add_cover(document, cover_lines)
                in_cover = False
                cover_lines = []
            else:
                cover_lines.append(line)
            index += 1
            continue

        if line.startswith("```"):
            flush_paragraph()
            block = []
            index += 1
            while index < len(lines) and not lines[index].startswith("```"):
                block.append(lines[index].rstrip())
                index += 1
            add_code(document, block)
            index += 1
            body_started = True
            continue

        if line.startswith("|"):
            flush_paragraph()
            block = []
            while index < len(lines) and lines[index].strip().startswith("|"):
                block.append(lines[index])
                index += 1
            header, rows = parse_pipe_table(block)
            build_table(document, header, rows, caption=pending_caption)
            pending_caption = None
            body_started = True
            continue

        stripped = line.strip()

        if stripped == "[COVER]":
            in_cover = True
            index += 1
            continue

        if stripped == "[PAGEBREAK]":
            flush_paragraph()
            document.add_paragraph().add_run().add_break(WD_BREAK.PAGE)
            last_was_break = True
            index += 1
            continue

        if stripped == "[TOC]":
            flush_paragraph()
            add_toc(document)
            last_was_break = False
            body_started = True
            index += 1
            continue

        if stripped.startswith("[[C1C8_TABLE]]"):
            flush_paragraph()
            insert_c1c8_table(document)
            body_started = True
            index += 1
            continue

        if stripped.startswith("[[METRICS_TABLE]]"):
            flush_paragraph()
            insert_metrics_table(document)
            body_started = True
            index += 1
            continue

        if stripped.startswith("[[C1C8_DETAILS]]"):
            flush_paragraph()
            insert_details(document)
            body_started = True
            index += 1
            continue

        if stripped.startswith("TABLE:"):
            flush_paragraph()
            pending_caption = stripped[6:].strip()
            index += 1
            continue

        if stripped.startswith("!["):
            flush_paragraph()
            match = re.match(r"!\[(.*?)\]\((.*?)\)", stripped)
            if match:
                caption = match.group(1)
                target = match.group(2)
                width = 16.0
                if "," in target:
                    target, raw = target.rsplit(",", 1)
                    try:
                        width = float(raw.strip())
                    except ValueError:
                        width = 16.0
                path = (SOURCE.parent / target.strip()).resolve()
                add_figure(document, caption, path, width)
            body_started = True
            index += 1
            continue

        if stripped.startswith("### "):
            flush_paragraph()
            add_heading(document, 3, stripped[4:].strip(), page_break=False)
            last_was_break = False
            body_started = True
            index += 1
            continue

        if stripped.startswith("## "):
            flush_paragraph()
            add_heading(document, 2, stripped[3:].strip(), page_break=False)
            last_was_break = False
            body_started = True
            index += 1
            continue

        if stripped.startswith("# "):
            flush_paragraph()
            add_heading(document, 1, stripped[2:].strip(),
                        page_break=body_started and not last_was_break)
            last_was_break = False
            body_started = True
            index += 1
            continue

        if stripped.startswith(("- ", "* ")) or re.match(r"^\d+\.\s", stripped):
            flush_paragraph()
            collected = [stripped]
            index += 1
            while index < len(lines):
                following = lines[index].strip()
                if not following:
                    break
                if following.startswith(("#", "|", "!", "```", "[[", "[")):
                    break
                if following.startswith("TABLE:"):
                    break
                if (following.startswith(("- ", "* "))
                        or re.match(r"^\d+\.\s", following)):
                    break
                collected.append(following)
                index += 1
            text = " ".join(collected)
            paragraph = add_paragraph(document, space_after=2)
            if text.startswith(("- ", "* ")):
                paragraph.paragraph_format.left_indent = Cm(1.25)
                paragraph.paragraph_format.first_line_indent = Cm(-0.5)
                run = paragraph.add_run("•\t")
                set_run_font(run, size=BODY_SIZE)
                add_inline(paragraph, text[2:].strip())
            else:
                paragraph.paragraph_format.left_indent = Cm(1.25)
                paragraph.paragraph_format.first_line_indent = Cm(-0.75)
                add_inline(paragraph, text)
            continue

        if stripped == "":
            flush_paragraph()
            index += 1
            continue

        paragraph_lines.append(stripped)
        index += 1

    flush_paragraph()

    document.save(OUTPUT)
    print("saved:", OUTPUT)
    print("figures:", figure_counter, "tables:", table_counter)


if __name__ == "__main__":
    build()
