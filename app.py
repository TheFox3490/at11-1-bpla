"""Entry point: run the three-mode BPLA application.

    .venv\\Scripts\\python.exe app.py

Ursina looks for ``update`` and ``input`` in the main module, so this file
forwards both to the application object in ``sim.app``.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import sim.app as application  # noqa: E402


def update() -> None:
    application.update()


def input(key: str) -> None:  # noqa: A001 - required name
    application.input(key)


if __name__ == "__main__":
    application.main()
