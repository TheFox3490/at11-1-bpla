"""Append-only JSONL journal and a durable action registry.

The registry is what makes C5 (repeat assignment / restart) meaningful: a
target that was confirmed in a previous process must not be actioned again.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from shared.contracts import atomic_write_text, read_json, utc_now, write_json


class Journal:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "a", encoding="utf-8")

    def log(self, event: str, **fields: Any) -> dict:
        record = {"t_wall": utc_now().isoformat(timespec="milliseconds").replace("+00:00", "Z"),
                  "event": event}
        record.update(fields)
        self._fh.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._fh.flush()
        os.fsync(self._fh.fileno())
        return record

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class ActionRegistry:
    """Durable map action_id -> status.  Saved atomically after every change."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.data: dict[str, dict] = {}
        if self.path.exists():
            try:
                self.data = read_json(self.path)
            except (json.JSONDecodeError, OSError):
                self.data = {}

    def get(self, action_id: str) -> dict | None:
        return self.data.get(action_id)

    def is_done(self, action_id: str) -> bool:
        return self.data.get(action_id, {}).get("status") in {"confirmed", "uncertain"}

    def mark(self, action_id: str, status: str, **fields: Any) -> None:
        entry = {"status": status, "updated_at": utc_now().isoformat()}
        entry.update(fields)
        self.data[action_id] = entry
        self.save()

    def save(self) -> None:
        atomic_write_text(self.path,
                          json.dumps(self.data, indent=2, ensure_ascii=False))
