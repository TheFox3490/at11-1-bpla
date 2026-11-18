"""Base class for the three application modes."""
from __future__ import annotations


class Mode:
    name = ""
    title = ""
    role = "neutral"

    def __init__(self, app):
        self.app = app
        self.active = False
        self.ui_entities: list = []

    def add_ui(self, entity):
        entity.enabled = self.active
        self.ui_entities.append(entity)
        return entity

    def set_ui_visible(self, visible: bool) -> None:
        for entity in self.ui_entities:
            entity.enabled = visible

    def enter(self) -> None:
        self.active = True
        self.set_ui_visible(True)
        self.on_enter()
        self.refresh()

    def exit(self) -> None:
        self.on_exit()
        self.set_ui_visible(False)
        self.active = False

    # Override in modes -------------------------------------------------- #
    def on_enter(self) -> None:
        pass

    def on_exit(self) -> None:
        pass

    def step(self, dt: float) -> None:
        pass

    def input(self, key: str) -> None:
        pass

    def refresh(self) -> None:
        pass

    def on_execution_done(self, run) -> None:
        pass
