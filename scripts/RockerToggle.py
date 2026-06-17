"""Pill-shaped two-segment rocker toggle widget for PyQt6."""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QPushButton, QWidget

import Parameter as params

_ACCT = params.UI_ACCENT_COLOR
_CARD = params.UI_CARD_COLOR
_BG   = params.UI_BG_COLOR
_DIM  = params.UI_DIM_COLOR
_SEP  = params.UI_SEP_COLOR


class RockerToggle(QWidget):
    """Pill-shaped two-segment rocker switch (e.g. OD / OS).

    The active segment is filled with the accent colour; the inactive one
    shows the card background with a dim border so it reads as recessed.
    Calls ``on_change(value)`` whenever the selection changes.
    """

    def __init__(
        self,
        opt_a: str,
        opt_b: str,
        initial: str = '',
        on_change=None,
        parent: QWidget = None,
    ) -> None:
        super().__init__(parent)
        self._value     = initial or opt_a
        self._on_change = on_change

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._btn_a = QPushButton(opt_a)
        self._btn_b = QPushButton(opt_b)

        for btn in (self._btn_a, self._btn_b):
            btn.setFixedHeight(22)
            btn.setMinimumWidth(36)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self._btn_a.clicked.connect(lambda: self._select(opt_a))
        self._btn_b.clicked.connect(lambda: self._select(opt_b))

        layout.addWidget(self._btn_a)
        layout.addWidget(self._btn_b)
        self._refresh()

    def value(self) -> str:
        return self._value

    def _select(self, val: str) -> None:
        if self._value != val:
            self._value = val
            self._refresh()
            if self._on_change:
                self._on_change(val)

    def _refresh(self) -> None:
        a_active = self._value == self._btn_a.text()
        self._btn_a.setStyleSheet(self._seg_qss(active=a_active,     left_cap=True))
        self._btn_b.setStyleSheet(self._seg_qss(active=not a_active, left_cap=False))

    @staticmethod
    def _seg_qss(active: bool, left_cap: bool) -> str:
        r_tl = "7px" if left_cap else "0px"
        r_tr = "0px" if left_cap else "7px"
        r_bl = "7px" if left_cap else "0px"
        r_br = "0px" if left_cap else "7px"
        bg   = _ACCT         if active else _CARD
        fg   = _BG           if active else _DIM
        bdr  = "transparent" if active else _SEP
        fw   = "bold"        if active else "normal"
        fs   = params.UI_FONT_SIZE - 1
        return (
            f"QPushButton {{"
            f" background: {bg};"
            f" color: {fg};"
            f" border: 1px solid {bdr};"
            f" border-top-left-radius: {r_tl};"
            f" border-top-right-radius: {r_tr};"
            f" border-bottom-left-radius: {r_bl};"
            f" border-bottom-right-radius: {r_br};"
            f" padding: 0px 10px;"
            f" font-size: {fs}pt;"
            f" font-weight: {fw};"
            f"}}"
        )
