"""PyQt6 control window for patient motion interface.

Provides a dark-themed GUI with:
  - Gimbal speed tuning and WASD key-binding reference
  - Eye motion profile playback (Bell's reflex, saccadic) with auto-rewind
  - Head motion playback from IMU CSV files
  - TRF Cartesian jog buttons (X/Y/Z translation and UX/UY/UZ rotation)
"""

from __future__ import annotations

import csv
import glob
import math
import os
import threading
import time
from typing import Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QDoubleSpinBox, QFileDialog, QFrame, QHBoxLayout,
    QLabel, QMainWindow, QProgressBar, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)

import Parameter as params
from Utils import (
    clamp, disable_motor, enable_echo, write_position,
    lighten_color as _lighten, btn_qss as _btn_qss, global_qss as _global_qss,
)

# ─────────────────────────────────────────────
#  Colour aliases (sourced from Parameter.py)
# ─────────────────────────────────────────────

_BG         = params.UI_BG_COLOR
_CARD       = params.UI_CARD_COLOR
_ACCT       = params.UI_ACCENT_COLOR
_FG         = params.UI_FG_COLOR
_DIM        = params.UI_DIM_COLOR
_SEP        = params.UI_SEP_COLOR
_BTN_PURPLE = params.UI_BTN_PURPLE
_BTN_GREEN  = params.UI_BTN_GREEN
_BTN_RED    = params.UI_BTN_RED
_BTN_GRAY   = params.UI_BTN_GRAY
_GREEN      = params.UI_BTN_GREEN
_RED        = params.UI_BTN_RED


class ControlWindow(QMainWindow):
    """Main control window using PyQt6.

    Args:
        use_head: Whether the Meca500 head robot is active.
        use_eye: Whether the Dynamixel eye gimbal is active.
        robot: mecademicpy Robot instance, or None.
        port_handler: Dynamixel PortHandler, or None.
        packet_handler: Dynamixel PacketHandler, or None.
        listener: pynput Listener (stopped on close).
    """

    _AXIS_IDX: dict[str, int] = {'x': 0, 'y': 1, 'z': 2, 'ux': 3, 'uy': 4, 'uz': 5}

    def __init__(
        self,
        use_head: bool,
        use_eye: bool,
        robot,
        port_handler,
        packet_handler,
        listener,
    ) -> None:
        super().__init__()
        self.use_head       = use_head
        self.use_eye        = use_eye
        self.robot          = robot
        self.port_handler   = port_handler
        self.packet_handler = packet_handler
        self.listener       = listener

        # Current gimbal encoder positions
        self.pos1 = params.EYE_CENTER
        self.pos2 = params.EYE_CENTER

        # Eye motion profile playback state
        self._eye_profile_data: list[tuple[float, int, int]] = []
        self._eye_play_idx     = 0
        self._eye_play_start   = 0.0
        self._eye_playing      = False
        self._eye_rewinding    = False
        self._eye_rewind_steps: list[tuple[int, int]] = []
        self._eye_rewind_idx   = 0

        # Eye profile UI widgets (populated in _build_eye_profiles_card)
        self._bells_btn:        Optional[QPushButton]  = None
        self._saccadic_btn:     Optional[QPushButton]  = None
        self._eye_progress_bar: Optional[QProgressBar] = None
        self._eye_progress_lbl: Optional[QLabel]       = None

        # Head motion preset UI widgets
        self._preset_btns:      list[QPushButton] = []   # legacy — kept for _load_file enable
        self._breath_start_btn: Optional[QPushButton] = None
        self._stop_breath_btn:  Optional[QPushButton] = None
        self._interrupt_btns:   list[QPushButton] = []

        # Breathing loop state
        self._breathing:           bool  = False
        self._resume_breath:       bool  = False
        self._breath_resume_idx:   int   = 0
        self._breath_resume_pitch: float = 0.0
        self._breath_resume_roll:  float = 0.0
        self._breath_resume_t:     float = 0.0

        # Gimbal reset state
        self._gimbal_resetting = False

        # Head motion playback state
        self._head_playback_data: list[tuple[float, float, float]] = []
        self._head_play_idx  = 0
        self.is_playing      = False
        self.playback_start  = 0.0
        self.prev_pitch      = 0.0
        self.prev_roll       = 0.0
        self._head_rewinding      = False
        self._head_return_done    = False

        # Head playback UI widgets (populated in _build_motion_playback_card)
        self.file_label:      Optional[QLabel]       = None
        self.info_label:      Optional[QLabel]       = None
        self.play_btn:        Optional[QPushButton]  = None
        self.stop_btn:        Optional[QPushButton]  = None
        self.progress_bar:    Optional[QProgressBar] = None
        self.progress_label:  Optional[QLabel]       = None

        # Gimbal speed UI
        self._gimbal_speed_slider: Optional[QSlider] = None
        self._vel_label:           Optional[QLabel]  = None

        # TRF Cartesian jog state
        self._trf_lin_step:    Optional[QDoubleSpinBox] = None
        self._trf_ang_step:    Optional[QDoubleSpinBox] = None
        self._trf_jog_axis     = 'x'
        self._trf_jog_sign     = 1.0
        self._cart_pos         = [0.0] * 6
        self._home_cart_offset = [0.0] * 6
        self._head_home_joints: list[float] = list(params.ROBOT_HEAD_INIT_POSE)
        self._pose_label:      Optional[QLabel] = None

        self._trf_jog_timer = QTimer(self)
        self._trf_jog_timer.timeout.connect(self._repeat_trf_jog)

        self._build_ui()

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000 / params.LOOP_HZ))
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    # ─────────────────────────────────────────
    #  Widget factory helpers
    # ─────────────────────────────────────────

    def _make_card(self, parent: QWidget) -> tuple[QFrame, QVBoxLayout]:
        """Create a styled card QFrame, add it to parent's layout, return (frame, layout)."""
        card = QFrame()
        card.setObjectName("card")
        inner = QVBoxLayout(card)
        inner.setContentsMargins(14, 10, 14, 10)
        inner.setSpacing(4)
        parent.layout().addWidget(card)
        return card, inner

    def _lbl(
        self,
        text: str,
        size: Optional[int] = None,
        bold: bool = False,
        color: Optional[str] = None,
    ) -> QLabel:
        """Create (but do not add) a styled QLabel."""
        fs = size if size is not None else params.UI_FONT_SIZE
        lbl = QLabel(text)
        weight = "bold" if bold else "normal"
        c = color or _FG
        lbl.setStyleSheet(
            f"color: {c}; font-size: {fs}pt; font-weight: {weight}; background: transparent;"
        )
        return lbl

    def _sep(self, layout: QVBoxLayout) -> None:
        """Add a small vertical gap between card sections."""
        spacer = QWidget()
        spacer.setFixedHeight(6)
        layout.addWidget(spacer)

    def _btn(
        self,
        text: str,
        command,
        color: str = _BTN_PURPLE,
        min_width: int = 80,
    ) -> QPushButton:
        """Create (but do not add) a styled QPushButton."""
        btn = QPushButton(text)
        btn.setStyleSheet(_btn_qss(color))
        btn.setMinimumWidth(min_width)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        btn.clicked.connect(lambda _=False: command())
        return btn

    def _jog_btn(self, label: str, axis: str, sign: float) -> QPushButton:
        """Create a hold-to-jog TRF button that sends moves while pressed."""
        btn = QPushButton(label)
        btn.setStyleSheet(_btn_qss(_BTN_GRAY))
        btn.setMinimumWidth(90)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        btn.pressed.connect(lambda: self._start_trf_jog(axis, sign))
        btn.released.connect(self._stop_trf_jog)
        return btn

    # ─────────────────────────────────────────
    #  UI construction
    # ─────────────────────────────────────────

    def _build_ui(self) -> None:
        self.setWindowTitle("Patient Motion Control")
        self.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint)
        self.setStyleSheet(_global_qss())

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 8)
        root_layout.setSpacing(0)

        # Header banner
        header = QFrame()
        header.setStyleSheet(f"background-color: {_ACCT}; border: none;")
        h_layout = QVBoxLayout(header)
        h_layout.setContentsMargins(12, 12, 12, 12)
        h_layout.setSpacing(2)

        parts = []
        if self.use_head:
            parts.append("Head  (Meca500)")
        if self.use_eye:
            parts.append("Eye  (Dynamixel)")
        sub = QLabel("  +  ".join(parts))
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setStyleSheet("color: #ddd6fe; font-size: 12pt; background: transparent;")
        h_layout.addWidget(sub)
        root_layout.addWidget(header)

        # Two-column body
        body = QWidget()
        body_layout = QHBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(0)
        root_layout.addWidget(body)

        left = QWidget()
        left.setFixedWidth(params.UI_LEFT_COL_WIDTH)
        left_layout = QVBoxLayout(left)
        left_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(8)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        right_layout.setContentsMargins(4, 4, 4, 4)
        right_layout.setSpacing(8)

        body_layout.addWidget(left)
        body_layout.addWidget(right, 1)

        if self.use_eye:
            self._build_gimbal_speed_card(left)

        self._build_key_bindings_card(left)

        if self.use_eye:
            self._build_eye_profiles_card(left)

        if self.use_head:
            self._build_motion_playback_card(left)
            self._build_trf_jog_card(right)

        self.adjustSize()
        self.setFixedSize(self.sizeHint())

    def _build_gimbal_speed_card(self, parent: QWidget) -> None:
        card, layout = self._make_card(parent)

        layout.addWidget(self._lbl("Gimbal Speed", params.UI_FONT_SIZE + 1, bold=True))
        self._sep(layout)

        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.addWidget(self._lbl("Step", color=_DIM))
        row_layout.addStretch()
        row_layout.addWidget(self._lbl("counts / tick", color=_DIM))
        layout.addWidget(row)

        slider_row = QWidget()
        sr_layout = QHBoxLayout(slider_row)
        sr_layout.setContentsMargins(0, 0, 0, 0)
        sr_layout.setSpacing(8)

        self._gimbal_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self._gimbal_speed_slider.setMinimum(1)
        self._gimbal_speed_slider.setMaximum(90)
        self._gimbal_speed_slider.setValue(params.STEP_SIZE)
        self._gimbal_speed_slider.setMinimumWidth(200)

        self._slider_value_lbl = QLabel(str(params.STEP_SIZE))
        self._slider_value_lbl.setStyleSheet(f"color: {_FG}; font-size: 9pt; background: transparent; min-width: 24px;")
        self._slider_value_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        sr_layout.addWidget(self._gimbal_speed_slider)
        sr_layout.addWidget(self._slider_value_lbl)
        layout.addWidget(slider_row)

        self._vel_label = self._lbl("", color=_DIM)
        layout.addWidget(self._vel_label)
        self._update_vel()
        self._gimbal_speed_slider.valueChanged.connect(self._on_speed_changed)

        self._sep(layout)
        reset_btn = self._btn("Gimbal Reset", self._reset_gimbal, color=_BTN_GRAY, min_width=110)
        layout.addWidget(reset_btn, alignment=Qt.AlignmentFlag.AlignLeft)

    def _build_key_bindings_card(self, parent: QWidget) -> None:
        card, layout = self._make_card(parent)
        layout.addWidget(self._lbl("Key Bindings", params.UI_FONT_SIZE + 1, bold=True))
        self._sep(layout)

        bindings: list[tuple[str, str, str]] = []
        if self.use_eye:
            bindings += [
                ("W / S",    "Eye  Superior / Inferior", _ACCT),
                ("A / D",    "Eye  Temporal / Nasal",    _ACCT),
            ]
        bindings.append(("Q / ESC", "Quit", _RED))

        for key_text, action, color in bindings:
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.addWidget(self._lbl(key_text, bold=True, color=color))
            row_layout.addStretch()
            row_layout.addWidget(self._lbl(action, color=_FG))
            layout.addWidget(row)

    def _build_eye_profiles_card(self, parent: QWidget) -> None:
        card, layout = self._make_card(parent)
        layout.addWidget(self._lbl("Eye Motion Profiles", params.UI_FONT_SIZE + 1, bold=True))
        self._sep(layout)

        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(6)

        self._bells_btn = self._btn(
            "Bell's Reflex",
            lambda: self._start_eye_profile('bells'),
            color=_BTN_PURPLE,
        )
        self._saccadic_btn = self._btn(
            "Saccadic",
            lambda: self._start_eye_profile('saccadic'),
            color=_BTN_PURPLE,
        )
        self._bells_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._saccadic_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        btn_layout.addWidget(self._bells_btn)
        btn_layout.addWidget(self._saccadic_btn)
        layout.addWidget(btn_row)

        self._eye_progress_bar = QProgressBar()
        self._eye_progress_bar.setMaximum(100)
        self._eye_progress_bar.setValue(0)
        self._eye_progress_bar.setTextVisible(False)
        layout.addWidget(self._eye_progress_bar)

        self._eye_progress_lbl = self._lbl("Ready", color=_DIM)
        layout.addWidget(self._eye_progress_lbl)

    def _build_motion_playback_card(self, parent: QWidget) -> None:
        card, layout = self._make_card(parent)
        layout.addWidget(self._lbl("Motion Playback", params.UI_FONT_SIZE + 1, bold=True))
        self._sep(layout)

        # Breathing loop
        layout.addWidget(self._lbl("Breathing", color=_DIM))

        self._breath_start_btn = self._btn("▶  Breathe", self._start_breathing, color=_BTN_GREEN)
        self._breath_start_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout.addWidget(self._breath_start_btn)

        self._stop_breath_btn = self._btn("■  Stop Breathing", self._stop_breathing, color=_BTN_RED)
        self._stop_breath_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._stop_breath_btn.setEnabled(False)
        layout.addWidget(self._stop_breath_btn)

        self._sep(layout)

        # Interruptions (available while breathing)
        layout.addWidget(self._lbl("Interruptions", color=_DIM))
        for label, keyword in [
            ("Cough",        params.HEAD_COUGH_PROFILE),
            ("Clear Throat", params.HEAD_CLEAR_THROAT_PROFILE),
        ]:
            btn = self._btn(label, lambda kw=keyword: self._play_preset(kw), color=_BTN_PURPLE)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            layout.addWidget(btn)
            self._interrupt_btns.append(btn)

        self._sep(layout)

        # File picker
        file_row = QWidget()
        fl_layout = QHBoxLayout(file_row)
        fl_layout.setContentsMargins(0, 0, 0, 0)
        self.file_label = self._lbl("No file loaded", color=_DIM)
        fl_layout.addWidget(self.file_label, 1)
        browse_btn = self._btn("Browse…", self._browse_file, min_width=70)
        fl_layout.addWidget(browse_btn)
        layout.addWidget(file_row)

        self.info_label = self._lbl("", color=_DIM)
        self.info_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        layout.addWidget(self.info_label)

        # Play / Stop
        ctrl_row = QWidget()
        ctrl_layout = QHBoxLayout(ctrl_row)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)
        ctrl_layout.setSpacing(6)

        self.play_btn = self._btn("▶  Play", self._start_playback, color=_BTN_GREEN, min_width=80)
        self.play_btn.setEnabled(False)
        self.stop_btn = self._btn("■  Stop", self._on_stop_pressed, color=_BTN_RED, min_width=80)
        self.stop_btn.setEnabled(False)
        ctrl_layout.addWidget(self.play_btn)
        ctrl_layout.addWidget(self.stop_btn)
        ctrl_layout.addStretch()
        layout.addWidget(ctrl_row)

        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        layout.addWidget(self.progress_bar)

        self.progress_label = self._lbl("", color=_DIM)
        self.progress_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        layout.addWidget(self.progress_label)

    def _build_trf_jog_card(self, parent: QWidget) -> None:
        card, layout = self._make_card(parent)
        layout.addWidget(self._lbl("TRF Cartesian Jog", params.UI_FONT_SIZE + 1, bold=True))
        self._sep(layout)

        go_init_btn = self._btn("Go Init", self._go_init_pose, color=_BTN_PURPLE)
        go_init_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout.addWidget(go_init_btn)
        self._sep(layout)

        # Step-size spinboxes
        for attr, label_text, unit in [
            ('_trf_lin_step', "Linear step",  "mm"),
            ('_trf_ang_step', "Angular step", "°"),
        ]:
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 2, 0, 2)
            row_layout.addWidget(self._lbl(label_text, color=_DIM))
            row_layout.addStretch()
            row_layout.addWidget(self._lbl(unit, color=_DIM))
            spin = QDoubleSpinBox()
            spin.setRange(0.1, 50.0)
            spin.setSingleStep(0.5)
            spin.setValue(1.0)
            spin.setFixedWidth(65)
            row_layout.addWidget(spin)
            setattr(self, attr, spin)
            layout.addWidget(row)

        # Translation
        layout.addWidget(self._lbl("Translation", bold=True, color=_DIM))
        for axis, neg_lbl, pos_lbl in [
            ('x', 'Nasal',    'Temporal'),
            ('y', 'Down',     'Up'),
            ('z', 'Inferior', 'Superior'),
        ]:
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 3, 0, 3)
            row_layout.addWidget(self._jog_btn(f"← {neg_lbl}", axis, -1))
            row_layout.addStretch()
            axis_lbl = self._lbl(axis.upper(), params.UI_FONT_SIZE + 1, bold=True, color=_ACCT)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            row_layout.addWidget(axis_lbl)
            row_layout.addStretch()
            row_layout.addWidget(self._jog_btn(f"{pos_lbl} →", axis, +1))
            layout.addWidget(row)

        # Rotation
        layout.addWidget(self._lbl("Rotation", bold=True, color=_DIM))
        for axis, neg_lbl, pos_lbl in [
            ('ux', 'Inferior', 'Superior'),
            ('uy', 'Left',     'Right'),
            ('uz', 'Temporal', 'Nasal'),
        ]:
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 3, 0, 3)
            row_layout.addWidget(self._jog_btn(f"← {neg_lbl}", axis, -1))
            row_layout.addStretch()
            axis_lbl = self._lbl(axis, params.UI_FONT_SIZE + 1, bold=True, color=_ACCT)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            row_layout.addWidget(axis_lbl)
            row_layout.addStretch()
            row_layout.addWidget(self._jog_btn(f"{pos_lbl} →", axis, +1))
            layout.addWidget(row)

        self._sep(layout)

        for text, cmd, color in [
            ("Set Home",    self._set_trf_home,      _BTN_PURPLE),
            ("Go Home",     self._go_trf_home,       _BTN_GREEN),
            ("Reset Error", self._reset_robot_error, _BTN_RED),
            ("Get Robot Pose", self._get_robot_pose, _BTN_GRAY),
        ]:
            btn = self._btn(text, cmd, color=color, min_width=90)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            layout.addWidget(btn)

        self._pose_label = self._lbl("", color=_DIM)
        self._pose_label.setWordWrap(True)
        self._pose_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        layout.addWidget(self._pose_label)

    # ─────────────────────────────────────────
    #  Gimbal speed display
    # ─────────────────────────────────────────

    def _on_speed_changed(self, value: int) -> None:
        if self._slider_value_lbl:
            self._slider_value_lbl.setText(str(value))
        self._update_vel()

    def _update_vel(self) -> None:
        if self._vel_label is None or self._gimbal_speed_slider is None:
            return
        step = self._gimbal_speed_slider.value()
        counts_per_sec = step * params.LOOP_HZ
        deg_per_sec    = counts_per_sec * 360 / params.COUNTS_PER_REV
        rpm            = counts_per_sec / params.COUNTS_PER_REV * 60
        self._vel_label.setText(f"{deg_per_sec:.1f} °/s   ·   {rpm:.1f} RPM")

    # ─────────────────────────────────────────
    #  Eye motion profiles
    # ─────────────────────────────────────────

    def _reset_gimbal(self) -> None:
        if not self.port_handler or self._eye_playing or self._eye_rewinding:
            return
        self._gimbal_resetting = True

    def _find_eye_profile(self, keyword: str) -> str:
        folder  = os.path.join(os.path.dirname(__file__), params.EYE_MOTION_PROFILE_FOLDER)
        matches = glob.glob(os.path.join(folder, f'*{keyword}*.csv'))
        if not matches:
            raise FileNotFoundError(
                f"No eye profile CSV matching '*{keyword}*.csv' in {folder}")
        return max(matches)

    def _load_eye_profile(self, keyword: str) -> list[tuple[float, int, int]]:
        """Parse an eye motion CSV and convert to timed motor-count tuples.

        CSV columns: t (s), x (deg), y (deg)
          - positive x = inferior  → DXL_1 position decreases
          - positive y = nasal     → DXL_2 position decreases
        """
        path = self._find_eye_profile(keyword)
        data: list[tuple[float, int, int]] = []
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                t  = float(row['t'])
                m1 = clamp(
                    round(params.EYE_CENTER - float(row['x']) * params.COUNTS_PER_DEG),
                    params.JOINT_MIN_1, params.JOINT_MAX_1,
                )
                m2 = clamp(
                    round(params.EYE_CENTER - float(row['y']) * params.COUNTS_PER_DEG),
                    params.JOINT_MIN_2, params.JOINT_MAX_2,
                )
                data.append((t, m1, m2))
        return data

    def _start_eye_profile(self, keyword: str) -> None:
        if self._eye_playing or self._eye_rewinding or self._gimbal_resetting:
            return
        try:
            data = self._load_eye_profile(keyword)
        except Exception as exc:
            print(f"[eye profile] {exc}")
            return

        write_position(self.port_handler, self.packet_handler, params.DXL_1, params.EYE_CENTER)
        write_position(self.port_handler, self.packet_handler, params.DXL_2, params.EYE_CENTER)
        self.pos1 = params.EYE_CENTER
        self.pos2 = params.EYE_CENTER

        self._eye_profile_data = data
        self._eye_play_idx     = 0
        self._eye_play_start   = time.monotonic()
        self._eye_playing      = True
        self._eye_rewinding    = False

        self._bells_btn.setEnabled(False)
        self._saccadic_btn.setEnabled(False)
        self._eye_progress_bar.setValue(0)
        self._eye_progress_lbl.setText("Playing…")
        self._eye_progress_lbl.setStyleSheet(f"color: {_FG}; background: transparent;")

    def _start_eye_rewind(self) -> None:
        n_steps = max(1, round(1.5 * params.LOOP_HZ))
        self._eye_rewind_steps = [
            (
                round(self.pos1 + (i / n_steps) * (params.EYE_CENTER - self.pos1)),
                round(self.pos2 + (i / n_steps) * (params.EYE_CENTER - self.pos2)),
            )
            for i in range(1, n_steps + 1)
        ]
        self._eye_rewind_idx = 0
        self._eye_rewinding  = True
        self._eye_progress_lbl.setText("Rewinding…")
        self._eye_progress_lbl.setStyleSheet(f"color: {_DIM}; background: transparent;")

    # ─────────────────────────────────────────
    #  UI state helpers
    # ─────────────────────────────────────────

    def _set_idle_ui(self) -> None:
        if self._breath_start_btn: self._breath_start_btn.setEnabled(True)
        if self._stop_breath_btn:  self._stop_breath_btn.setEnabled(False)
        for btn in self._interrupt_btns: btn.setEnabled(True)
        if self.play_btn:  self.play_btn.setEnabled(bool(self._head_playback_data))
        if self.stop_btn:  self.stop_btn.setEnabled(False)

    def _set_busy_ui(self) -> None:
        if self._breath_start_btn: self._breath_start_btn.setEnabled(False)
        if self._stop_breath_btn:  self._stop_breath_btn.setEnabled(False)
        for btn in self._interrupt_btns: btn.setEnabled(False)
        if self.play_btn:  self.play_btn.setEnabled(False)
        if self.stop_btn:  self.stop_btn.setEnabled(True)

    def _set_breathing_ui(self) -> None:
        if self._breath_start_btn: self._breath_start_btn.setEnabled(False)
        if self._stop_breath_btn:  self._stop_breath_btn.setEnabled(True)
        for btn in self._interrupt_btns: btn.setEnabled(True)
        if self.play_btn:  self.play_btn.setEnabled(False)
        if self.stop_btn:  self.stop_btn.setEnabled(True)

    # ─────────────────────────────────────────
    #  Breathing loop
    # ─────────────────────────────────────────

    def _start_breathing(self) -> None:
        """Load the rest profile and loop it continuously."""
        if self._breathing or self.is_playing:
            return
        folder  = os.path.join(os.path.dirname(__file__), params.HEAD_MOTION_PROFILE_FOLDER)
        matches = glob.glob(os.path.join(folder, f'*{params.HEAD_REST_PROFILE}*.txt'))
        matches += glob.glob(os.path.join(folder, f'*{params.HEAD_REST_PROFILE}*.csv'))
        if not matches:
            self.progress_label.setText(f"No '{params.HEAD_REST_PROFILE}' profile found")
            self.progress_label.setStyleSheet(f"color: {_RED}; background: transparent;")
            return
        self._load_file(max(matches))
        if not self._head_playback_data or not self.robot:
            return
        self._breathing     = True
        self._head_play_idx = 0
        self.prev_pitch     = 0.0
        self.prev_roll      = 0.0
        self.playback_start = time.monotonic()
        self.is_playing     = True
        self._set_breathing_ui()
        self.progress_bar.setValue(0)
        self.progress_label.setText("Breathing…")
        self.progress_label.setStyleSheet(f"color: {_FG}; background: transparent;")

    def _stop_breathing(self) -> None:
        """Stop the breathing loop and return the robot to home."""
        self._breathing     = False
        self._resume_breath = False
        if not self._head_rewinding and self.is_playing:
            self._start_head_rewind()

    def _start_interruption(self, keyword: str) -> None:
        """Pause breathing, play a one-shot interruption, then resume breathing."""
        # Save breathing position for resume
        self._breath_resume_idx   = self._head_play_idx
        self._breath_resume_pitch = self.prev_pitch
        self._breath_resume_roll  = self.prev_roll
        data = self._head_playback_data
        if self._head_play_idx < len(data):
            self._breath_resume_t = data[self._head_play_idx][0]
        else:
            self._breath_resume_t = data[-1][0] if data else 0.0

        self._breathing     = False
        self._resume_breath = True

        # Halt the current breathing queue
        if self.robot:
            try:
                self.robot.ClearMotion()
                self.robot.ResumeMotion()
            except Exception:
                pass

        # Load the interruption file
        folder  = os.path.join(os.path.dirname(__file__), params.HEAD_MOTION_PROFILE_FOLDER)
        matches = glob.glob(os.path.join(folder, f'*{keyword}*.txt'))
        matches += glob.glob(os.path.join(folder, f'*{keyword}*.csv'))
        if not matches:
            # No file — fall back to continuing breathing
            self._breathing     = True
            self._resume_breath = False
            self._set_breathing_ui()
            return
        self._load_file(max(matches))

        # Reset playback timing for the interruption
        self._head_play_idx = 0
        self.prev_pitch     = 0.0
        self.prev_roll      = 0.0
        self.playback_start = time.monotonic()
        self.is_playing     = True
        self._set_busy_ui()
        self.progress_label.setText(f"Playing {keyword}…")
        self.progress_label.setStyleSheet(f"color: {_FG}; background: transparent;")

    def _resume_breathing_from_saved(self) -> None:
        """Reload rest data and resume the breathing loop from the saved index."""
        folder  = os.path.join(os.path.dirname(__file__), params.HEAD_MOTION_PROFILE_FOLDER)
        matches = glob.glob(os.path.join(folder, f'*{params.HEAD_REST_PROFILE}*.txt'))
        matches += glob.glob(os.path.join(folder, f'*{params.HEAD_REST_PROFILE}*.csv'))
        if not matches or not self.robot:
            self._stop_playback()
            return
        self._load_file(max(matches))

        # Robot is at TRF home (0°,0°). Move it to the saved breathing orientation first.
        rp = self._breath_resume_pitch
        rr = self._breath_resume_roll
        if abs(rp) > 0.01 or abs(rr) > 0.01:
            try:
                self.robot.MoveLinRelTrf(
                    0, 0, 0,
                    params.HEAD_PITCH_SIGN * rp,
                    0,
                    params.HEAD_ROLL_SIGN  * rr,
                )
            except Exception as exc:
                print(f"[breath resume] {exc}")

        # Resume playback from the saved index with matching time offset
        self._head_play_idx = self._breath_resume_idx
        self.prev_pitch     = rp
        self.prev_roll      = rr
        self.playback_start = time.monotonic() - self._breath_resume_t
        self.is_playing     = True
        self._breathing     = True
        self._set_breathing_ui()
        self.progress_label.setText("Breathing (resumed)…")
        self.progress_label.setStyleSheet(f"color: {_FG}; background: transparent;")

    # ─────────────────────────────────────────
    #  Head motion preset (one-shot)
    # ─────────────────────────────────────────

    def _play_preset(self, keyword: str) -> None:
        if self._breathing:
            self._start_interruption(keyword)
            return
        folder  = os.path.join(os.path.dirname(__file__), params.HEAD_MOTION_PROFILE_FOLDER)
        matches = glob.glob(os.path.join(folder, f'*{keyword}*.txt'))
        matches += glob.glob(os.path.join(folder, f'*{keyword}*.csv'))
        if not matches:
            self.file_label.setText(f"No '{keyword}' profile found")
            self.file_label.setStyleSheet(f"color: {_RED}; background: transparent;")
            return
        self._load_file(max(matches))
        self._start_playback()

    def _browse_file(self) -> None:
        motion_dir = os.path.join(os.path.dirname(__file__), params.HEAD_MOTION_PROFILE_FOLDER)
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select head motion profile",
            motion_dir,
            "Text/CSV files (*.txt *.csv);;All files (*.*)",
        )
        if path:
            self._load_file(path)

    def _load_file(self, path: str) -> None:
        """Parse a head motion CSV and compute orientation via complementary filter.

        CSV columns: time (sample count), gyro0, gyro1, gyro2 (°/s), act0, act1, act2 (g)

        IMU frame (+X=inferior, +Y=nasal, +Z=out-of-face):
          pitch — rotation about Y=nasal  (nodding);    driven by gyro1
          roll  — rotation about X=inferior (lat tilt); driven by gyro0

        Timestamps are sample indices; multiplying by 0.001 converts to seconds
        (1 kHz sampling).  The initial sample defines t=0 s and 0°/0° orientation.

        Fusion pipeline
        ───────────────
        1. Accelerometer IIR low-pass filter (HEAD_ACCEL_LPF_BETA, ≈8 Hz cutoff)
        2. Accel-derived absolute tilt from gravity; reference subtracted so pose starts at 0°
        3. Gyro integration using actual dt from timestamps
        4. Complementary filter (HEAD_CF_ALPHA)
        """
        raw: list[tuple[float, float, float, float, float, float, float]] = []
        try:
            with open(path, newline='') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    if len(row) < 7:
                        continue
                    raw.append((
                        float(row[0]),
                        float(row[1]),
                        float(row[2]),
                        float(row[3]),
                        float(row[4]),
                        float(row[5]),
                        float(row[6]),
                    ))
        except Exception as exc:
            self.file_label.setText(f"Error loading file: {exc}")
            self.file_label.setStyleSheet(f"color: {_RED}; background: transparent;")
            return

        if not raw:
            self.file_label.setText("No valid data found")
            self.file_label.setStyleSheet(f"color: {_RED}; background: transparent;")
            return

        alpha = params.HEAD_CF_ALPHA
        beta  = params.HEAD_ACCEL_LPF_BETA

        t0_s = raw[0][0] * 0.001
        _, _, _, _, a0_0, a1_0, a2_0 = raw[0]
        ax_f, ay_f, az_f = a0_0, a1_0, a2_0

        az_safe   = az_f if abs(az_f) > 1e-6 else 1e-6
        pitch_ref = math.atan2(ax_f,  az_safe)
        roll_ref  = math.atan2(-ay_f, az_safe)

        pitch_deg = 0.0
        roll_deg  = 0.0
        t_prev_s  = t0_s

        result: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]

        for sample_count, g0, g1, _g2, a0, a1, a2 in raw[1:]:
            ax_f = beta * ax_f + (1.0 - beta) * a0
            ay_f = beta * ay_f + (1.0 - beta) * a1
            az_f = beta * az_f + (1.0 - beta) * a2

            az_safe     = az_f if abs(az_f) > 1e-6 else 1e-6
            pitch_accel = math.degrees(math.atan2(ax_f,  az_safe) - pitch_ref)
            roll_accel  = math.degrees(math.atan2(-ay_f, az_safe) - roll_ref)

            t_s   = sample_count * 0.001 - t0_s
            dt    = max(t_s - t_prev_s, 1e-6)
            pitch_gyro = pitch_deg + g1 * dt
            roll_gyro  = roll_deg  + g0 * dt

            pitch_deg = alpha * pitch_gyro + (1.0 - alpha) * pitch_accel
            roll_deg  = alpha * roll_gyro  + (1.0 - alpha) * roll_accel

            result.append((t_s, pitch_deg, roll_deg))
            t_prev_s = t_s

        self._head_playback_data = result
        self._head_play_idx  = 0
        self.is_playing      = False

        duration_s = result[-1][0]
        self.file_label.setText(os.path.basename(path))
        self.file_label.setStyleSheet(f"color: {_FG}; background: transparent;")
        self.info_label.setText(
            f"{len(result)} samples  ·  {duration_s:.1f} s  "
            f"·  CF α={alpha}  LPF β={beta}"
        )
        self.play_btn.setEnabled(True)
        self.progress_label.setText("")
        self.progress_bar.setValue(0)

    def _start_playback(self) -> None:
        if not self._head_playback_data or not self.robot:
            return
        self._head_play_idx = 0
        self.prev_pitch     = 0.0
        self.prev_roll      = 0.0
        self.playback_start = time.monotonic()
        self.is_playing     = True
        self._set_busy_ui()
        self.progress_bar.setValue(0)
        self.progress_label.setText("Playing…")
        print(f"[playback] robot={self.robot}, samples={len(self._head_playback_data)}, use_head={self.use_head}")

    def _on_stop_pressed(self) -> None:
        """Stop button handler — cancels breathing loop, interruption, or one-shot."""
        if self._head_rewinding:
            self._stop_playback()
        elif self.is_playing:
            self._breathing     = False
            self._resume_breath = False
            self._start_head_rewind()

    def _stop_playback(self) -> None:
        """Halt head motion playback and clear the robot's motion queue.

        ClearMotion() empties the queue but also pauses it; ResumeMotion()
        re-opens it so subsequent commands are accepted without a reconnect.
        """
        self.is_playing      = False
        self._head_rewinding = False
        self._breathing      = False
        self._resume_breath  = False
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        if self.robot:
            try:
                self.robot.ClearMotion()
                self.robot.ResumeMotion()
            except Exception:
                pass
        self._set_idle_ui()
        self.progress_label.setText("Stopped")
        self.progress_label.setStyleSheet(f"color: {_DIM}; background: transparent;")

    # ─────────────────────────────────────────
    #  TRF Cartesian jog
    # ─────────────────────────────────────────

    def _send_trf_step(self, axis: str, sign: float) -> None:
        if not self.robot:
            return
        lin = (self._trf_lin_step.value() if self._trf_lin_step else 1.0) * sign
        ang = (self._trf_ang_step.value() if self._trf_ang_step else 1.0) * sign
        deltas: dict[str, tuple] = {
            'x':  (lin, 0,   0,   0,   0,   0),
            'y':  (0,   lin, 0,   0,   0,   0),
            'z':  (0,   0,   lin, 0,   0,   0),
            'ux': (0,   0,   0,   ang, 0,   0),
            'uy': (0,   0,   0,   0,   ang, 0),
            'uz': (0,   0,   0,   0,   0,   ang),
        }
        try:
            self.robot.MoveLinRelTrf(*deltas[axis])
            delta = lin if axis in ('x', 'y', 'z') else ang
            self._cart_pos[self._AXIS_IDX[axis]] += delta
        except Exception as exc:
            print(f"[trf] {exc}")

    def _start_trf_jog(self, axis: str, sign: float) -> None:
        self._trf_jog_axis = axis
        self._trf_jog_sign = sign
        self._send_trf_step(axis, sign)
        self._trf_jog_timer.setInterval(400)
        self._trf_jog_timer.start()

    def _repeat_trf_jog(self) -> None:
        self._send_trf_step(self._trf_jog_axis, self._trf_jog_sign)
        self._trf_jog_timer.setInterval(200)

    def _stop_trf_jog(self) -> None:
        self._trf_jog_timer.stop()

    def _go_init_pose(self) -> None:
        if not self.robot:
            return
        try:
            self.robot.MoveJoints(*params.ROBOT_HEAD_INIT_POSE)
            self._cart_pos = [0.0] * 6
        except Exception as exc:
            print(f"[init] {exc}")

    def _set_trf_home(self) -> None:
        self._home_cart_offset = list(self._cart_pos)
        if self.robot:
            try:
                joints = self.robot.GetRtTargetJointPos()
                vals = joints.data if hasattr(joints, 'data') else joints
                self._head_home_joints = list(vals)
                print(f"[home] Joint home set: {[round(v, 2) for v in self._head_home_joints]}")
            except Exception as exc:
                print(f"[home] Could not read joints: {exc}")

    def _go_trf_home(self) -> None:
        if not self.robot:
            return
        delta = [h - c for h, c in zip(self._home_cart_offset, self._cart_pos)]
        if all(abs(d) < 0.001 for d in delta):
            return
        try:
            self.robot.MoveLinRelTrf(*delta)
            self._cart_pos = list(self._home_cart_offset)
        except Exception as exc:
            print(f"[home] {exc}")

    # ─────────────────────────────────────────
    #  Robot error recovery
    # ─────────────────────────────────────────

    def _reset_robot_error(self) -> None:
        if not self.robot:
            return
        try:
            self.robot.ResetError()
            self.robot.ResumeMotion()
            print("[robot] Error reset.")
        except Exception as exc:
            print(f"[robot] Reset failed: {exc}")

    def _get_robot_pose(self) -> None:
        if not self.robot or self._pose_label is None:
            return
        try:
            joints = self.robot.GetRtTargetJointPos()
            vals   = joints.data if hasattr(joints, 'data') else joints
            text   = "  ".join(f"J{i+1}: {v:.1f}°" for i, v in enumerate(vals))
            self._pose_label.setText(text)
            self._pose_label.setStyleSheet(f"color: {_FG}; background: transparent;")
            print(f"[pose] {text}")
        except Exception as exc:
            self._pose_label.setText(f"Error: {exc}")
            self._pose_label.setStyleSheet(f"color: {_RED}; background: transparent;")

    # ─────────────────────────────────────────
    #  Control loop
    # ─────────────────────────────────────────

    def _tick(self) -> None:
        """Main loop called every 1/LOOP_HZ seconds via QTimer."""
        if params.QUIT_FLAG:
            self.close()
            return

        step = self._gimbal_speed_slider.value() if self._gimbal_speed_slider else params.STEP_SIZE

        if self.use_eye:
            if self._gimbal_resetting:
                self._tick_gimbal_reset(step)
            else:
                self._tick_gimbal_wasd(step)
                self._tick_eye_profile()

        if self.use_head:
            self._tick_head_playback()

    def _tick_gimbal_wasd(self, step: int) -> None:
        if params.GIMBAL_KEYS['up']:
            new = clamp(self.pos1 + step, params.JOINT_MIN_1, params.JOINT_MAX_1)
            if new != self.pos1:
                self.pos1 = new
                write_position(self.port_handler, self.packet_handler, params.DXL_1, self.pos1)
        elif params.GIMBAL_KEYS['down']:
            new = clamp(self.pos1 - step, params.JOINT_MIN_1, params.JOINT_MAX_1)
            if new != self.pos1:
                self.pos1 = new
                write_position(self.port_handler, self.packet_handler, params.DXL_1, self.pos1)

        if params.GIMBAL_KEYS['left']:
            new = clamp(self.pos2 + step, params.JOINT_MIN_2, params.JOINT_MAX_2)
            if new != self.pos2:
                self.pos2 = new
                write_position(self.port_handler, self.packet_handler, params.DXL_2, self.pos2)
        elif params.GIMBAL_KEYS['right']:
            new = clamp(self.pos2 - step, params.JOINT_MIN_2, params.JOINT_MAX_2)
            if new != self.pos2:
                self.pos2 = new
                write_position(self.port_handler, self.packet_handler, params.DXL_2, self.pos2)

    def _tick_gimbal_reset(self, step: int) -> None:
        def _step_toward_center(pos: int) -> int:
            diff = params.EYE_CENTER - pos
            if diff == 0:
                return pos
            return pos + (min(step, abs(diff)) * (1 if diff > 0 else -1))

        new1 = _step_toward_center(self.pos1)
        new2 = _step_toward_center(self.pos2)

        if new1 != self.pos1:
            self.pos1 = new1
            write_position(self.port_handler, self.packet_handler, params.DXL_1, self.pos1)
        if new2 != self.pos2:
            self.pos2 = new2
            write_position(self.port_handler, self.packet_handler, params.DXL_2, self.pos2)

        if self.pos1 == params.EYE_CENTER and self.pos2 == params.EYE_CENTER:
            self._gimbal_resetting = False

    def _tick_eye_profile(self) -> None:
        if self._eye_playing and not self._eye_rewinding:
            elapsed = time.monotonic() - self._eye_play_start
            n_total = len(self._eye_profile_data)

            while self._eye_play_idx < n_total:
                t, m1, m2 = self._eye_profile_data[self._eye_play_idx]
                if t > elapsed:
                    break
                write_position(self.port_handler, self.packet_handler, params.DXL_1, m1)
                write_position(self.port_handler, self.packet_handler, params.DXL_2, m2)
                self.pos1, self.pos2 = m1, m2
                self._eye_play_idx += 1

            if self._eye_play_idx >= n_total:
                self._start_eye_rewind()
            else:
                pct = int(50 * self._eye_play_idx / n_total)
                self._eye_progress_bar.setValue(pct)
                self._eye_progress_lbl.setText(f"Playing…  {self._eye_play_idx}/{n_total}")

        elif self._eye_rewinding:
            if self._eye_rewind_idx < len(self._eye_rewind_steps):
                m1, m2 = self._eye_rewind_steps[self._eye_rewind_idx]
                write_position(self.port_handler, self.packet_handler, params.DXL_1, m1)
                write_position(self.port_handler, self.packet_handler, params.DXL_2, m2)
                self.pos1, self.pos2 = m1, m2
                self._eye_rewind_idx += 1
                pct = 50 + int(50 * self._eye_rewind_idx / len(self._eye_rewind_steps))
                self._eye_progress_bar.setValue(pct)
            else:
                self._eye_playing   = False
                self._eye_rewinding = False
                self._eye_progress_bar.setValue(100)
                self._eye_progress_lbl.setText("Done ✓")
                self._eye_progress_lbl.setStyleSheet(f"color: {_GREEN}; background: transparent;")
                self._bells_btn.setEnabled(True)
                self._saccadic_btn.setEnabled(True)

    def _tick_head_playback(self) -> None:
        """Advance head motion playback (or rewind) by one 50 Hz tick.

        Forward pass: accumulate all orientation deltas due within this tick
        and send a single MoveLinRelTrf.  When the data is exhausted, kick off
        a linear return-to-home rewind over ~1.5 s.

        Coordinate mapping (IMU frame → robot TRF):
          pitch (sagittal tilt) → UX  scaled by HEAD_PITCH_SIGN
          roll  (frontal tilt)  → UZ  scaled by HEAD_ROLL_SIGN
        """
        if not self.is_playing or not self.robot:
            return

        if self._head_rewinding:
            if self._head_return_done:
                self._head_rewinding   = False
                self._head_return_done = False
                if self._resume_breath:
                    self._resume_breath = False
                    self._resume_breathing_from_saved()
                else:
                    self._stop_playback()
                    self.progress_bar.setMinimum(0)
                    self.progress_bar.setMaximum(100)
                    self.progress_bar.setValue(0)
                    self.progress_label.setText("Done ✓")
                    self.progress_label.setStyleSheet(f"color: {_GREEN}; background: transparent;")
            return

        # Forward playback
        elapsed_s = time.monotonic() - self.playback_start
        n_total   = len(self._head_playback_data)
        acc_pitch = 0.0
        acc_roll  = 0.0

        while self._head_play_idx < n_total:
            t_s, pitch, roll = self._head_playback_data[self._head_play_idx]
            if t_s > elapsed_s:
                break
            d_pitch = pitch - self.prev_pitch
            d_roll  = roll  - self.prev_roll
            if abs(d_pitch) < 5.0 and abs(d_roll) < 5.0:
                acc_pitch += d_pitch
                acc_roll  += d_roll
            self.prev_pitch = pitch
            self.prev_roll  = roll
            self._head_play_idx += 1

        if abs(acc_pitch) > 0.01 or abs(acc_roll) > 0.01:
            try:
                self.robot.MoveLinRelTrf(
                    0, 0, 0,
                    params.HEAD_PITCH_SIGN * acc_pitch,
                    0,
                    params.HEAD_ROLL_SIGN * acc_roll,
                )
            except Exception as exc:
                print(f"[head playback] {exc}")

        if self._head_play_idx >= n_total:
            if self._breathing:
                # Seamless loop: restart from the beginning
                self._head_play_idx = 0
                self.prev_pitch     = 0.0
                self.prev_roll      = 0.0
                self.playback_start = time.monotonic()
            else:
                self._start_head_rewind()
        else:
            pct = int(100 * self._head_play_idx / n_total)
            self.progress_bar.setValue(pct)
            if self._breathing:
                self.progress_label.setText(f"Breathing…  {elapsed_s:.1f} s  ({pct}%)")
            else:
                self.progress_label.setText(f"{elapsed_s:.1f} s  ({pct}%)")

    def _start_head_rewind(self) -> None:
        """Send MoveJoints to home and wait in a background thread for completion."""
        if self.robot:
            try:
                self.robot.ClearMotion()
                self.robot.ResumeMotion()
                self.robot.MoveJoints(*self._head_home_joints)
            except Exception as exc:
                print(f"[head return] {exc}")
        self._head_return_done = False
        self._head_rewinding   = True
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(0)   # indeterminate / busy animation
        self.progress_label.setText("Returning home…")
        self.progress_label.setStyleSheet(f"color: {_DIM}; background: transparent;")
        threading.Thread(target=self._wait_head_return, daemon=True).start()

    def _wait_head_return(self) -> None:
        """Block until the robot finishes moving, then signal the tick loop."""
        try:
            if self.robot:
                self.robot.WaitIdle(timeout=30)
        except Exception as exc:
            print(f"[head return wait] {exc}")
        self._head_return_done = True

    # ─────────────────────────────────────────
    #  Shutdown
    # ─────────────────────────────────────────

    def closeEvent(self, event) -> None:
        self._on_close()
        event.accept()

    def _on_close(self) -> None:
        """Stop all motion, disconnect hardware, and shut down."""
        self._timer.stop()
        self._trf_jog_timer.stop()

        self._eye_playing   = False
        self._eye_rewinding = False
        if self.is_playing:
            self._stop_playback()
        self.listener.stop()

        if self.use_eye and self.port_handler:
            disable_motor(self.port_handler, self.packet_handler, params.DXL_1)
            disable_motor(self.port_handler, self.packet_handler, params.DXL_2)
            self.port_handler.closePort()
            print("Gimbal port closed.")

        if self.use_head and self.robot:
            try:
                self.robot.ClearMotion()
                self.robot.DeactivateRobot()
                self.robot.Disconnect()
                print("Robot disconnected.")
            except Exception:
                pass

        params.ROBOT_INSTANCE = None
        enable_echo()
