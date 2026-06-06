"""Tkinter control window for patient motion interface.

Provides a dark-themed GUI with:
  - Gimbal speed tuning and WASD key-binding reference
  - Eye motion profile playback (Bell's reflex, saccadic) with auto-rewind
  - Head motion playback from IMU CSV files
  - Joint jog sliders (J1–J6) for the Meca500
  - TRF Cartesian jog buttons (X/Y/Z translation and UX/UY/UZ rotation)
"""

from __future__ import annotations

import csv
import glob
import os
import time
import tkinter as tk
from tkinter import filedialog, ttk
from typing import Optional

import Parameter as params
from Utils import clamp, disable_motor, enable_echo, write_position


# ─────────────────────────────────────────────
#  Dark UI palette (Catppuccin Mocha)
# ─────────────────────────────────────────────

_BG    = "#1e1e2e"   # window / page background
_CARD  = "#2a2a3e"   # card surface
_ACCT  = "#7c6af5"   # accent (labels, progress)
_FG    = "#cdd6f4"   # primary text
_DIM   = "#6c7086"   # secondary text / disabled
_SEP   = "#313244"   # separators / trough

# Button backgrounds — light enough for black text on macOS native rendering
_BTN_PURPLE = "#c4bbfc"   # accent actions  (Browse, Set Home)
_BTN_GREEN  = "#a6e3a1"   # positive actions (Play, Go Home)
_BTN_RED    = "#f38ba8"   # destructive      (Stop, Reset Error)
_BTN_GRAY   = "#9399b2"   # hold-to-jog arrows

# Semantic aliases kept for progress-label colours
_GREEN = "#a6e3a1"
_RED   = "#f38ba8"


class ControlWindow:
    """Main control window, driven by a tkinter event loop.

    Args:
        root: Tk root window.
        use_head: Whether the Meca500 head robot is active.
        use_eye: Whether the Dynamixel eye gimbal is active.
        robot: mecademicpy Robot instance, or None.
        port_handler: Dynamixel PortHandler, or None.
        packet_handler: Dynamixel PacketHandler, or None.
        listener: pynput Listener (stopped on close).
    """

    # Maps TRF axis name → index in the 6-DOF pose vector
    _AXIS_IDX: dict[str, int] = {'x': 0, 'y': 1, 'z': 2, 'ux': 3, 'uy': 4, 'uz': 5}

    def __init__(
        self,
        root: tk.Tk,
        use_head: bool,
        use_eye: bool,
        robot,
        port_handler,
        packet_handler,
        listener,
    ) -> None:
        self.root           = root
        self.use_head       = use_head
        self.use_eye        = use_eye
        self.robot          = robot
        self.port_handler   = port_handler
        self.packet_handler = packet_handler
        self.listener       = listener

        # Current gimbal encoder positions
        self.pos1 = params.EYE_CENTER
        self.pos2 = params.EYE_CENTER

        # Gimbal speed Tk variable (shared between Scale and velocity label)
        self.step_var = tk.IntVar(value=params.STEP_SIZE)

        # Eye motion profile playback state
        self._eye_profile_data: list[tuple[float, int, int]] = []
        self._eye_play_idx     = 0
        self._eye_play_start   = 0.0
        self._eye_playing      = False
        self._eye_rewinding    = False
        self._eye_rewind_steps: list[tuple[int, int]] = []
        self._eye_rewind_idx   = 0

        # Eye profile UI widgets (populated in _build_eye_profiles_card)
        self._bells_btn:        Optional[tk.Button]        = None
        self._saccadic_btn:     Optional[tk.Button]        = None
        self._eye_progress_bar: Optional[ttk.Progressbar]  = None
        self._eye_progress_lbl: Optional[tk.Label]         = None

        # Gimbal reset state
        self._gimbal_resetting = False

        # Head motion playback state
        self.playback_data:  list[tuple[int, float, float]] = []
        self.playback_idx    = 0
        self.is_playing      = False
        self.playback_start  = 0.0
        self.prev_pitch      = 0.0
        self.prev_roll       = 0.0

        # Joint jog state
        self.joint_vars:   list[tk.DoubleVar] = []
        self.jog_vel_var:  Optional[tk.DoubleVar] = None
        self._jog_after_id = None

        # TRF Cartesian jog state
        self.trf_lin_step_var: Optional[tk.DoubleVar] = None
        self.trf_ang_step_var: Optional[tk.DoubleVar] = None
        self._trf_after_id     = None
        self._cart_pos         = [0.0] * 6   # cumulative offset from init pose
        self._home_cart_offset = [0.0] * 6   # saved home offset

        self._build_ui()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    # ─────────────────────────────────────────
    #  Widget factory helpers
    # ─────────────────────────────────────────

    @staticmethod
    def _lighten(hex_color: str, amount: int = 25) -> str:
        """Return *hex_color* brightened by *amount* per channel (capped at 255)."""
        r = min(255, int(hex_color[1:3], 16) + amount)
        g = min(255, int(hex_color[3:5], 16) + amount)
        b = min(255, int(hex_color[5:7], 16) + amount)
        return f'#{r:02x}{g:02x}{b:02x}'

    def _card(self, parent: tk.Frame, fill: str = 'x', expand: bool = False) -> tk.Frame:
        """Create and pack a rounded card frame inside *parent*."""
        frame = tk.Frame(
            parent, bg=_CARD, padx=14, pady=10,
            highlightbackground=_SEP, highlightthickness=1,
        )
        frame.pack(fill=fill, expand=expand, padx=12, pady=4)
        return frame

    def _lbl(
        self,
        parent: tk.Widget,
        text: str,
        size: int = 9,
        bold: bool = False,
        color: Optional[str] = None,
    ) -> tk.Label:
        """Create (but do not pack) a styled Label."""
        font = ('Arial', size, 'bold') if bold else ('Arial', size)
        return tk.Label(parent, text=text, bg=parent['bg'], fg=color or _FG, font=font)

    def _sep(self, parent: tk.Frame) -> None:
        """Pack a 1-pixel horizontal separator into *parent*."""
        tk.Frame(parent, bg=_SEP, height=1).pack(fill='x', pady=(4, 6))

    def _btn(
        self,
        parent: tk.Widget,
        text: str,
        command,
        color: str = _BTN_PURPLE,
        fg: str = 'black',
        width: int = 10,
    ) -> tk.Button:
        """Create (but do not pack) a styled flat Button."""
        btn = tk.Button(
            parent, text=text, command=command,
            bg=color, fg=fg,
            activebackground=self._lighten(color), activeforeground=fg,
            disabledforeground='#888888',
            font=('Arial', 9, 'bold'),
            relief='flat', bd=0, padx=10, pady=5, cursor='hand2', width=width,
        )
        btn.bind('<Enter>', lambda _, c=color: btn.config(bg=self._lighten(c)))
        btn.bind('<Leave>', lambda _, c=color: btn.config(bg=c))
        return btn

    def _scale(self, parent: tk.Widget, **kwargs) -> tk.Scale:
        """Create (but do not pack) a styled Scale widget."""
        return tk.Scale(
            parent, bg=_CARD, fg=_FG, troughcolor=_SEP,
            activebackground=_ACCT, highlightthickness=0, bd=0,
            **kwargs,
        )

    def _jog_btn(
        self,
        parent: tk.Widget,
        label: str,
        axis: str,
        sign: float,
        width: int = 3,
    ) -> tk.Button:
        """Create a hold-to-jog TRF button that sends moves while pressed."""
        btn = tk.Button(
            parent, text=label, width=width,
            bg=_BTN_GRAY, fg='black',
            activebackground=self._lighten(_BTN_GRAY), activeforeground='black',
            disabledforeground='#888888',
            font=('Arial', 11, 'bold'),
            relief='flat', bd=0, padx=6, pady=6, cursor='hand2',
        )
        btn.bind('<ButtonPress-1>',   lambda _: self._start_trf_jog(axis, sign))
        btn.bind('<ButtonRelease-1>', lambda _: self._stop_trf_jog())
        btn.bind('<Enter>', lambda _: btn.config(bg=self._lighten(_BTN_GRAY)))
        btn.bind('<Leave>', lambda _: btn.config(bg=_BTN_GRAY))
        return btn

    # ─────────────────────────────────────────
    #  UI construction
    # ─────────────────────────────────────────

    def _build_ui(self) -> None:
        """Construct the full window layout."""
        self._configure_window()
        left, right, far_right = self._build_body_columns()

        if self.use_eye:
            self._build_gimbal_speed_card(left)

        self._build_key_bindings_card(left)

        if self.use_eye:
            self._build_eye_profiles_card(left)

        if self.use_head:
            self._build_motion_playback_card(left)
            self._build_joint_jog_card(right)
            self._build_trf_jog_card(far_right)

        tk.Frame(self.root, bg=_BG, height=8).pack()

    def _configure_window(self) -> None:
        """Set window title, background, and header banner."""
        self.root.title("Patient Motion Control")
        self.root.configure(bg=_BG)
        self.root.attributes('-topmost', True)
        self.root.resizable(False, False)

        header = tk.Frame(self.root, bg=_ACCT, pady=12)
        header.pack(fill='x')
        self._lbl(header, "Patient Motion Control", 13, bold=True, color='#ffffff').pack()

        parts = []
        if self.use_head:
            parts.append("Head  (Meca500)")
        if self.use_eye:
            parts.append("Eye  (Dynamixel)")
        self._lbl(header, "  +  ".join(parts), 9, color='#ddd6fe').pack(pady=(2, 0))

    def _build_body_columns(self) -> tuple[tk.Frame, tk.Frame, tk.Frame]:
        """Pack the three-column body frame and return (left, right, far_right)."""
        body = tk.Frame(self.root, bg=_BG)
        body.pack(fill='both', expand=True)

        left      = tk.Frame(body, bg=_BG)
        right     = tk.Frame(body, bg=_BG)
        far_right = tk.Frame(body, bg=_BG)

        for col in (left, right, far_right):
            col.pack(side=tk.LEFT, fill='both', expand=True)

        return left, right, far_right

    def _build_gimbal_speed_card(self, parent: tk.Frame) -> None:
        """Gimbal speed card: step-size slider + live velocity readout."""
        card = self._card(parent)
        self._lbl(card, "Gimbal Speed", 10, bold=True).pack(anchor='w')
        self._sep(card)

        row = tk.Frame(card, bg=_CARD)
        row.pack(fill='x')
        self._lbl(row, "Step", 9, color=_DIM).pack(side=tk.LEFT)
        self._lbl(row, "counts / tick", 9, color=_DIM).pack(side=tk.RIGHT)

        self._scale(card, from_=1, to=90, orient=tk.HORIZONTAL,
                    variable=self.step_var, showvalue=True,
                    length=240).pack(fill='x', pady=(4, 2))

        self.vel_label = self._lbl(card, "", 9, color=_DIM)
        self.vel_label.pack(anchor='w')
        self._update_vel()
        self.step_var.trace_add('write', lambda *_: self._update_vel())

        self._sep(card)
        self._btn(card, "Gimbal Reset", self._reset_gimbal,
                  color=_BTN_GRAY, fg='black', width=14).pack(anchor='w')

    def _build_key_bindings_card(self, parent: tk.Frame) -> None:
        """Key bindings reference card."""
        card = self._card(parent)
        self._lbl(card, "Key Bindings", 10, bold=True).pack(anchor='w')
        self._sep(card)

        bindings: list[tuple[str, str, str]] = []
        if self.use_eye:
            bindings += [
                ("W / S",    "Eye  Superior / Inferior", _ACCT),
                ("A / D",    "Eye  Temporal / Nasal",    _ACCT),
            ]
        bindings.append(("Q / ESC", "Quit", _RED))

        for key_text, action, color in bindings:
            row = tk.Frame(card, bg=_CARD)
            row.pack(fill='x', pady=2)
            self._lbl(row, key_text, 9, bold=True, color=color).pack(side=tk.LEFT)
            self._lbl(row, action,   9, color=_FG).pack(side=tk.RIGHT)

    def _build_eye_profiles_card(self, parent: tk.Frame) -> None:
        """Eye motion profile card: Bell's reflex and saccadic buttons + progress."""
        card = self._card(parent)
        self._lbl(card, "Eye Motion Profiles", 10, bold=True).pack(anchor='w')
        self._sep(card)

        btn_row = tk.Frame(card, bg=_CARD)
        btn_row.pack(fill='x', pady=(0, 6))

        self._bells_btn = self._btn(
            btn_row, "Bell's Reflex",
            lambda: self._start_eye_profile('bells'),
            color=_BTN_PURPLE, fg='black', width=13,
        )
        self._bells_btn.pack(side=tk.LEFT, padx=(0, 6))

        self._saccadic_btn = self._btn(
            btn_row, "Saccadic",
            lambda: self._start_eye_profile('saccadic'),
            color=_BTN_PURPLE, fg='black', width=10,
        )
        self._saccadic_btn.pack(side=tk.LEFT)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure(
            "Eye.Horizontal.TProgressbar",
            troughcolor=_SEP, background=_ACCT,
            borderwidth=0, lightcolor=_ACCT, darkcolor=_ACCT,
        )
        self._eye_progress_bar = ttk.Progressbar(
            card, maximum=100, value=0, style="Eye.Horizontal.TProgressbar",
        )
        self._eye_progress_bar.pack(fill='x', pady=(0, 4))

        self._eye_progress_lbl = self._lbl(card, "Ready", 9, color=_DIM)
        self._eye_progress_lbl.pack(anchor='w')

    def _build_motion_playback_card(self, parent: tk.Frame) -> None:
        """Head motion playback card: file browser, play/stop, and progress bar."""
        card = self._card(parent)
        self._lbl(card, "Motion Playback", 10, bold=True).pack(anchor='w')
        self._sep(card)

        file_row = tk.Frame(card, bg=_CARD)
        file_row.pack(fill='x', pady=(0, 6))
        self.file_label = self._lbl(file_row, "No file loaded", 9, color=_DIM)
        self.file_label.pack(side=tk.LEFT, anchor='w')
        self._btn(file_row, "Browse…", self._browse_file, width=8).pack(side=tk.RIGHT)

        self.info_label = self._lbl(card, "", 9, color=_DIM)
        self.info_label.pack(anchor='w', pady=(0, 6))

        btn_row = tk.Frame(card, bg=_CARD)
        btn_row.pack(fill='x', pady=(0, 6))

        self.play_btn = self._btn(btn_row, "▶  Play", self._start_playback,
                                  color=_BTN_GREEN, fg='black', width=9)
        self.play_btn.config(state=tk.DISABLED)
        self.play_btn.pack(side=tk.LEFT, padx=(0, 6))

        self.stop_btn = self._btn(btn_row, "■  Stop", self._stop_playback,
                                  color=_BTN_RED, fg='black', width=9)
        self.stop_btn.config(state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure(
            "Accent.Horizontal.TProgressbar",
            troughcolor=_SEP, background=_ACCT,
            borderwidth=0, lightcolor=_ACCT, darkcolor=_ACCT,
        )
        self.progress_bar = ttk.Progressbar(
            card, maximum=100, value=0, style="Accent.Horizontal.TProgressbar",
        )
        self.progress_bar.pack(fill='x', pady=(0, 4))

        self.progress_label = self._lbl(card, "", 9, color=_DIM)
        self.progress_label.pack(anchor='w')

    def _build_joint_jog_card(self, parent: tk.Frame) -> None:
        """Joint jog card: velocity slider and per-axis position sliders (J1–J6)."""
        card = self._card(parent, fill='both', expand=True)
        self._lbl(card, "Joint Jog", 10, bold=True).pack(anchor='w')
        self._sep(card)

        vel_row = tk.Frame(card, bg=_CARD)
        vel_row.pack(fill='x', pady=(0, 4))
        self._lbl(vel_row, "Velocity", 9, color=_DIM).pack(side=tk.LEFT, padx=(0, 8))
        self.jog_vel_var = tk.DoubleVar(value=params.MAX_JOINT_VEL_PERCENTAGE)
        self._scale(vel_row, from_=params.JOINT_VEL_MIN, to=params.JOINT_VEL_MAX,
                    resolution=0.1, orient=tk.HORIZONTAL,
                    variable=self.jog_vel_var, length=160).pack(side=tk.LEFT)
        self._lbl(vel_row, "%", 9, color=_DIM).pack(side=tk.LEFT, padx=4)
        self.jog_vel_var.trace_add('write', lambda *_: self._apply_vel_limit())

        for i, (label, init_angle) in enumerate(
                zip(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'], params.ROBOT_HEAD_INIT_POSE)):
            lo, hi = params.MECA500_JOINT_LIMITS[i]
            row = tk.Frame(card, bg=_CARD)
            row.pack(fill='x', pady=3)
            self._lbl(row, label, 9, bold=True, color=_ACCT).pack(side=tk.LEFT, padx=(0, 6))
            var = tk.DoubleVar(value=init_angle)
            self._scale(row, from_=lo, to=hi, resolution=0.5,
                        orient=tk.HORIZONTAL, variable=var, length=200).pack(side=tk.LEFT)
            var.trace_add('write', lambda *_: self._schedule_jog())
            self.joint_vars.append(var)

    def _build_trf_jog_card(self, parent: tk.Frame) -> None:
        """TRF Cartesian jog card: step-size inputs, hold-to-jog buttons, home controls."""
        card = self._card(parent, fill='both', expand=True)
        self._lbl(card, "TRF Cartesian Jog", 10, bold=True).pack(anchor='w')
        self._sep(card)

        # Step size spinboxes
        step_grid = tk.Frame(card, bg=_CARD)
        step_grid.pack(fill='x', pady=(0, 8))
        self.trf_lin_step_var = tk.DoubleVar(value=1.0)
        self.trf_ang_step_var = tk.DoubleVar(value=1.0)
        for label, var, unit in [
            ("Linear step",  self.trf_lin_step_var, "mm"),
            ("Angular step", self.trf_ang_step_var, "°"),
        ]:
            row = tk.Frame(step_grid, bg=_CARD)
            row.pack(fill='x', pady=2)
            self._lbl(row, label, 9, color=_DIM).pack(side=tk.LEFT)
            tk.Spinbox(row, from_=0.1, to=50.0, increment=0.5,
                       textvariable=var, width=5, font=('Arial', 9),
                       bg=_BG, fg=_FG, buttonbackground=_SEP,
                       insertbackground=_FG, relief='flat').pack(side=tk.RIGHT)
            self._lbl(row, unit, 9, color=_DIM).pack(side=tk.RIGHT, padx=(0, 4))

        # Translation axes: (axis, negative-direction label, positive-direction label)
        self._lbl(card, "Translation", 9, bold=True, color=_DIM).pack(anchor='w', pady=(4, 2))
        for axis, neg_lbl, pos_lbl in [
            ('x', 'Nasal',    'Temporal'),
            ('y', 'Down',     'Up'),
            ('z', 'Inferior', 'Superior'),
        ]:
            row = tk.Frame(card, bg=_CARD)
            row.pack(fill='x', pady=3)
            self._jog_btn(row, f"- {neg_lbl}", axis, -1, width=10).pack(side=tk.LEFT)
            self._lbl(row, axis.upper(), 10, bold=True, color=_ACCT).pack(
                side=tk.LEFT, expand=True)
            self._jog_btn(row, f"{pos_lbl} +", axis, +1, width=10).pack(side=tk.RIGHT)

        # Rotation axes
        self._lbl(card, "Rotation", 9, bold=True, color=_DIM).pack(anchor='w', pady=(8, 2))
        for axis, neg_lbl, pos_lbl in [
            ('ux', 'Inferior', 'Superior'),
            ('uy', 'Left',     'Right'),
            ('uz', 'Temporal', 'Nasal'),
        ]:
            row = tk.Frame(card, bg=_CARD)
            row.pack(fill='x', pady=3)
            self._jog_btn(row, f"- {neg_lbl}", axis, -1, width=10).pack(side=tk.LEFT)
            self._lbl(row, axis, 10, bold=True, color=_ACCT).pack(side=tk.LEFT, expand=True)
            self._jog_btn(row, f"{pos_lbl} +", axis, +1, width=10).pack(side=tk.RIGHT)

        tk.Frame(card, bg=_SEP, height=1).pack(fill='x', pady=(10, 6))

        home_row = tk.Frame(card, bg=_CARD)
        home_row.pack(fill='x', pady=(0, 4))
        self._btn(home_row, "Set Home", self._set_trf_home,
                  color=_BTN_PURPLE, fg='black', width=10).pack(side=tk.LEFT, padx=(0, 6))
        self._btn(home_row, "Go Home", self._go_trf_home,
                  color=_BTN_GREEN, fg='black', width=10).pack(side=tk.LEFT)

        self._btn(card, "Reset Error", self._reset_robot_error,
                  color=_BTN_RED, fg='black', width=12).pack(anchor='w')

    # ─────────────────────────────────────────
    #  Gimbal speed display
    # ─────────────────────────────────────────

    def _update_vel(self) -> None:
        """Recompute and display the velocity corresponding to the current step size."""
        if not hasattr(self, 'vel_label'):
            return
        try:
            step = self.step_var.get()
        except tk.TclError:
            return
        counts_per_sec = step * params.LOOP_HZ
        deg_per_sec    = counts_per_sec * 360 / params.COUNTS_PER_REV
        rpm            = counts_per_sec / params.COUNTS_PER_REV * 60
        self.vel_label.config(text=f"{deg_per_sec:.1f} °/s   ·   {rpm:.1f} RPM")

    # ─────────────────────────────────────────
    #  Eye motion profiles
    # ─────────────────────────────────────────

    def _reset_gimbal(self) -> None:
        """Begin a smooth return of both motors to EYE_CENTER at the current step speed."""
        if not self.port_handler or self._eye_playing or self._eye_rewinding:
            return
        self._gimbal_resetting = True

    def _find_eye_profile(self, keyword: str) -> str:
        """Return the path of the most-recent CSV in eye_motion_profile/ matching *keyword*.

        The lexicographically largest filename is returned, which picks the
        most-recent date-prefixed file (e.g. 20260604_bells.csv).

        Raises:
            FileNotFoundError: If no matching CSV exists.
        """
        folder  = os.path.join(os.path.dirname(__file__), params.EYE_MOTION_PROFILE_FOLDER)
        matches = glob.glob(os.path.join(folder, f'*{keyword}*.csv'))
        if not matches:
            raise FileNotFoundError(
                f"No eye profile CSV matching '*{keyword}*.csv' in {folder}")
        return max(matches)

    def _load_eye_profile(self, keyword: str) -> list[tuple[float, int, int]]:
        """Parse an eye motion CSV and convert to timed motor-count tuples.

        CSV columns: t (s), x (deg), y (deg)
          - positive x = inferior  → DXL_1 position decreases (S key direction)
          - positive y = nasal     → DXL_2 position decreases (D key direction)

        Motor count formula:
          m = clamp(EYE_CENTER - angle_deg * COUNTS_PER_DEG, JOINT_MIN, JOINT_MAX)

        Returns:
            List of (t_seconds, m1_counts, m2_counts) tuples.
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
        """Load a profile and begin playback; ignored if already playing."""
        if self._eye_playing or self._eye_rewinding or self._gimbal_resetting:
            return
        try:
            data = self._load_eye_profile(keyword)
        except Exception as exc:
            print(f"[eye profile] {exc}")
            return

        # Reset motors to center before starting
        write_position(self.port_handler, self.packet_handler, params.DXL_1, params.EYE_CENTER)
        write_position(self.port_handler, self.packet_handler, params.DXL_2, params.EYE_CENTER)
        self.pos1 = params.EYE_CENTER
        self.pos2 = params.EYE_CENTER

        self._eye_profile_data = data
        self._eye_play_idx     = 0
        self._eye_play_start   = time.monotonic()
        self._eye_playing      = True
        self._eye_rewinding    = False

        self._bells_btn.config(state=tk.DISABLED)
        self._saccadic_btn.config(state=tk.DISABLED)
        self._eye_progress_bar['value'] = 0
        self._eye_progress_lbl.config(text="Playing…", fg=_FG)

    def _start_eye_rewind(self) -> None:
        """Generate a ~1.5 s linear interpolation back to EYE_CENTER and start it."""
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
        self._eye_progress_lbl.config(text="Rewinding…", fg=_DIM)

    # ─────────────────────────────────────────
    #  Head motion playback
    # ─────────────────────────────────────────

    def _browse_file(self) -> None:
        """Open a file-chooser dialog and load the selected motion CSV."""
        motion_dir = os.path.join(os.path.dirname(__file__), params.MOTION_DATA_FOLDER)
        path = filedialog.askopenfilename(
            initialdir=motion_dir,
            title="Select motion data file",
            filetypes=[("Text/CSV files", "*.txt *.csv"), ("All files", "*.*")],
        )
        if path:
            self._load_file(path)

    def _load_file(self, path: str) -> None:
        """Parse a head motion CSV (columns: time_ms, …, pitch, roll) into playback_data."""
        data: list[tuple[int, float, float]] = []
        try:
            with open(path, newline='') as f:
                reader = csv.reader(f)
                next(reader)   # skip header row
                for row in reader:
                    if len(row) < 9:
                        continue
                    data.append((int(float(row[0])), float(row[7]), float(row[8])))
        except Exception as exc:
            self.file_label.config(text=f"Error loading file: {exc}", fg=_RED)
            return

        if not data:
            self.file_label.config(text="No valid data found", fg=_RED)
            return

        t0 = data[0][0]
        self.playback_data = [(t - t0, p, r) for t, p, r in data]
        self.playback_idx  = 0
        self.is_playing    = False

        duration_s = self.playback_data[-1][0] / 1000.0
        self.file_label.config(text=os.path.basename(path), fg=_FG)
        self.info_label.config(text=f"{len(data)} samples  ·  {duration_s:.1f} s", fg=_DIM)
        self.play_btn.config(state=tk.NORMAL)
        self.progress_label.config(text="")
        self.progress_bar['value'] = 0

    def _start_playback(self) -> None:
        """Begin replaying the loaded head motion file."""
        if not self.playback_data or not self.robot:
            return
        self.playback_idx   = 0
        self.prev_pitch     = self.playback_data[0][1]
        self.prev_roll      = self.playback_data[0][2]
        self.playback_start = time.monotonic()
        self.is_playing     = True
        self.play_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.progress_bar['value'] = 0
        self.progress_label.config(text="Playing…")

    def _stop_playback(self) -> None:
        """Halt head motion playback and clear the robot's motion queue."""
        self.is_playing = False
        if self.robot:
            try:
                self.robot.ClearMotion()
            except Exception:
                pass
        self.play_btn.config(state=tk.NORMAL if self.playback_data else tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.progress_label.config(text="Stopped")

    # ─────────────────────────────────────────
    #  TRF Cartesian jog
    # ─────────────────────────────────────────

    def _send_trf_step(self, axis: str, sign: float) -> None:
        """Send a single incremental TRF move along *axis* in the given *sign* direction."""
        if not self.robot:
            return
        lin = (self.trf_lin_step_var.get() if self.trf_lin_step_var else 1.0) * sign
        ang = (self.trf_ang_step_var.get() if self.trf_ang_step_var else 1.0) * sign
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
        """Send an immediate step then schedule repeating steps at 5 Hz."""
        self._send_trf_step(axis, sign)
        self._trf_after_id = self.root.after(400, self._repeat_trf_jog, axis, sign)

    def _repeat_trf_jog(self, axis: str, sign: float) -> None:
        """Called repeatedly while a jog button is held down."""
        self._send_trf_step(axis, sign)
        self._trf_after_id = self.root.after(200, self._repeat_trf_jog, axis, sign)

    def _stop_trf_jog(self) -> None:
        """Cancel the scheduled repeating TRF jog."""
        if self._trf_after_id:
            self.root.after_cancel(self._trf_after_id)
            self._trf_after_id = None

    def _set_trf_home(self) -> None:
        """Save the current Cartesian offset as the home position."""
        self._home_cart_offset = list(self._cart_pos)
        print(f"[home] Saved at offset {self._home_cart_offset}")

    def _go_trf_home(self) -> None:
        """Move back to the saved home position via a single TRF relative move."""
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
    #  Joint jog
    # ─────────────────────────────────────────

    def _apply_vel_limit(self) -> None:
        """Push the current velocity slider value to the robot."""
        if not self.robot or not self.jog_vel_var:
            return
        try:
            self.robot.SetJointVelLimit(self.jog_vel_var.get())
        except Exception:
            pass

    def _schedule_jog(self) -> None:
        """Debounce slider changes: send the joint move 300 ms after the last change."""
        if self._jog_after_id:
            self.root.after_cancel(self._jog_after_id)
        self._jog_after_id = self.root.after(300, self._jog_joints)

    def _jog_joints(self) -> None:
        """Send a MoveJoints command for the current slider angles."""
        if not self.robot or not self.joint_vars:
            return
        angles = [v.get() for v in self.joint_vars]
        try:
            self.robot.ClearMotion()
        except Exception as exc:
            print(f"[jog] ClearMotion failed: {exc}")
        try:
            self.robot.SetJointVelLimit(self.jog_vel_var.get())
            self.robot.MoveJoints(*angles)
            print(f"[jog] MoveJoints → {angles}")
        except Exception as exc:
            print(f"[jog] MoveJoints failed: {exc}")

    # ─────────────────────────────────────────
    #  Robot error recovery
    # ─────────────────────────────────────────

    def _reset_robot_error(self) -> None:
        """Clear any active robot error and resume motion."""
        if not self.robot:
            return
        try:
            self.robot.ResetError()
            self.robot.ResumeMotion()
            print("[robot] Error reset.")
        except Exception as exc:
            print(f"[robot] Reset failed: {exc}")

    # ─────────────────────────────────────────
    #  Control loop
    # ─────────────────────────────────────────

    def _tick(self) -> None:
        """Main loop called every 1/LOOP_HZ seconds via root.after."""
        if params.QUIT_FLAG:
            self._on_close()
            return

        try:
            step = self.step_var.get()
        except tk.TclError:
            step = params.STEP_SIZE

        if self.use_eye:
            if self._gimbal_resetting:
                self._tick_gimbal_reset(step)
            else:
                self._tick_gimbal_wasd(step)
                self._tick_eye_profile()

        if self.use_head:
            self._tick_head_playback()

        self.root.after(int(1000 / params.LOOP_HZ), self._tick)

    def _tick_gimbal_wasd(self, step: int) -> None:
        """Process WASD key state and write updated encoder positions."""
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
        """Move each motor one step toward EYE_CENTER; clear flag when both arrive."""
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
        """Advance eye-profile playback or rewind by one tick."""
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
                self._eye_progress_bar['value'] = pct
                self._eye_progress_lbl.config(
                    text=f"Playing…  {self._eye_play_idx}/{n_total}", fg=_FG)

        elif self._eye_rewinding:
            if self._eye_rewind_idx < len(self._eye_rewind_steps):
                m1, m2 = self._eye_rewind_steps[self._eye_rewind_idx]
                write_position(self.port_handler, self.packet_handler, params.DXL_1, m1)
                write_position(self.port_handler, self.packet_handler, params.DXL_2, m2)
                self.pos1, self.pos2 = m1, m2
                self._eye_rewind_idx += 1
                pct = 50 + int(50 * self._eye_rewind_idx / len(self._eye_rewind_steps))
                self._eye_progress_bar['value'] = pct
            else:
                self._eye_playing   = False
                self._eye_rewinding = False
                self._eye_progress_bar['value'] = 100
                self._eye_progress_lbl.config(text="Done ✓", fg=_GREEN)
                self._bells_btn.config(state=tk.NORMAL)
                self._saccadic_btn.config(state=tk.NORMAL)

    def _tick_head_playback(self) -> None:
        """Advance head motion playback by dispatching all due frames this tick."""
        if not self.is_playing or not self.robot:
            return

        elapsed_ms = (time.monotonic() - self.playback_start) * 1000.0
        n_total    = len(self.playback_data)

        while self.playback_idx < n_total:
            t_ms, pitch, roll = self.playback_data[self.playback_idx]
            if t_ms > elapsed_ms:
                break

            d_pitch = pitch - self.prev_pitch
            d_roll  = roll  - self.prev_roll

            # Skip outlier jumps (sensor glitches / discontinuities > 5°)
            if abs(d_pitch) < 5.0 and abs(d_roll) < 5.0:
                if abs(d_pitch) > 0.01 or abs(d_roll) > 0.01:
                    self.robot.MoveLinRelTrf(0, 0, 0, d_pitch, 0, d_roll)

            self.prev_pitch = pitch
            self.prev_roll  = roll
            self.playback_idx += 1

        if self.playback_idx >= n_total:
            self._stop_playback()
            self.progress_bar['value'] = 100
            self.progress_label.config(text="Done")
        else:
            pct = int(100 * self.playback_idx / n_total)
            self.progress_bar['value'] = pct
            self.progress_label.config(text=f"{elapsed_ms / 1000:.1f} s  ({pct}%)")

    # ─────────────────────────────────────────
    #  Shutdown
    # ─────────────────────────────────────────

    def _on_close(self) -> None:
        """Stop all motion, disconnect hardware, and destroy the window."""
        self._eye_playing   = False
        self._eye_rewinding = False
        if self.is_playing:
            self._stop_playback()
        self._stop_trf_jog()
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
        self.root.destroy()
