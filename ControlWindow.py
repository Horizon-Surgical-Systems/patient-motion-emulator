import time
import csv
import os
import tkinter as tk
from tkinter import filedialog, ttk
import Parameter as params
from Utils import clamp, write_position, disable_motor, enable_echo

# ── UI palette ────────────────────────────────
_BG    = "#1e1e2e"
_CARD  = "#2a2a3e"
_ACCT  = "#7c6af5"
_FG    = "#cdd6f4"
_DIM   = "#6c7086"
_SEP   = "#313244"
_GREEN = "#a6e3a1"
_RED   = "#f38ba8"

# Light button colours — macOS reliably shows black text on these
_BTN_PURPLE = "#c4bbfc"   # accent actions  (Browse, Set Home, Reset Error)
_BTN_GREEN  = "#a6e3a1"   # positive actions (Play, Go Home)
_BTN_RED    = "#f38ba8"   # destructive      (Stop)
_BTN_GRAY   = "#9399b2"   # jog arrows


class ControlWindow:
    def __init__(self, root, use_head, use_eye, robot, portHandler, packetHandler, listener):
        self.root          = root
        self.use_head      = use_head
        self.use_eye       = use_eye
        self.robot         = robot
        self.portHandler   = portHandler
        self.packetHandler = packetHandler
        self.listener      = listener
        self.pos1          = 2047
        self.pos2          = 2047

        # Playback state
        self.playback_data  = []
        self.playback_idx   = 0
        self.is_playing     = False
        self.playback_start = 0.0
        self.prev_pitch     = 0.0
        self.prev_roll      = 0.0

        # Joint jog state
        self.joint_vars    = []
        self.jog_vel_var   = None
        self._jog_after_id = None

        self.step_var = tk.IntVar(value=params.STEP_SIZE)

        # TRF Cartesian jog state
        self.trf_lin_step_var   = None          # DoubleVar created in _build_ui
        self.trf_ang_step_var   = None
        self._trf_after_id      = None
        self._cart_pos          = [0.0] * 6     # cumulative (x,y,z,ux,uy,uz) from init
        self._home_cart_offset  = [0.0] * 6     # saved home offset

        self._build_ui()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    # ── helpers ───────────────────────────────

    @staticmethod
    def _lighten(hex_color, amount=25):
        r = min(255, int(hex_color[1:3], 16) + amount)
        g = min(255, int(hex_color[3:5], 16) + amount)
        b = min(255, int(hex_color[5:7], 16) + amount)
        return f'#{r:02x}{g:02x}{b:02x}'

    def _card(self, parent=None, fill='x', expand=False):
        if parent is None:
            parent = self.root
        f = tk.Frame(parent, bg=_CARD, padx=14, pady=10,
                     highlightbackground=_SEP, highlightthickness=1)
        f.pack(fill=fill, expand=expand, padx=12, pady=4)
        return f

    def _lbl(self, parent, text, size=9, bold=False, color=None):
        font = ('Arial', size, 'bold') if bold else ('Arial', size)
        return tk.Label(parent, text=text, bg=parent['bg'],
                        fg=color or _FG, font=font)

    def _sep(self, parent):
        tk.Frame(parent, bg=_SEP, height=1).pack(fill='x', pady=(4, 6))

    def _btn(self, parent, text, command, color=_BTN_PURPLE, fg='black', width=10):
        b = tk.Button(parent, text=text, command=command,
                      bg=color, fg=fg, activebackground=self._lighten(color),
                      activeforeground=fg, disabledforeground='#888888',
                      font=('Arial', 9, 'bold'),
                      relief='flat', bd=0, padx=10, pady=5,
                      cursor='hand2', width=width)
        b.bind('<Enter>', lambda _, c=color: b.config(bg=self._lighten(c)))
        b.bind('<Leave>', lambda _, c=color: b.config(bg=c))
        return b

    def _scale(self, parent, **kw):
        return tk.Scale(parent, bg=_CARD, fg=_FG, troughcolor=_SEP,
                        activebackground=_ACCT, highlightthickness=0,
                        bd=0, **kw)

    def _jog_btn(self, parent, label, axis, sign, width=3):
        """Labeled jog button that sends incremental TRF moves while held."""
        b = tk.Button(parent, text=label, width=width,
                      bg=_BTN_GRAY, fg='black', activebackground=self._lighten(_BTN_GRAY),
                      activeforeground='black', disabledforeground='#888888',
                      font=('Arial', 11, 'bold'),
                      relief='flat', bd=0, padx=6, pady=6, cursor='hand2')
        b.bind('<ButtonPress-1>',   lambda _: self._start_trf_jog(axis, sign))
        b.bind('<ButtonRelease-1>', lambda _: self._stop_trf_jog())
        b.bind('<Enter>', lambda _: b.config(bg=self._lighten(_BTN_GRAY)))
        b.bind('<Leave>', lambda _: b.config(bg=_BTN_GRAY))
        return b

    # ── UI ────────────────────────────────────

    def _build_ui(self):
        self.root.title("Patient Motion Control")
        self.root.configure(bg=_BG)
        self.root.attributes('-topmost', True)
        self.root.resizable(False, False)

        # ── Header ────────────────────────────
        hdr = tk.Frame(self.root, bg=_ACCT, pady=12)
        hdr.pack(fill='x')
        self._lbl(hdr, "Patient Motion Control", 13, bold=True,
                  color='#ffffff').pack()
        parts = []
        if self.use_head: parts.append("Head  (Meca500)")
        if self.use_eye:  parts.append("Eye  (Dynamixel)")
        self._lbl(hdr, "  +  ".join(parts), 9,
                  color='#ddd6fe').pack(pady=(2, 0))

        # ── Three-column body ─────────────────
        body = tk.Frame(self.root, bg=_BG)
        body.pack(fill='both', expand=True)

        left      = tk.Frame(body, bg=_BG)
        left.pack(side=tk.LEFT, fill='both', expand=True)

        right     = tk.Frame(body, bg=_BG)
        right.pack(side=tk.LEFT, fill='both', expand=True)

        far_right = tk.Frame(body, bg=_BG)
        far_right.pack(side=tk.LEFT, fill='both', expand=True)

        # ── Gimbal speed (left) ───────────────
        if self.use_eye:
            card = self._card(left)
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

        # ── Key bindings (left) ───────────────
        card = self._card(left)
        self._lbl(card, "Key Bindings", 10, bold=True).pack(anchor='w')
        self._sep(card)

        bindings = []
        if self.use_eye:
            bindings += [("W / S", "Eye  Superior / Inferior", _ACCT),
                         ("A / D", "Eye  Temporal / Nasal",    _ACCT)]
        bindings.append(("Q / ESC", "Quit", _RED))

        for key, action, color in bindings:
            row = tk.Frame(card, bg=_CARD)
            row.pack(fill='x', pady=2)
            self._lbl(row, key, 9, bold=True, color=color).pack(side=tk.LEFT)
            self._lbl(row, action, 9, color=_FG).pack(side=tk.RIGHT)

        # ── Motion playback (left) ────────────
        if self.use_head:
            card = self._card(left)
            self._lbl(card, "Motion Playback", 10, bold=True).pack(anchor='w')
            self._sep(card)

            file_row = tk.Frame(card, bg=_CARD)
            file_row.pack(fill='x', pady=(0, 6))
            self.file_label = self._lbl(file_row, "No file loaded", 9, color=_DIM)
            self.file_label.pack(side=tk.LEFT, anchor='w')
            self._btn(file_row, "Browse…", self._browse_file,
                      width=8).pack(side=tk.RIGHT)

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
            style.configure("Accent.Horizontal.TProgressbar",
                            troughcolor=_SEP, background=_ACCT,
                            borderwidth=0, lightcolor=_ACCT, darkcolor=_ACCT)
            self.progress_bar = ttk.Progressbar(card, maximum=100, value=0,
                                                style="Accent.Horizontal.TProgressbar")
            self.progress_bar.pack(fill='x', pady=(0, 4))

            self.progress_label = self._lbl(card, "", 9, color=_DIM)
            self.progress_label.pack(anchor='w')

            # ── Joint Jog (right) ─────────────
            card = self._card(right, fill='both', expand=True)
            self._lbl(card, "Joint Jog", 10, bold=True).pack(anchor='w')
            self._sep(card)

            vel_row = tk.Frame(card, bg=_CARD)
            vel_row.pack(fill='x', pady=(0, 4))
            self._lbl(vel_row, "Velocity", 9, color=_DIM).pack(side=tk.LEFT,
                                                                padx=(0, 8))
            self.jog_vel_var = tk.DoubleVar(value=params.MAX_JOINT_VEL_PERCENTAGE)
            self._scale(vel_row, from_=params.JOINT_VEL_MIN, to=params.JOINT_VEL_MAX,
                        resolution=0.1, orient=tk.HORIZONTAL,
                        variable=self.jog_vel_var, length=160).pack(side=tk.LEFT)
            self._lbl(vel_row, "%", 9, color=_DIM).pack(side=tk.LEFT, padx=4)
            self.jog_vel_var.trace_add('write', lambda *_: self._apply_vel_limit())

            joint_labels = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
            for i, (jlbl, angle) in enumerate(
                    zip(joint_labels, params.ROBOT_HEAD_INIT_POSE)):
                lo, hi = params.MECA500_JOINT_LIMITS[i]
                row = tk.Frame(card, bg=_CARD)
                row.pack(fill='x', pady=3)
                self._lbl(row, jlbl, 9, bold=True, color=_ACCT).pack(
                    side=tk.LEFT, padx=(0, 6))
                var = tk.DoubleVar(value=angle)
                self._scale(row, from_=lo, to=hi, resolution=0.5,
                            orient=tk.HORIZONTAL, variable=var,
                            length=200).pack(side=tk.LEFT)
                var.trace_add('write', lambda *_: self._schedule_jog())
                self.joint_vars.append(var)

            # ── TRF Cartesian Jog (far_right) ────
            card = self._card(far_right, fill='both', expand=True)
            self._lbl(card, "TRF Cartesian Jog", 10, bold=True).pack(anchor='w')
            self._sep(card)

            # Step size inputs
            step_grid = tk.Frame(card, bg=_CARD)
            step_grid.pack(fill='x', pady=(0, 8))

            self.trf_lin_step_var = tk.DoubleVar(value=1.0)
            self.trf_ang_step_var = tk.DoubleVar(value=1.0)

            for row_label, var, unit in [
                ("Linear step",  self.trf_lin_step_var, "mm"),
                ("Angular step", self.trf_ang_step_var, "°"),
            ]:
                r = tk.Frame(step_grid, bg=_CARD)
                r.pack(fill='x', pady=2)
                self._lbl(r, row_label, 9, color=_DIM).pack(side=tk.LEFT)
                tk.Spinbox(r, from_=0.1, to=50.0, increment=0.5,
                           textvariable=var, width=5, font=('Arial', 9),
                           bg=_BG, fg=_FG, buttonbackground=_SEP,
                           insertbackground=_FG, relief='flat').pack(side=tk.RIGHT)
                self._lbl(r, unit, 9, color=_DIM).pack(side=tk.RIGHT, padx=(0, 4))

            # Translation axes  (neg_label, axis_name, pos_label)
            self._lbl(card, "Translation", 9, bold=True, color=_DIM).pack(anchor='w', pady=(4, 2))
            for axis, neg_lbl, pos_lbl in [
                ('x',  'Nasal',    'Temporal'),
                ('y',  'Down',     'Up'),
                ('z',  'Inferior', 'Superior'),
            ]:
                r = tk.Frame(card, bg=_CARD)
                r.pack(fill='x', pady=3)
                self._jog_btn(r, f"- {neg_lbl}", axis, -1, width=10).pack(side=tk.LEFT)
                self._lbl(r, axis.upper(), 10, bold=True, color=_ACCT).pack(side=tk.LEFT, expand=True)
                self._jog_btn(r, f"{pos_lbl} +", axis, +1, width=10).pack(side=tk.RIGHT)

            # Rotation axes  (neg_label, axis_name, pos_label)
            self._lbl(card, "Rotation", 9, bold=True, color=_DIM).pack(anchor='w', pady=(8, 2))
            for axis, neg_lbl, pos_lbl in [
                ('ux', 'Inferior', 'Superior'),
                ('uy', 'Left',     'Right'),
                ('uz', 'Temporal', 'Nasal'),
            ]:
                r = tk.Frame(card, bg=_CARD)
                r.pack(fill='x', pady=3)
                self._jog_btn(r, f"- {neg_lbl}", axis, -1, width=10).pack(side=tk.LEFT)
                self._lbl(r, axis, 10, bold=True, color=_ACCT).pack(side=tk.LEFT, expand=True)
                self._jog_btn(r, f"{pos_lbl} +", axis, +1, width=10).pack(side=tk.RIGHT)

            tk.Frame(card, bg=_SEP, height=1).pack(fill='x', pady=(10, 6))

            home_row = tk.Frame(card, bg=_CARD)
            home_row.pack(fill='x', pady=(0, 4))
            self._btn(home_row, "Set Home", self._set_trf_home,
                      color=_BTN_PURPLE, fg='black', width=10).pack(side=tk.LEFT, padx=(0, 6))
            self._btn(home_row, "Go Home", self._go_trf_home,
                      color=_BTN_GREEN, fg='black', width=10).pack(side=tk.LEFT)

            self._btn(card, "Reset Error", self._reset_robot_error,
                      color=_BTN_RED, fg='black', width=12).pack(anchor='w')

        tk.Frame(self.root, bg=_BG, height=8).pack()

    def _update_vel(self):
        if not hasattr(self, 'vel_label'):
            return
        try:
            value = self.step_var.get()
        except tk.TclError:
            return
        counts_per_sec = value * params.LOOP_HZ
        deg_per_sec    = counts_per_sec * 360 / params.COUNTS_PER_REV
        rpm            = counts_per_sec / params.COUNTS_PER_REV * 60
        self.vel_label.config(text=f"{deg_per_sec:.1f} °/s   ·   {rpm:.1f} RPM")

    # ── Motion file I/O ───────────────────────

    def _browse_file(self):
        motion_dir = os.path.join(os.path.dirname(__file__), params.MOTION_DATA_FOLDER)
        path = filedialog.askopenfilename(
            initialdir=motion_dir,
            title="Select motion data file",
            filetypes=[("Text/CSV files", "*.txt *.csv"), ("All files", "*.*")]
        )
        if path:
            self._load_file(path)

    def _load_file(self, path):
        data = []
        try:
            with open(path, newline='') as f:
                reader = csv.reader(f)
                next(reader)    # skip header: time,gyro0..2,act0..2,pitch,roll
                for row in reader:
                    if len(row) < 9:
                        continue
                    t     = int(float(row[0]))
                    pitch = float(row[7])
                    roll  = float(row[8])
                    data.append((t, pitch, roll))
        except Exception as e:
            self.file_label.config(text=f"Error loading file: {e}", fg=_RED)
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
        self.info_label.config(
            text=f"{len(data)} samples  ·  {duration_s:.1f} s", fg=_DIM)
        self.play_btn.config(state=tk.NORMAL)
        self.progress_label.config(text="")
        self.progress_bar['value'] = 0

    # ── Playback control ──────────────────────

    def _start_playback(self):
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

    def _stop_playback(self):
        self.is_playing = False
        if self.robot:
            try:
                self.robot.ClearMotion()
            except Exception:
                pass
        if self.use_head:
            self.play_btn.config(state=tk.NORMAL if self.playback_data else tk.DISABLED)
            self.stop_btn.config(state=tk.DISABLED)
            self.progress_label.config(text="Stopped")

    # ── Error recovery ────────────────────────

    def _reset_robot_error(self):
        if not self.robot:
            return
        try:
            self.robot.ResetError()
            self.robot.ResumeMotion()
            print("[robot] Error reset.")
        except Exception as e:
            print(f"[head] Reset failed: {e}")

    # ── TRF Cartesian jog ────────────────────

    _AXIS_IDX = {'x': 0, 'y': 1, 'z': 2, 'ux': 3, 'uy': 4, 'uz': 5}

    def _send_trf_step(self, axis: str, sign: float):
        if not self.robot:
            return
        lin = (self.trf_lin_step_var.get() if self.trf_lin_step_var else 1.0) * sign
        ang = (self.trf_ang_step_var.get() if self.trf_ang_step_var else 1.0) * sign
        deltas = {'x':  (lin, 0, 0, 0, 0, 0),
                  'y':  (0, lin, 0, 0, 0, 0),
                  'z':  (0, 0, lin, 0, 0, 0),
                  'ux': (0, 0, 0, ang, 0, 0),
                  'uy': (0, 0, 0, 0, ang, 0),
                  'uz': (0, 0, 0, 0, 0, ang)}
        try:
            self.robot.MoveLinRelTrf(*deltas[axis])
            self._cart_pos[self._AXIS_IDX[axis]] += lin if axis in ('x', 'y', 'z') else ang
        except Exception as e:
            print(f"[trf] {e}")

    def _start_trf_jog(self, axis: str, sign: float):
        self._send_trf_step(axis, sign)
        # 400 ms initial delay, then repeat at 200 ms (5 Hz)
        self._trf_after_id = self.root.after(400, self._repeat_trf_jog, axis, sign)

    def _repeat_trf_jog(self, axis: str, sign: float):
        self._send_trf_step(axis, sign)
        self._trf_after_id = self.root.after(200, self._repeat_trf_jog, axis, sign)

    def _stop_trf_jog(self):
        if self._trf_after_id:
            self.root.after_cancel(self._trf_after_id)
            self._trf_after_id = None

    def _set_trf_home(self):
        self._home_cart_offset = list(self._cart_pos)
        print(f"[home] Saved at offset {self._home_cart_offset}")

    def _go_trf_home(self):
        if not self.robot:
            return
        delta = [h - c for h, c in zip(self._home_cart_offset, self._cart_pos)]
        if all(abs(d) < 0.001 for d in delta):
            return
        try:
            self.robot.MoveLinRelTrf(*delta)
            self._cart_pos = list(self._home_cart_offset)
        except Exception as e:
            print(f"[home] {e}")

    # ── Joint jog ─────────────────────────────

    def _apply_vel_limit(self):
        if not self.robot or not self.jog_vel_var:
            return
        try:
            self.robot.SetJointVelLimit(self.jog_vel_var.get())
        except Exception:
            pass

    def _schedule_jog(self):
        if self._jog_after_id:
            self.root.after_cancel(self._jog_after_id)
        self._jog_after_id = self.root.after(300, self._jog_joints)

    def _jog_joints(self):
        if not self.robot or not self.joint_vars:
            return
        angles = [v.get() for v in self.joint_vars]
        try:
            self.robot.ClearMotion()
        except Exception as e:
            print(f"[jog] ClearMotion failed: {e}")
        try:
            self.robot.SetJointVelLimit(self.jog_vel_var.get())
            self.robot.MoveJoints(*angles)
            print(f"[jog] MoveJoints sent: {angles}")
        except Exception as e:
            print(f"[jog] MoveJoints failed: {e}")

    # ── Control loop ──────────────────────────

    def _tick(self):
        if params.QUIT_FLAG:
            self._on_close()
            return

        try:
            step = self.step_var.get()
        except tk.TclError:
            step = params.STEP_SIZE

        # ── Eye gimbal: WASD ──────────────────
        if self.use_eye:
            if params.GIMBAL_KEYS['up']:
                new1 = clamp(self.pos1 + step, params.JOINT_MIN_1, params.JOINT_MAX_1)
                if new1 != self.pos1:
                    self.pos1 = new1
                    write_position(self.portHandler, self.packetHandler, params.DXL_1, self.pos1)
            elif params.GIMBAL_KEYS['down']:
                new1 = clamp(self.pos1 - step, params.JOINT_MIN_1, params.JOINT_MAX_1)
                if new1 != self.pos1:
                    self.pos1 = new1
                    write_position(self.portHandler, self.packetHandler, params.DXL_1, self.pos1)

            if params.GIMBAL_KEYS['left']:
                new2 = clamp(self.pos2 + step, params.JOINT_MIN_2, params.JOINT_MAX_2)
                if new2 != self.pos2:
                    self.pos2 = new2
                    write_position(self.portHandler, self.packetHandler, params.DXL_2, self.pos2)
            elif params.GIMBAL_KEYS['right']:
                new2 = clamp(self.pos2 - step, params.JOINT_MIN_2, params.JOINT_MAX_2)
                if new2 != self.pos2:
                    self.pos2 = new2
                    write_position(self.portHandler, self.packetHandler, params.DXL_2, self.pos2)

        # ── Motion playback ───────────────────
        if self.use_head and self.is_playing and self.robot:
            elapsed_ms = (time.monotonic() - self.playback_start) * 1000.0

            while self.playback_idx < len(self.playback_data):
                t_ms, pitch, roll = self.playback_data[self.playback_idx]
                if t_ms > elapsed_ms:
                    break

                delta_pitch = pitch - self.prev_pitch
                delta_roll  = roll  - self.prev_roll

                # Skip outlier jumps (sensor glitches / discontinuities)
                if abs(delta_pitch) < 5.0 and abs(delta_roll) < 5.0:
                    if abs(delta_pitch) > 0.01 or abs(delta_roll) > 0.01:
                        self.robot.MoveLinRelTrf(0, 0, 0, delta_pitch, 0, delta_roll)

                self.prev_pitch = pitch
                self.prev_roll  = roll
                self.playback_idx += 1

            if self.playback_idx >= len(self.playback_data):
                self._stop_playback()
                self.progress_bar['value'] = 100
                self.progress_label.config(text="Done")
            else:
                pct       = int(100 * self.playback_idx / len(self.playback_data))
                elapsed_s = elapsed_ms / 1000.0
                self.progress_bar['value'] = pct
                self.progress_label.config(text=f"{elapsed_s:.1f} s  ({pct}%)")

        self.root.after(int(1000 / params.LOOP_HZ), self._tick)

    # ── Shutdown ──────────────────────────────

    def _on_close(self):
        if self.is_playing:
            self._stop_playback()
        self._stop_trf_jog()

        self.listener.stop()

        if self.use_eye and self.portHandler:
            disable_motor(self.portHandler, self.packetHandler, params.DXL_1)
            disable_motor(self.portHandler, self.packetHandler, params.DXL_2)
            self.portHandler.closePort()
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
