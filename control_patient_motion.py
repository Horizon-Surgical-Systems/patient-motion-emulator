import argparse
import atexit
import time
import csv
import os
import tkinter as tk
from tkinter import filedialog
import Parameter as params
import mecademicpy.robot as mdr
from dynamixel_sdk import PortHandler, PacketHandler
from pynput import keyboard
from Utils import *


def on_press(key):
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if   char == 'w':               params.GIMBAL_KEYS['up']    = True
        elif char == 's':               params.GIMBAL_KEYS['down']  = True
        elif char == 'a':               params.GIMBAL_KEYS['left']  = True
        elif char == 'd':               params.GIMBAL_KEYS['right'] = True
        elif char == 'q':
            params.QUIT_FLAG = True
            return False
        elif key == keyboard.Key.up:    params.HEAD_KEYS['up']    = True
        elif key == keyboard.Key.down:  params.HEAD_KEYS['down']  = True
        elif key == keyboard.Key.left:  params.HEAD_KEYS['left']  = True
        elif key == keyboard.Key.right: params.HEAD_KEYS['right'] = True
        elif key == keyboard.Key.esc:
            params.QUIT_FLAG = True
            return False

    except AttributeError:
        pass


def on_release(key):
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if   char == 'w':               params.GIMBAL_KEYS['up']    = False
        elif char == 's':               params.GIMBAL_KEYS['down']  = False
        elif char == 'a':               params.GIMBAL_KEYS['left']  = False
        elif char == 'd':               params.GIMBAL_KEYS['right'] = False
        elif key == keyboard.Key.up:    params.HEAD_KEYS['up']    = False
        elif key == keyboard.Key.down:  params.HEAD_KEYS['down']  = False
        elif key == keyboard.Key.left:  params.HEAD_KEYS['left']  = False
        elif key == keyboard.Key.right: params.HEAD_KEYS['right'] = False

        if params.ROBOT_INSTANCE and not any(params.HEAD_KEYS.values()):
            try:
                params.ROBOT_INSTANCE.ClearMotion()
            except Exception:
                pass

    except AttributeError:
        pass


# ─────────────────────────────────────────────
#  CONTROL WINDOW
# ─────────────────────────────────────────────

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
        self.playback_data  = []    # list of (time_ms, pitch, roll), t=0 at first sample
        self.playback_idx   = 0
        self.is_playing     = False
        self.playback_start = 0.0   # monotonic time when playback began
        self.prev_pitch     = 0.0
        self.prev_roll      = 0.0

        self._build_ui()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    # ── UI ────────────────────────────────────

    def _build_ui(self):
        self.root.title("Patient Motion Control Panel")
        self.root.attributes('-topmost', True)
        self.root.resizable(False, False)

        pad = {'padx': 10, 'pady': 4}

        # ── Gimbal speed ──────────────────────
        tk.Label(self.root, text="STEP_SIZE  (counts / tick)").pack(**pad)

        self.step_var = tk.IntVar(value=params.STEP_SIZE)

        tk.Scale(self.root, from_=1, to=90, orient=tk.HORIZONTAL,
                 variable=self.step_var, showvalue=False,
                 length=260).pack(**pad)

        tk.Spinbox(self.root, from_=1, to=90, textvariable=self.step_var,
                   width=6, font=('Helvetica', 13)).pack(**pad)

        self.vel_label = tk.Label(self.root, text="", font=('Helvetica', 13))
        self.vel_label.pack(**pad)
        self._update_vel()
        self.step_var.trace_add('write', lambda *_: self._update_vel())

        tk.Frame(self.root, height=1, bg='grey').pack(fill='x', padx=10, pady=6)

        # ── Key bindings ──────────────────────
        if self.use_eye:
            tk.Label(self.root, text="W / S   Eye Superior / Inferior").pack(**pad)
            tk.Label(self.root, text="A / D   Eye Temporal / Nasal").pack(**pad)
        if self.use_head:
            tk.Label(self.root, text="↑ / ↓   Head Superior / Inferior").pack(**pad)
            tk.Label(self.root, text="← / →   Head Temporal / Nasal").pack(**pad)
        tk.Label(self.root, text="Q / ESC   Quit", fg='grey').pack(**pad)

        # ── Motion playback (head only) ───────
        if self.use_head:
            tk.Frame(self.root, height=1, bg='grey').pack(fill='x', padx=10, pady=6)

            tk.Label(self.root, text="Motion Playback",
                     font=('Helvetica', 12, 'bold')).pack(**pad)

            self.file_label = tk.Label(self.root, text="No file loaded",
                                       fg='grey', wraplength=280)
            self.file_label.pack(**pad)

            tk.Button(self.root, text="Browse...",
                      command=self._browse_file).pack(**pad)

            self.info_label = tk.Label(self.root, text="", fg='grey')
            self.info_label.pack(**pad)

            btn_row = tk.Frame(self.root)
            btn_row.pack(**pad)
            self.play_btn = tk.Button(btn_row, text="Play", width=8,
                                      command=self._start_playback,
                                      state=tk.DISABLED)
            self.play_btn.pack(side=tk.LEFT, padx=4)
            self.stop_btn = tk.Button(btn_row, text="Stop", width=8,
                                      command=self._stop_playback,
                                      state=tk.DISABLED)
            self.stop_btn.pack(side=tk.LEFT, padx=4)

            self.progress_label = tk.Label(self.root, text="")
            self.progress_label.pack(**pad)

    def _update_vel(self):
        try:
            value = self.step_var.get()
        except tk.TclError:
            return
        counts_per_sec = value * params.LOOP_HZ
        deg_per_sec    = counts_per_sec * 360 / params.COUNTS_PER_REV
        rpm            = counts_per_sec / params.COUNTS_PER_REV * 60
        self.vel_label.config(text=f"{deg_per_sec:.1f} °/s   {rpm:.1f} RPM")

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
            self.file_label.config(text=f"Error loading file: {e}", fg='red')
            return

        if not data:
            self.file_label.config(text="No valid data found", fg='red')
            return

        # Normalize timestamps so playback starts at t=0
        t0 = data[0][0]
        self.playback_data = [(t - t0, p, r) for t, p, r in data]
        self.playback_idx  = 0
        self.is_playing    = False

        duration_s = self.playback_data[-1][0] / 1000.0
        self.file_label.config(text=os.path.basename(path), fg='black')
        self.info_label.config(
            text=f"{len(data)} samples  ·  {duration_s:.1f} s", fg='grey')
        self.play_btn.config(state=tk.NORMAL)
        self.progress_label.config(text="")

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
        self.progress_label.config(text="Playing...")

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

        # ── Head: arrow keys (disabled during playback) ──
        if self.use_head and not self.is_playing:
            if params.HEAD_KEYS['up']:
                self.robot.MoveLinRelTrf(0, 0, 0,  params.HEAD_STEP_DEG, 0, 0)
            elif params.HEAD_KEYS['down']:
                self.robot.MoveLinRelTrf(0, 0, 0, -params.HEAD_STEP_DEG, 0, 0)

            if params.HEAD_KEYS['left']:
                self.robot.MoveLinRelTrf(0, 0, 0, 0, 0, -params.HEAD_STEP_DEG)
            elif params.HEAD_KEYS['right']:
                self.robot.MoveLinRelTrf(0, 0, 0, 0, 0,  params.HEAD_STEP_DEG)

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
                self.progress_label.config(text="Done")
            else:
                pct       = int(100 * self.playback_idx / len(self.playback_data))
                elapsed_s = elapsed_ms / 1000.0
                self.progress_label.config(text=f"{elapsed_s:.1f} s  ({pct}%)")

        self.root.after(int(1000 / params.LOOP_HZ), self._tick)

    # ── Shutdown ──────────────────────────────

    def _on_close(self):
        if self.is_playing:
            self._stop_playback()

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


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Patient motion control — head (Meca500) and/or eye gimbal (Dynamixel)')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--head', action='store_true',
                       help='Control head only (Meca500, arrow keys)')
    group.add_argument('--eye',  action='store_true',
                       help='Control eye gimbal only (Dynamixel, WASD)')
    args = parser.parse_args()

    use_head = not args.eye
    use_eye  = not args.head

    robot         = None
    portHandler   = None
    packetHandler = None

    # ── Meca500 ───────────────────────────────
    if use_head:
        print("Connecting to Meca500...")
        robot = mdr.Robot()
        robot.Connect(address=params.ROBOT_IP_ADDRESS, disconnect_on_exception=False)
        robot.ActivateAndHome()
        robot.WaitHomed()
        print("Robot homed.")

        robot.SetTrf(params.HEAD_OFFSET[0], params.HEAD_OFFSET[1], params.HEAD_OFFSET[2], 0, 0, 0)
        robot.SetJointVelLimit(params.MAX_JOINT_VEL_PERCENTAGE)
        robot.MoveJoints(*params.ROBOT_HEAD_INIT_POSE)
        robot.WaitIdle(60)
        print("Robot at initial pose.")

        params.ROBOT_INSTANCE = robot

    # ── Dynamixel ─────────────────────────────
    if use_eye:
        portHandler   = PortHandler(params.PORT)
        packetHandler = PacketHandler(params.PROTOCOL)

        if not portHandler.openPort():
            print("ERROR: Failed to open port.")
            if robot:
                robot.DeactivateRobot()
                robot.Disconnect()
            return
        if not portHandler.setBaudRate(params.BAUD_RATE):
            print("ERROR: Failed to set baud rate.")
            portHandler.closePort()
            if robot:
                robot.DeactivateRobot()
                robot.Disconnect()
            return

        setup_motor(portHandler, packetHandler, params.DXL_1)
        setup_motor(portHandler, packetHandler, params.DXL_2)
        print("Eye gimbal motors ready.")

        write_position(portHandler, packetHandler, params.DXL_1, 2047)
        write_position(portHandler, packetHandler, params.DXL_2, 2047)

    atexit.register(enable_echo)
    disable_echo()

    root = tk.Tk()
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    ControlWindow(root, use_head, use_eye, robot, portHandler, packetHandler, listener)
    root.mainloop()

    print("Shutting down.")


if __name__ == '__main__':
    main()
