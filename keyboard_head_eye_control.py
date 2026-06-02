import argparse
import atexit
import tkinter as tk
import Parameter as params
import mecademicpy.robot as mdr
from dynamixel_sdk import PortHandler, PacketHandler
from pynput import keyboard
from Utils import *

# ─────────────────────────────────────────────
#  KEY STATE
#  WASD      → eye gimbal (Dynamixel)
#  Arrow keys → head      (Meca500)
# ─────────────────────────────────────────────
gimbal_keys    = {'up': False, 'down': False, 'left': False, 'right': False}
head_keys      = {'up': False, 'down': False, 'left': False, 'right': False}
quit_flag      = False
robot_instance = None   # used by on_release to clear motion


def on_press(key):
    global quit_flag
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if   char == 'w':               gimbal_keys['up']    = True
        elif char == 's':               gimbal_keys['down']  = True
        elif char == 'a':               gimbal_keys['left']  = True
        elif char == 'd':               gimbal_keys['right'] = True
        elif char == 'q':
            quit_flag = True
            return False
        elif key == keyboard.Key.up:    head_keys['up']    = True
        elif key == keyboard.Key.down:  head_keys['down']  = True
        elif key == keyboard.Key.left:  head_keys['left']  = True
        elif key == keyboard.Key.right: head_keys['right'] = True
        elif key == keyboard.Key.esc:
            quit_flag = True
            return False

    except AttributeError:
        pass


def on_release(key):
    global robot_instance
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if   char == 'w':               gimbal_keys['up']    = False
        elif char == 's':               gimbal_keys['down']  = False
        elif char == 'a':               gimbal_keys['left']  = False
        elif char == 'd':               gimbal_keys['right'] = False
        elif key == keyboard.Key.up:    head_keys['up']    = False
        elif key == keyboard.Key.down:  head_keys['down']  = False
        elif key == keyboard.Key.left:  head_keys['left']  = False
        elif key == keyboard.Key.right: head_keys['right'] = False

        # Clear the robot's motion queue once all head keys are released
        # so the head stops promptly rather than draining queued moves.
        if robot_instance and not any(head_keys.values()):
            try:
                robot_instance.ClearMotion()
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

        self._build_ui()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    def _build_ui(self):
        self.root.title("Patient Motion Control Panel")
        self.root.attributes('-topmost', True)
        self.root.resizable(False, False)

        pad = {'padx': 10, 'pady': 4}

        tk.Label(self.root, text="STEP_SIZE  (counts / tick)").pack(**pad)

        # Slider + spinbox share one IntVar — changes to either update both
        self.step_var = tk.IntVar(value=params.STEP_SIZE)

        tk.Scale(self.root, from_=1, to=90, orient=tk.HORIZONTAL,
                 variable=self.step_var, showvalue=False,
                 length=260).pack(**pad)

        tk.Spinbox(self.root, from_=1, to=90, textvariable=self.step_var,
                   width=6, font=('Helvetica', 13)).pack(**pad)

        # Velocity preview — create label before adding trace so it exists when trace fires
        self.vel_label = tk.Label(self.root, text="", font=('Helvetica', 13))
        self.vel_label.pack(**pad)
        self._update_vel()
        self.step_var.trace_add('write', lambda *_: self._update_vel())

        tk.Frame(self.root, height=1, bg='grey').pack(fill='x', padx=10, pady=6)

        # Key bindings reminder
        if self.use_eye:
            tk.Label(self.root, text="W / S   Eye Superior / Inferior").pack(**pad)
            tk.Label(self.root, text="A / D   Eye Temporal / Nasal").pack(**pad)
        if self.use_head:
            tk.Label(self.root, text="↑ / ↓   Head Superior / Inferior").pack(**pad)
            tk.Label(self.root, text="← / →   Head Temporal / Nasal").pack(**pad)
        tk.Label(self.root, text="Q / ESC   Quit", fg='grey').pack(**pad)

    def _update_vel(self):
        try:
            value = self.step_var.get()
        except tk.TclError:
            return
        counts_per_sec = value * params.LOOP_HZ
        deg_per_sec    = counts_per_sec * 360 / params.COUNTS_PER_REV
        rpm            = counts_per_sec / params.COUNTS_PER_REV * 60
        self.vel_label.config(text=f"{deg_per_sec:.1f} °/s   {rpm:.1f} RPM")

    def _tick(self):
        if quit_flag:
            self._on_close()
            return

        try:
            step = self.step_var.get()
        except tk.TclError:
            step = params.STEP_SIZE

        # ── Eye gimbal: WASD ──────────────────────
        if self.use_eye:
            if gimbal_keys['up']:
                new1 = clamp(self.pos1 + step, params.JOINT_MIN_1, params.JOINT_MAX_1)
                if new1 != self.pos1:
                    self.pos1 = new1
                    write_position(self.portHandler, self.packetHandler, params.DXL_1, self.pos1)
            elif gimbal_keys['down']:
                new1 = clamp(self.pos1 - step, params.JOINT_MIN_1, params.JOINT_MAX_1)
                if new1 != self.pos1:
                    self.pos1 = new1
                    write_position(self.portHandler, self.packetHandler, params.DXL_1, self.pos1)

            if gimbal_keys['left']:
                new2 = clamp(self.pos2 + step, params.JOINT_MIN_2, params.JOINT_MAX_2)
                if new2 != self.pos2:
                    self.pos2 = new2
                    write_position(self.portHandler, self.packetHandler, params.DXL_2, self.pos2)
            elif gimbal_keys['right']:
                new2 = clamp(self.pos2 - step, params.JOINT_MIN_2, params.JOINT_MAX_2)
                if new2 != self.pos2:
                    self.pos2 = new2
                    write_position(self.portHandler, self.packetHandler, params.DXL_2, self.pos2)

        # ── Head: arrow keys ──────────────────────
        if self.use_head:
            if head_keys['up']:
                self.robot.MoveLinRelTrf(0, 0, 0,  params.HEAD_STEP_DEG, 0, 0)
            elif head_keys['down']:
                self.robot.MoveLinRelTrf(0, 0, 0, -params.HEAD_STEP_DEG, 0, 0)

            if head_keys['left']:
                self.robot.MoveLinRelTrf(0, 0, 0, 0, 0, -params.HEAD_STEP_DEG)
            elif head_keys['right']:
                self.robot.MoveLinRelTrf(0, 0, 0, 0, 0,  params.HEAD_STEP_DEG)

        self.root.after(int(1000 / params.LOOP_HZ), self._tick)

    def _on_close(self):
        global robot_instance

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

        robot_instance = None
        enable_echo()
        self.root.destroy()


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────

def main():
    global robot_instance

    parser = argparse.ArgumentParser(description='Keyboard control for head (Meca500) and/or eye gimbal (Dynamixel)')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--head', action='store_true', help='Control head only (Meca500, arrow keys)')
    group.add_argument('--eye',  action='store_true', help='Control eye gimbal only (Dynamixel, WASD)')
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

        robot_instance = robot

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
