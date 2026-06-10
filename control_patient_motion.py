"""Entry point for patient motion control.

Connects to the Meca500 head robot and/or the Dynamixel eye gimbal, then
opens the ControlWindow GUI.  The serial port is selected automatically
based on the host OS, and can be overridden in Parameter.py.

Usage:
    python control_patient_motion.py          # head + eye (default)
    python control_patient_motion.py --head   # Meca500 only
    python control_patient_motion.py --eye    # Dynamixel gimbal only
"""

from __future__ import annotations

import argparse
import atexit
import sys
from typing import Optional

from PyQt6.QtWidgets import QApplication

import mecademicpy.robot as mdr
from dynamixel_sdk import PacketHandler, PortHandler
from pynput import keyboard

import Parameter as params
from ControlWindow import ControlWindow
from Utils import disable_echo, enable_echo, setup_motor, write_position


# ─────────────────────────────────────────────
#  Keyboard callbacks
# ─────────────────────────────────────────────

def on_press(key: keyboard.Key) -> Optional[bool]:
    """Update gimbal key state on key-down; return False to stop the listener on quit."""
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if char == 'w':
            params.GIMBAL_KEYS['up'] = True
        elif char == 's':
            params.GIMBAL_KEYS['down'] = True
        elif char == 'a':
            params.GIMBAL_KEYS['left'] = True
        elif char == 'd':
            params.GIMBAL_KEYS['right'] = True
        elif char == 'q' or key == keyboard.Key.esc:
            params.QUIT_FLAG = True
            return False

    except AttributeError:
        pass

    return None


def on_release(key: keyboard.Key) -> None:
    """Clear gimbal key state on key-up."""
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if char == 'w':
            params.GIMBAL_KEYS['up'] = False
        elif char == 's':
            params.GIMBAL_KEYS['down'] = False
        elif char == 'a':
            params.GIMBAL_KEYS['left'] = False
        elif char == 'd':
            params.GIMBAL_KEYS['right'] = False

    except AttributeError:
        pass


# ─────────────────────────────────────────────
#  Hardware initialisation helpers
# ─────────────────────────────────────────────

def _connect_robot() -> Optional[mdr.Robot]:
    """Connect, activate, home, and position the Meca500.

    Returns the robot instance on success, or None if the connection fails
    (the GUI will open without head control in that case).
    """
    print("Connecting to Meca500…")
    try:
        robot = mdr.Robot()
        robot.Connect(address=params.ROBOT_IP_ADDRESS, disconnect_on_exception=False)
        robot.ActivateRobot()
        robot.ActivateAndHome()
        robot.WaitHomed()
        print("Robot homed.")

        robot.SetTrf(*params.HEAD_OFFSET, 0, 0, 0)
        robot.SetJointVelLimit(params.MAX_JOINT_VEL_PERCENTAGE)
        robot.MoveJoints(*params.ROBOT_HEAD_INIT_POSE)
        robot.WaitIdle(60)
        print("Robot at initial pose.")

        params.ROBOT_INSTANCE = robot
        return robot

    except Exception as exc:
        print(f"WARNING: Could not connect to Meca500 — {exc}")
        print("Opening UI without head control.")
        return None


def _connect_gimbal() -> tuple[Optional[PortHandler], Optional[PacketHandler]]:
    """Open the serial port and initialise both Dynamixel motors.

    Returns (portHandler, packetHandler) on success, or (None, None) on failure.
    """
    if sys.platform == 'darwin':
        params.PORT = '/dev/cu.usbmodem101'
    elif sys.platform.startswith('linux'):
        params.PORT = '/dev/ttyACM0'
    elif sys.platform == 'win32':
        params.PORT = 'COM3'
    print(f"Serial port: {params.PORT}")

    port   = PortHandler(params.PORT)
    packet = PacketHandler(params.PROTOCOL)

    if not port.openPort():
        print("ERROR: Failed to open port.")
        return None, None
    if not port.setBaudRate(params.BAUD_RATE):
        print("ERROR: Failed to set baud rate.")
        port.closePort()
        return None, None

    setup_motor(port, packet, params.DXL_1)
    setup_motor(port, packet, params.DXL_2)
    write_position(port, packet, params.DXL_1, params.EYE_CENTER)
    write_position(port, packet, params.DXL_2, params.EYE_CENTER)
    print("Eye gimbal motors ready.")

    return port, packet


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────

def main() -> None:
    """Parse arguments, initialise hardware, and launch the control GUI."""
    parser = argparse.ArgumentParser(
        description='Patient motion control — head (Meca500) and/or eye gimbal (Dynamixel)')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--head', action='store_true',
                       help='Control head only (Meca500)')
    group.add_argument('--eye',  action='store_true',
                       help='Control eye gimbal only (Dynamixel)')
    args = parser.parse_args()

    use_head = not args.eye
    use_eye  = not args.head

    robot         = None
    port_handler  = None
    packet_handler = None

    if use_head:
        robot = _connect_robot()

    if use_eye:
        port_handler, packet_handler = _connect_gimbal()
        if port_handler is None:
            if robot:
                robot.DeactivateRobot()
                robot.Disconnect()
            return

    atexit.register(enable_echo)
    disable_echo()

    app = QApplication(sys.argv)
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    window = ControlWindow(use_head, use_eye, robot, port_handler, packet_handler, listener)
    window.show()
    sys.exit(app.exec())

    print("Shutting down.")


if __name__ == '__main__':
    main()
