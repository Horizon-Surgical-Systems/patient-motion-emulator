import argparse
import atexit
import tkinter as tk
import Parameter as params
import mecademicpy.robot as mdr
from dynamixel_sdk import PortHandler, PacketHandler
from pynput import keyboard
from Utils import enable_echo, disable_echo, setup_motor, write_position
from ControlWindow import ControlWindow


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

    except AttributeError:
        pass


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
        try:
            robot = mdr.Robot()
            robot.Connect(address=params.ROBOT_IP_ADDRESS, disconnect_on_exception=False)
            robot.ActivateRobot()
            robot.ActivateAndHome()
            robot.WaitHomed()
            print("Robot homed.")

            robot.SetTrf(params.HEAD_OFFSET[0], params.HEAD_OFFSET[1], params.HEAD_OFFSET[2], 0, 0, 0)
            robot.SetJointVelLimit(params.MAX_JOINT_VEL_PERCENTAGE)
            robot.MoveJoints(*params.ROBOT_HEAD_INIT_POSE)
            robot.WaitIdle(60)
            print("Robot at initial pose.")

            params.ROBOT_INSTANCE = robot
        except Exception as e:
            print(f"WARNING: Could not connect to Meca500 — {e}")
            print("Opening UI without head control.")
            robot = None

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
