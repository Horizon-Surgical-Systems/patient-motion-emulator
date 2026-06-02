import time
import sys
import os
import atexit
import termios
import Parameter as params
from dynamixel_sdk import *
from pynput import keyboard
from Utils import *

_saved_term_settings = None


def disable_echo():
    """Save current terminal settings, then disable echo on stdin."""
    global _saved_term_settings
    fd = sys.stdin.fileno()
    _saved_term_settings = termios.tcgetattr(fd)      # full snapshot
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~termios.ECHO                   # clear ECHO bit only
    termios.tcsetattr(fd, termios.TCSADRAIN, new)


def enable_echo():
    """Restore terminal to the exact state captured by disable_echo()."""
    global _saved_term_settings
    if _saved_term_settings is not None:
        try:
            termios.tcsetattr(sys.stdin.fileno(),
                              termios.TCSADRAIN,
                              _saved_term_settings)
        except Exception:
            pass
        _saved_term_settings = None

# ─────────────────────────────────────────────
#  MOTOR HELPERS
# ─────────────────────────────────────────────

def read_position(port, packet, motor_id):
    data, result, error = packet.read4ByteTxRx(port, motor_id, params.ADDR_PRESENT_POSITION)
    if result != COMM_SUCCESS:
        return None
    if data > 2**31 - 1:
        data = data - 2**32
    return data


def write_position(port, packet, motor_id, position):
    packet.write4ByteTxRx(port, motor_id, params.ADDR_GOAL_POSITION, position)

def setup_motor(port, packet, motor_id):
    packet.write1ByteTxRx(port, motor_id, params.ADDR_PROFILE_VELOCITY, 0)
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 0)
    packet.write1ByteTxRx(port, motor_id, params.ADDR_OPERATING_MODE, params.POSITION_CONTROL_MODE)
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 1)


def disable_motor(port, packet, motor_id):
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 0)

def diagnose_motor(port, packet, motor_id):
    """Read multiple motor registers to diagnose issues."""
    print(f"\n--- Diagnostic Info for Motor {motor_id} ---")

    data, result, error = packet.read4ByteTxRx(port, motor_id, params.ADDR_PRESENT_POSITION)
    if result == COMM_SUCCESS:
        print(f"  Present Position (raw): {data}")
        print(f"  Present Position (int): {data if data <= 4095 else 'Out of range'}")
    else:
        print(f"  Present Position: Read failed - {packet.getTxRxResult(result)}")

    mode, result, error = packet.read1ByteTxRx(port, motor_id, params.ADDR_OPERATING_MODE)
    if result == COMM_SUCCESS:
        mode_name = {0: "Current", 1: "Velocity", 3: "Position",
                     4: "Extended Position"}.get(mode, f"Unknown ({mode})")
        print(f"  Operating Mode: {mode} ({mode_name})")
    else:
        print(f"  Operating Mode: Read failed - {packet.getTxRxResult(result)}")

    torque, result, error = packet.read1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE)
    if result == COMM_SUCCESS:
        print(f"  Torque Enabled: {torque == 1}")
    else:
        print(f"  Torque Enable: Read failed - {packet.getTxRxResult(result)}")
    print()

# ─────────────────────────────────────────────
#  KEY STATE  (arrow keys + WASD)
# ─────────────────────────────────────────────
keys_held = {'up': False, 'down': False, 'left': False, 'right': False}
quit_flag = False

def on_press(key):
    global quit_flag
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if   char == 'w':               keys_held['up']    = True
        elif char == 's':               keys_held['down']  = True
        elif char == 'a':               keys_held['left']  = True
        elif char == 'd':               keys_held['right'] = True
        elif char == 'q':
            quit_flag = True
            return False
        elif key == keyboard.Key.up:    keys_held['up']    = True
        elif key == keyboard.Key.down:  keys_held['down']  = True
        elif key == keyboard.Key.left:  keys_held['left']  = True
        elif key == keyboard.Key.right: keys_held['right'] = True
        elif key == keyboard.Key.esc:
            quit_flag = True
            return False

    except AttributeError:
        pass


def on_release(key):
    try:
        char = key.char.lower() if hasattr(key, 'char') and key.char else None

        if   char == 'w':               keys_held['up']    = False
        elif char == 's':               keys_held['down']  = False
        elif char == 'a':               keys_held['left']  = False
        elif char == 'd':               keys_held['right'] = False
        elif key == keyboard.Key.up:    keys_held['up']    = False
        elif key == keyboard.Key.down:  keys_held['down']  = False
        elif key == keyboard.Key.left:  keys_held['left']  = False
        elif key == keyboard.Key.right: keys_held['right'] = False

    except AttributeError:
        pass


# Status line
_status_printed = False

def print_status(pos1, pos2):
    """Overwrite the current terminal line with live motor positions."""
    global _status_printed
    line = f"  Motor 1 (W/S): {pos1:4d}  |  Motor 2 (A/D): {pos2:4d}   "
    sys.stdout.write('\r' + line)
    sys.stdout.flush()
    _status_printed = True

def end_status_line():
    """Advance past the status line so subsequent prints start clean."""
    if _status_printed:
        sys.stdout.write('\n')
        sys.stdout.flush()


def main():
    portHandler   = PortHandler(params.PORT)
    packetHandler = PacketHandler(params.PROTOCOL)

    if not portHandler.openPort():
        print("ERROR: Failed to open port.")
        return
    if not portHandler.setBaudRate(params.BAUD_RATE):
        print("ERROR: Failed to set baud rate.")
        portHandler.closePort()
        return

    print("Port opened successfully.")

    # Safety net: restore echo even on hard crash
    atexit.register(enable_echo)

    # Configure both motors for position control
    setup_motor(portHandler, packetHandler, params.DXL_1)
    setup_motor(portHandler, packetHandler, params.DXL_2)
    print("Motors configured for position control.")
    time.sleep(0.5)

    # Initialize at centre
    pos1 = 2047
    pos2 = 2047
    write_position(portHandler, packetHandler, params.DXL_1, pos1)
    write_position(portHandler, packetHandler, params.DXL_2, pos2)

    print(f"Starting positions — Motor 1: {pos1}, Motor 2: {pos2}")
    print()
    print("  W / S   : Superior/Inferior [also: ↑ / ↓]")
    print("  A / D   : Temporal /Nasal   [also: ← / →]")
    print("  Q / ESC : Quit")
    print()

    # Disable echo AFTER all startup text is flushed
    disable_echo()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while listener.is_alive() and not quit_flag:
            moved = False

            # ── Motor 1: Superior / Inferior ──────────
            if keys_held['up']:
                new1 = clamp(pos1 + params.STEP_SIZE, params.JOINT_MIN_1, params.JOINT_MAX_1)
                if new1 != pos1:
                    pos1 = new1
                    write_position(portHandler, packetHandler, params.DXL_1, pos1)
                    moved = True
            elif keys_held['down']:
                new1 = clamp(pos1 - params.STEP_SIZE, params.JOINT_MIN_1, params.JOINT_MAX_1)
                if new1 != pos1:
                    pos1 = new1
                    write_position(portHandler, packetHandler, params.DXL_1, pos1)
                    moved = True

            # ── Motor 2: Temporal / Nasal ──────────────
            if keys_held['left']:
                new2 = clamp(pos2 + params.STEP_SIZE, params.JOINT_MIN_2, params.JOINT_MAX_2)
                if new2 != pos2:
                    pos2 = new2
                    write_position(portHandler, packetHandler, params.DXL_2, pos2)
                    moved = True
            elif keys_held['right']:
                new2 = clamp(pos2 - params.STEP_SIZE, params.JOINT_MIN_2, params.JOINT_MAX_2)
                if new2 != pos2:
                    pos2 = new2
                    write_position(portHandler, packetHandler, params.DXL_2, pos2)
                    moved = True

            if moved:
                print_status(pos1, pos2)

            time.sleep(params.LOOP_DELAY)

    except KeyboardInterrupt:
        pass

    finally:
        listener.stop()
        end_status_line()
        print("Shutting down — disabling torque.")
        disable_motor(portHandler, packetHandler, params.DXL_1)
        disable_motor(portHandler, packetHandler, params.DXL_2)
        portHandler.closePort()
        print("Port closed.")
        
        enable_echo()   # explicit restore; atexit is a backup


if __name__ == '__main__':
    main()
