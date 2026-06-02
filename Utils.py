import sys
import termios
import Parameter as params
from dynamixel_sdk import *

# ─────────────────────────────────────────────
#  TERMINAL ECHO
# ─────────────────────────────────────────────
_saved_term_settings = None


def disable_echo():
    global _saved_term_settings
    fd = sys.stdin.fileno()
    _saved_term_settings = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new)


def enable_echo():
    global _saved_term_settings
    if _saved_term_settings is not None:
        try:
            fd = sys.stdin.fileno()
            termios.tcflush(fd, termios.TCIFLUSH)   # discard buffered keystrokes
            termios.tcsetattr(fd, termios.TCSADRAIN, _saved_term_settings)
        except Exception:
            pass
        _saved_term_settings = None


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


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
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 0)
    packet.write1ByteTxRx(port, motor_id, params.ADDR_OPERATING_MODE, params.POSITION_CONTROL_MODE)
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 1)
    packet.write4ByteTxRx(port, motor_id, params.ADDR_PROFILE_VELOCITY, 0)


def disable_motor(port, packet, motor_id):
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 0)