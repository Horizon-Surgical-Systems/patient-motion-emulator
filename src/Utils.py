"""Shared utilities: terminal control, motor I/O, and math helpers."""

from __future__ import annotations

import sys
import termios
from typing import Optional

from dynamixel_sdk import (
    PacketHandler,
    PortHandler,
    COMM_SUCCESS,
)

import Parameter as params


# ─────────────────────────────────────────────
#  Terminal echo
# ─────────────────────────────────────────────

_saved_term_settings = None


def disable_echo() -> None:
    """Disable terminal echo so keystrokes are invisible while running."""
    global _saved_term_settings
    fd = sys.stdin.fileno()
    _saved_term_settings = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new)


def enable_echo() -> None:
    """Flush buffered keystrokes and restore the original terminal settings."""
    global _saved_term_settings
    if _saved_term_settings is None:
        return
    try:
        fd = sys.stdin.fileno()
        termios.tcflush(fd, termios.TCIFLUSH)
        termios.tcsetattr(fd, termios.TCSADRAIN, _saved_term_settings)
    except Exception:
        pass
    _saved_term_settings = None


# ─────────────────────────────────────────────
#  Math helpers
# ─────────────────────────────────────────────

def clamp(value: float, lo: float, hi: float) -> float:
    """Return *value* clamped to the closed interval [*lo*, *hi*]."""
    return max(lo, min(hi, value))


# ─────────────────────────────────────────────
#  Dynamixel motor I/O
# ─────────────────────────────────────────────

def read_position(
    port: PortHandler,
    packet: PacketHandler,
    motor_id: int,
) -> Optional[int]:
    """Read the present encoder position of *motor_id*.

    Returns the position in counts, or None on communication failure.
    Converts the raw unsigned 32-bit value to a signed integer.
    """
    data, result, _ = packet.read4ByteTxRx(port, motor_id, params.ADDR_PRESENT_POSITION)
    if result != COMM_SUCCESS:
        return None
    # XL-330 wraps at 2^32; treat values above 2^31-1 as negative
    if data > 2**31 - 1:
        data -= 2**32
    return data


def write_position(
    port: PortHandler,
    packet: PacketHandler,
    motor_id: int,
    position: int,
) -> None:
    """Send a goal position command to *motor_id*."""
    packet.write4ByteTxRx(port, motor_id, params.ADDR_GOAL_POSITION, position)


def setup_motor(
    port: PortHandler,
    packet: PacketHandler,
    motor_id: int,
) -> None:
    """Initialize *motor_id* in position-control mode with a physiological force cap.

    Current Limit (addr 38) is an EEPROM register that must be written while
    torque is disabled.  EYE_CURRENT_LIMIT_COUNTS is derived from the 4 N
    extraocular-muscle force limit and the gimbal moment arm — see Parameter.py.
    """
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 0)
    packet.write2ByteTxRx(port, motor_id, params.ADDR_CURRENT_LIMIT,
                          params.EYE_CURRENT_LIMIT_COUNTS)
    packet.write1ByteTxRx(port, motor_id, params.ADDR_OPERATING_MODE, params.POSITION_CONTROL_MODE)
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 1)
    packet.write4ByteTxRx(port, motor_id, params.ADDR_PROFILE_VELOCITY, 0)


def disable_motor(
    port: PortHandler,
    packet: PacketHandler,
    motor_id: int,
) -> None:
    """Release torque on *motor_id*."""
    packet.write1ByteTxRx(port, motor_id, params.ADDR_TORQUE_ENABLE, 0)


# ─────────────────────────────────────────────
#  UI stylesheet helpers
# ─────────────────────────────────────────────

def lighten_color(hex_color: str, amount: int = 25) -> str:
    """Return hex_color brightened by amount per channel (capped at 255)."""
    r = min(255, int(hex_color[1:3], 16) + amount)
    g = min(255, int(hex_color[3:5], 16) + amount)
    b = min(255, int(hex_color[5:7], 16) + amount)
    return f'#{r:02x}{g:02x}{b:02x}'


def btn_qss(color: str) -> str:
    """QSS string for a flat QPushButton with the given background color."""
    hover = lighten_color(color)
    fs    = params.UI_FONT_SIZE
    dim   = params.UI_DIM_COLOR
    return f"""
        QPushButton {{
            background-color: {color};
            color: black;
            border: none;
            border-radius: 4px;
            padding: 5px 10px;
            font-family: Arial;
            font-size: {fs}pt;
            font-weight: bold;
        }}
        QPushButton:hover {{ background-color: {hover}; }}
        QPushButton:pressed {{ background-color: {color}; }}
        QPushButton:disabled {{ background-color: {dim}; color: #888888; }}
    """


def global_qss() -> str:
    """Build the application-wide Qt stylesheet from Parameter UI values."""
    fs   = params.UI_FONT_SIZE
    bg   = params.UI_BG_COLOR
    fg   = params.UI_FG_COLOR
    sep  = params.UI_SEP_COLOR
    acct = params.UI_ACCENT_COLOR
    return f"""
QMainWindow, QWidget {{
    background-color: {bg};
    color: {fg};
    font-family: Arial;
    font-size: {fs}pt;
}}
QFrame#card {{
    background-color: {bg};
    border: none;
}}
QLabel {{ background: transparent; color: {fg}; }}
QProgressBar {{
    background-color: {sep};
    border: none;
    border-radius: 3px;
    max-height: 8px;
}}
QProgressBar::chunk {{ background-color: {acct}; border-radius: 3px; }}
QSlider::groove:horizontal {{
    background: {sep};
    height: 4px;
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {acct};
    width: 12px;
    height: 12px;
    margin: -4px 0;
    border-radius: 6px;
}}
QSlider::sub-page:horizontal {{ background: {acct}; border-radius: 2px; }}
QDoubleSpinBox {{
    background-color: {bg};
    color: {fg};
    border: 1px solid {sep};
    border-radius: 3px;
    padding: 2px 4px;
    selection-background-color: {acct};
}}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
    background-color: {sep};
    border: none;
    width: 14px;
}}
"""
