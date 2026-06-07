"""Global configuration constants and runtime state for patient motion control.

All numeric constants, hardware addresses, and mutable runtime state live here
so every module reads from a single source of truth without circular imports.
Runtime state variables (GIMBAL_KEYS, QUIT_FLAG, ROBOT_INSTANCE) allow
cross-module access without global declarations.
"""

# ─────────────────────────────────────────────
#  Meca500 Head Robot
# ─────────────────────────────────────────────

ROBOT_IP_ADDRESS = "192.168.0.100"

# Cartesian poses [X, Y, Z, UX, UY, UZ] in mm / degrees
ROBOT_INIT_POSE          = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ROBOT_EYE_PUCK_INIT_POSE = [0.0, -20.0, 20.0, 0.0, -90.0, 0.0]
EYE_PUCK_OFFSET          = [0.0, 0.0, 40.0]   # [X, Y, Z] mm

# Joint angles J1–J6 in degrees
ROBOT_HEAD_INIT_POSE = [0, -30, 30, 90, -90, 0]

# TRF origin offset from flange [X, Y, Z] in mm
HEAD_OFFSET = [0.0, -80.0, 0.0]

# Joint travel limits [min, max] in degrees (per Meca500 datasheet)
MECA500_JOINT_LIMITS = [
    (-170, 170),   # J1
    ( -70,  90),   # J2
    (-135,  70),   # J3
    (-170, 170),   # J4
    (-115, 115),   # J5
    (-360, 360),   # J6
]

MAX_VELOCITY             = 20    # mm/s — Cartesian TRF move limit
MAX_JOINT_VEL_PERCENTAGE = 2     # % of max joint velocity for initial pose move
JOINT_VEL_MIN            = 0.2  # % — lower bound for joint jog UI slider
JOINT_VEL_MAX            = 5.0  # % — upper bound for joint jog UI slider

# ─────────────────────────────────────────────
#  Dynamixel Eye Gimbal
# ─────────────────────────────────────────────

# Serial port — overridden at runtime in control_patient_motion.py based on OS
PORT      = '/dev/ttyACM0'
BAUD_RATE = 57600
PROTOCOL  = 2.0

# Motor IDs (assigned via Dynamixel Wizard 2.0)
DXL_1 = 1   # Superior / Inferior axis
DXL_2 = 2   # Temporal / Nasal axis

# XL-330 control table addresses
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_VELOCITY    = 104
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

POSITION_CONTROL_MODE = 3
COUNTS_PER_REV        = 4096              # encoder resolution (counts / revolution)
COUNTS_PER_DEG        = COUNTS_PER_REV / 360.0   # ~11.378 counts per degree

# Encoder center: XL-330 range 0–4095 (0°–360°); 2047 ≈ 180° = neutral position
EYE_CENTER = 2047

# Joint limits: ±45° from center
_EYE_LIMIT_COUNTS = round(45 * COUNTS_PER_DEG)   # = 512 counts
JOINT_MIN_1 = EYE_CENTER - _EYE_LIMIT_COUNTS      # = 1535
JOINT_MAX_1 = EYE_CENTER + _EYE_LIMIT_COUNTS      # = 2559
JOINT_MIN_2 = JOINT_MIN_1
JOINT_MAX_2 = JOINT_MAX_1

# Movement tuning
STEP_SIZE  = 44           # encoder counts moved per loop tick while key is held
LOOP_HZ    = 50           # control loop rate (Hz)
LOOP_DELAY = 1.0 / LOOP_HZ

HEAD_STEP_DEG = 2         # degrees per loop tick for Meca500 head rotation

# ─────────────────────────────────────────────
#  Head Motion Coordinate Mapping
# ─────────────────────────────────────────────
#
#  IMU frame (sensor mounted on forehead):
#    +X  =  temporal  (toward right ear)
#    +Y  =  inferior  (toward chin, downward)
#    +Z  =  out of face (normal to forehead, forward-upward)
#
#  The IMU fusion algorithm outputs gravity-referenced Euler angles:
#    pitch  →  sagittal-plane tilt:  extension(+) / flexion(-)
#    roll   →  frontal-plane tilt:   right tilt(+) / left tilt(-)
#
#  Robot TRF mapping  (MoveLinRelTrf dux, duy, duz):
#    pitch  → UX  (rotation about TRF X)  — sagittal plane / nodding
#    roll   → UZ  (rotation about TRF Z)  — frontal plane  / lateral tilt
#    yaw    → UY  (rotation about TRF Y)  — axial rotation — NOT in data
#
#  Signs below map IMU positive direction to robot positive UX / UZ.
#  Verify empirically: play a file and confirm the robot moves in the
#  expected direction; negate the sign if it is inverted.
#
HEAD_PITCH_SIGN = +1   # +1: IMU extension(+pitch) → robot UX(+); −1: invert
HEAD_ROLL_SIGN  = +1   # +1: IMU right tilt(+roll)  → robot UZ(+); −1: invert

# Window used to compute the resting-pose baseline at the start of each file.
# Samples within this many milliseconds of t=0 are averaged to form the
# reference pitch/roll; all subsequent samples are expressed relative to it.
HEAD_BASELINE_MS = 500

# ─────────────────────────────────────────────
#  File Paths
# ─────────────────────────────────────────────

HEAD_MOTION_PROFILE_FOLDER = 'head_motion_profile'
EYE_MOTION_PROFILE_FOLDER  = 'eye_motion_profile'

# ─────────────────────────────────────────────
#  Runtime State
# ─────────────────────────────────────────────

GIMBAL_KEYS    = {'up': False, 'down': False, 'left': False, 'right': False}
HEAD_KEYS      = {'up': False, 'down': False, 'left': False, 'right': False}
QUIT_FLAG      = False
ROBOT_INSTANCE = None
