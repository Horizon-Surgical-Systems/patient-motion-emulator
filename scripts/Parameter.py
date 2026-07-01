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
ROBOT_HEAD_INIT_POSE = [0, 0, 30, 90, -90, 30]

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
MAX_JOINT_VEL_PERCENTAGE = 4     # % of max joint velocity for jogging / homing
HEAD_PLAYBACK_VEL_PCT    = 50    # % of max joint velocity during profile playback
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
ADDR_CURRENT_LIMIT    = 38   # EEPROM — write only while torque is disabled; 2 bytes
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

# ─────────────────────────────────────────────
#  Eye Force Safety Limit
# ─────────────────────────────────────────────
#
#  Physiological basis (Robinson DA, 1975; Miller JM & Robinson DA, 1984):
#    - Peak isometric force per rectus muscle:  ~1.5–2.0 N
#    - Full envelope (active + passive tissue): up to ~4.0 N
#    - Effective moment arm with orbital pulleys: ~10–13 mm
#      (Demer JL et al., 1995 — fibromuscular pulley system)
#    - Peak torque during a fast saccade: ~15–25 mNm
#    - Smooth-pursuit torque:              ~1–5 mNm
#
#  XL-330-M288-T motor constants (Robotis e-Manual):
#    - Stall torque:   0.52 Nm at 5 V
#    - Stall current:  1 100 mA at 5 V
#    - Torque constant Kt = 0.52 / 1.1 ≈ 0.473 Nm/A
#    - Current Limit unit: 1 count ≈ 2.69 mA  (1750 counts full-scale)
#
#  Derivation  (with EYE_MOMENT_ARM_MM = 12 mm, EYE_FORCE_LIMIT_N = 4.0 N):
#    torque_Nm  = 4.0 N × 0.012 m        = 0.048 Nm
#    current_A  = 0.048 / 0.473          ≈ 0.101 A = 101 mA
#    counts     = 101 / 2.69             ≈ 38 counts  → EYE_CURRENT_LIMIT_COUNTS

EYE_FORCE_LIMIT_N     = 1.0    # N — physiological peak
EYE_MOMENT_ARM_MM     = 23.0   # mm — measure gimbal's lever arm and update this

_XL330_KT             = 0.473   # Nm/A  — torque constant (stall torque / stall current)
_XL330_MA_PER_COUNT   = 2.69   # mA per Current Limit count

EYE_CURRENT_LIMIT_COUNTS = max(1, round(
    EYE_FORCE_LIMIT_N * (EYE_MOMENT_ARM_MM / 1000.0)   # → Nm
    / _XL330_KT                                          # → A
    / (_XL330_MA_PER_COUNT / 1000.0)                    # → counts
))

# Movement tuning
STEP_SIZE  = 44           # encoder counts moved per loop tick while key is held
LOOP_HZ    = 50           # control loop rate (Hz)
LOOP_DELAY = 1.0 / LOOP_HZ

HEAD_STEP_DEG = 2         # degrees per loop tick for Meca500 head rotation

# ─────────────────────────────────────────────
#  Head IMU Sensor Fusion
# ─────────────────────────────────────────────
#
#  Complementary filter fuses gyroscope (good short-term, drifts long-term)
#  with accelerometer (gravity reference, no drift, but noisy):
#
#    pitch[n] = α · (pitch[n-1] + gyro1·dt) + (1−α) · pitch_from_accel
#    roll[n]  = α · (roll[n-1]  + gyro0·dt) + (1−α) · roll_from_accel
#
#  HEAD_CF_ALPHA   — gyro weight (0–1).  Higher → trust gyro more (less accel
#                    correction); good starting point is 0.98 at 1 kHz.
#  HEAD_ACCEL_LPF_BETA — IIR low-pass weight for accelerometer.
#                    β = exp(−2π·fc·dt); at 1 kHz, β=0.95 ≈ fc 8 Hz.
#                    Higher β → lower cutoff, more smoothing.
HEAD_CF_ALPHA        = 0.98   # complementary filter gyro weight
HEAD_ACCEL_LPF_BETA  = 0.99   # accelerometer IIR low-pass weight (~8 Hz cutoff)

# ─────────────────────────────────────────────
#  Head Motion Coordinate Mapping
# ─────────────────────────────────────────────
#
#  IMU frame (sensor mounted on forehead):
#    +X  =  inferior  (toward chin, downward)
#    +Y  =  nasal     (toward nose)
#    +Z  =  out of face (normal to forehead, forward-upward)
#
#  The IMU fusion algorithm outputs gravity-referenced Euler angles:
#    pitch  →  sagittal-plane tilt (rotation about Y=nasal):  extension(+) / flexion(-)
#    roll   →  frontal-plane tilt  (rotation about X=inferior): right tilt(+) / left tilt(-)
#    yaw    →  axial rotation (rotation about Z=out-of-face)  — NOT in data
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

# How often (seconds) the breathing loop rehomes the robot to cancel drift.
HEAD_BREATH_REHOME_INTERVAL_S: float = 5.0

# Window used to compute the resting-pose baseline at the start of each file.
# Samples within this many milliseconds of t=0 are averaged to form the
# reference pitch/roll; all subsequent samples are expressed relative to it.
HEAD_BASELINE_MS = 500

# ─────────────────────────────────────────────
#  File Paths
# ─────────────────────────────────────────────

HEAD_MOTION_PROFILE_FOLDER = 'head_motion_profile'
EYE_MOTION_PROFILE_FOLDER  = 'eye_motion_profile'

EYE_BELLS_PROFILE           = '20260618_OD_bells'
EYE_SACCADIC_PROFILE        = '20260618_OD_saccadic'
EYE_GAZE_AVERSION_PROFILE        = 'gaze_aversion'
EYE_DIVERGENT_DRIFT_NASAL_PROFILE    = '20260628_OD_ocular_drift_nasal'
EYE_DIVERGENT_DRIFT_TEMPORAL_PROFILE = '20260628_OD_ocular_drift_temporal'
EYE_DIVERGENT_DRIFT_SUPERIOR_PROFILE = '20260628_OD_ocular_drift_superior'
EYE_DIVERGENT_DRIFT_INFERIOR_PROFILE = '20260628_OD_ocular_drift_inferior'
EYE_VOR_PROFILE                      = 'vor'

HEAD_REST_PROFILE            = 'rohit_rest'
HEAD_COUGH_PROFILE           = 'flore_cough'
HEAD_CLEAR_THROAT_PROFILE    = '09_clear_throat'
HEAD_MOVING_AWAY_PROFILE     = 'rohit_fast_head_motion'
HEAD_VERBAL_CONSENT_PROFILE  = 'brando_verbal_consent'
HEAD_HANDS_MOVING_PROFILE    = 'hands_moving'

# ─────────────────────────────────────────────
#  UI
# ─────────────────────────────────────────────

UI_FONT_SIZE        = 12    # pt — base font size for the control window
UI_LEFT_COL_WIDTH   = 320   # px — fixed width of the left column

# Colour palette (Catppuccin Mocha)
UI_BG_COLOR     = "#1e1e2e"   # window / page background
UI_CARD_COLOR   = "#2a2a3e"   # card surface
UI_ACCENT_COLOR = "#7c6af5"   # accent (labels, progress bar)
UI_FG_COLOR     = "#cdd6f4"   # primary text
UI_DIM_COLOR    = "#6c7086"   # secondary text / disabled
UI_SEP_COLOR    = "#313244"   # separators / trough
UI_BTN_PURPLE   = "#c4bbfc"   # accent actions  (Browse, Set Home)
UI_BTN_GREEN    = "#a6e3a1"   # positive actions (Play, Go Home)
UI_BTN_RED      = "#f38ba8"   # destructive      (Stop, Reset Error)
UI_BTN_GRAY     = "#9399b2"   # hold-to-jog / neutral

# ─────────────────────────────────────────────
#  Runtime State
# ─────────────────────────────────────────────

GIMBAL_KEYS    = {'up': False, 'down': False, 'left': False, 'right': False}
HEAD_KEYS      = {'up': False, 'down': False, 'left': False, 'right': False}
QUIT_FLAG      = False
ROBOT_INSTANCE = None
