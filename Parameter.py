ROBOT_IP_ADDRESS = "192.168.0.100"

ROBOT_INIT_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [X, Y, Z, UX, UY, UZ]

ROBOT_EYE_PUCK_INIT_POSE = [0.0, -20.0, 20.0, 0.0, -90.0, 0.0]  # [X, Y, Z, UX, UY, UZ]
EYE_PUCK_OFFSET = [0.0, 0.0, 40.0] # [X, Y, Z] in mm

ROBOT_HEAD_INIT_POSE = [0, -30, 30, 90, -90, 0]   # joint angles [J1..J6] in degrees
HEAD_OFFSET = [0.0, -80.0, 0.0]                    # [X, Y, Z] in mm

# Meca500 joint travel limits [min, max] in degrees (per hardware datasheet)
MECA500_JOINT_LIMITS = [
    (-170, 170),   # J1
    ( -70,  90),   # J2
    (-135,  70),   # J3
    (-170, 170),   # J4
    (-115, 115),   # J5
    (-360, 360),   # J6
]

MAX_VELOCITY          = 20   # mm/s
MAX_JOINT_VEL_PERCENTAGE = 2  # %
JOINT_VEL_MIN         = 0.2  # % — lower bound for UI velocity slider
JOINT_VEL_MAX         = 5.0  # % — upper bound for UI velocity slider

# ---------------------
# Eye Gimbal Parametersf
# ---------------------

# Motor Configuration
# PORT      = '/dev/cu.usbmodem101'
PORT      = '/dev/ttyACM0'
BAUD_RATE = 57600
PROTOCOL  = 2.0

# Motor IDs (configured through Dynamixel Wizard 2.0)
DXL_1 = 1   # Superior / Inferior
DXL_2 = 2   # Temporal / Nasal

#  CONTROL TABLE  (XL-330)
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_VELOCITY    = 104
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

POSITION_CONTROL_MODE = 3
COUNTS_PER_REV        = 4096   # XL-330 encoder resolution (counts per revolution)

#  JOINT LIMITS
#
#  XL-330 encoder range: 0 – 4095  (0° – 360°)
#  Center position     : 2048       (≈ 180°)
#  1 degree            ≈ 11.4 steps
#
#  Default below allows ±45° from centre:
#    MIN = 2048 - 512 = 1536
#    MAX = 2048 + 512 = 2560
JOINT_MIN_1 = 1536
JOINT_MAX_1 = 2560

JOINT_MIN_2 = 1536
JOINT_MAX_2 = 2560

#  MOVEMENT TUNING
STEP_SIZE     = 44        # encoder counts per loop tick while key held
LOOP_HZ       = 50        # control loop rate (Hz)
LOOP_DELAY    = 1.0 / LOOP_HZ

HEAD_STEP_DEG = 2       # degrees per loop tick for Meca500 head rotation

MOTION_DATA_FOLDER = 'motion_data'   # folder containing IMU motion recording txt files

# ─────────────────────────────────────────────
#  RUNTIME STATE
# ─────────────────────────────────────────────
GIMBAL_KEYS    = {'up': False, 'down': False, 'left': False, 'right': False}
HEAD_KEYS      = {'up': False, 'down': False, 'left': False, 'right': False}
QUIT_FLAG      = False
ROBOT_INSTANCE = None