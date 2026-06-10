# meca_arm_trajectory_planning

Patient motion simulation system for surgical training. Controls a **Meca500 6-DOF robot arm** (head motion) and a **Dynamixel XL-330 gimbal** (eye motion) to reproduce realistic patient head and eye movements from recorded IMU and DM data.

---

## Hardware

| Component | Model | Interface |
|-----------|-------|-----------|
| Head robot | Mecademic Meca500 | Ethernet (192.168.0.100) |
| Eye gimbal motor 1 (Superior/Inferior) | Robotis XL-330-M288-T (ID 1) | USB serial |
| Eye gimbal motor 2 (Temporal/Nasal) | Robotis XL-330-M288-T (ID 2) | USB serial |
| Head IMU | — | Recorded offline (CSV) |
| Eye | - | Recorded from DM |

---

## Requirements

```
Python >= 3.10
mecademicpy == 3.0.3
dynamixel-sdk == 4.0.5
pynput == 1.8.2
pyqt6
```

Install:

```bash
pip install -r requirements.txt
```

> **macOS note:** if PyQt6 cannot find the Qt platform plugin on first run, set the environment variable before launching:
> ```bash
> export QT_QPA_PLATFORM_PLUGIN_PATH="$(python -c 'import PyQt6; import os; print(os.path.dirname(PyQt6.__file__))')/Qt6/plugins"
> ```

---

## Quick start

### GUI

```bash
# Head + eye (default)
python main.py

# Head only (no gimbal)
python main.py --head

# Eye gimbal only (no Meca500)
python main.py --eye
```

On launch the script:
1. Connects to the Meca500, activates, homes, and moves to the init pose
2. Opens the serial port, initialises both XL-330 motors, and centres them
3. Opens the PyQt6 control window

### Standalone CLI scripts

| Script | Purpose |
|--------|---------|
| `keyboard_gimbal_control.py` | WASD keyboard control of the eye gimbal (terminal) |

---

## GUI overview

The control window has two columns.

### Left column

#### Breathing (continuous loop)
| Button | Action |
|--------|--------|
| **▶ Breathe** | Load `HEAD_REST_PROFILE` and loop it indefinitely |
| **■ Stop Breathing** | Halt the loop and return the head to the home position |

#### Interruptions
Play one-shot trajectories while breathing is active. After the trajectory finishes the head rewinds to home and the breathing loop automatically resumes from the same point in the cycle.

| Button | Profile |
|--------|---------|
| **Cough** | `HEAD_COUGH_PROFILE` |
| **Clear Throat** | `HEAD_CLEAR_THROAT_PROFILE` |

> If breathing is **not** active, these buttons play as one-shot trajectories.

#### Custom file playback
Load any head motion profile CSV/TXT via **Browse…**, then use **▶ Play** / **■ Stop** for one-shot playback with auto-rewind to home.

#### Eye Motion Profiles
| Button | Profile |
|--------|---------|
| **Bell's Reflex** | Conjugate eye deviation (`bells`) |
| **Saccadic** | Saccadic eye movement (`saccadic`) |

Eye profiles auto-rewind the gimbal to centre after each play.

#### Gimbal speed
Slider controls encoder counts per 50 Hz tick. Label shows equivalent °/s and RPM. **Gimbal Reset** smoothly returns both motors to centre.

#### Key bindings
| Key | Action |
|-----|--------|
| `W` / `S` | Eye superior / inferior |
| `A` / `D` | Eye temporal / nasal |
| `Q` / `ESC` | Quit |

### Right column — TRF Cartesian Jog

Hold any jog button to move the Meca500 along the selected TRF axis. Step sizes are set via the **Linear step (mm)** and **Angular step (°)** spinboxes.

| Button | Action |
|--------|--------|
| **Go Init** | Move joints to `ROBOT_HEAD_INIT_POSE` |
| **Set Home** | Capture current joint angles as the playback home position |
| **Go Home** | Return to the last captured home position |
| **Reset Error** | Clear robot error and resume motion |
| **Get Robot Pose** | Read and display current joint angles J1–J6 |

---

## Motion profile formats

### Head profiles (`head_motion_profile/`)

Plain CSV/TXT files with a one-row header. Seven columns per sample:

```
time, gyro0, gyro1, gyro2, acc0, acc1, acc2
```

| Column | Unit | Description |
|--------|------|-------------|
| `time` | sample count | Multiply by 0.001 for seconds (1 kHz IMU) |
| `gyro0` | °/s | Roll-axis gyroscope (rotation about X=inferior) |
| `gyro1` | °/s | Pitch-axis gyroscope (rotation about Y=nasal) |
| `gyro2` | °/s | Yaw-axis gyroscope (not used) |
| `acc0–2` | g | Accelerometer X/Y/Z |

The loader fuses gyroscope + accelerometer via a complementary filter to produce per-sample pitch and roll angles, then sends incremental `MoveLinRelTrf` commands mapped as:

```
pitch  →  TRF UX  (sagittal / nodding)
roll   →  TRF UZ  (frontal / lateral tilt)
```

### Eye profiles (`eye_motion_profile/`)

CSV files with a one-row header, three columns:

```
t, x, y
```

| Column | Unit | Description |
|--------|------|-------------|
| `t` | s | Timestamp |
| `x` | deg | Vertical displacement (+inferior) |
| `y` | deg | Horizontal displacement (+nasal) |

Values are converted to XL-330 encoder counts and clamped to ±45° from centre.

---

## Configuration (`Parameter.py`)

All tunable constants live here. Key sections:

### Meca500 head robot

| Constant | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP_ADDRESS` | `"192.168.0.100"` | Robot ethernet IP |
| `ROBOT_HEAD_INIT_POSE` | `[0, -30, 30, 90, -90, 0]` | Joint angles at startup (deg) |
| `HEAD_OFFSET` | `[0, -80, 0]` | TRF offset from flange (mm) |
| `MAX_VELOCITY` | `20` | Cartesian jog speed limit (mm/s) |
| `MAX_JOINT_VEL_PERCENTAGE` | `2` | Joint velocity cap for init move (%) |

### Head motion profiles

| Constant | Value |
|----------|-------|
| `HEAD_REST_PROFILE` | `'01_rest_2'` |
| `HEAD_COUGH_PROFILE` | `'08_cough'` |
| `HEAD_CLEAR_THROAT_PROFILE` | `'09_clear_throat'` |

### IMU sensor fusion

| Constant | Default | Description |
|----------|---------|-------------|
| `HEAD_CF_ALPHA` | `0.98` | Complementary filter gyro weight |
| `HEAD_ACCEL_LPF_BETA` | `0.97` | Accelerometer IIR low-pass weight |
| `HEAD_PITCH_SIGN` | `+1` | Invert pitch direction if needed |
| `HEAD_ROLL_SIGN` | `+1` | Invert roll direction if needed |
| `HEAD_BASELINE_MS` | `500` | Resting-pose baseline window (ms) |

### Dynamixel eye gimbal

| Constant | Default | Description |
|----------|---------|-------------|
| `DXL_1` / `DXL_2` | `1` / `2` | Motor IDs |
| `BAUD_RATE` | `57600` | Serial baud rate |
| `EYE_CENTER` | `2047` | Neutral encoder count |
| `EYE_FORCE_LIMIT_N` | `1.0` | Force cap per motor (N) |
| `EYE_MOMENT_ARM_MM` | `23.0` | Gimbal lever arm (mm) — measure and update |

### UI

| Constant | Default | Description |
|----------|---------|-------------|
| `UI_FONT_SIZE` | `12` | Base font size (pt) |
| `UI_LEFT_COL_WIDTH` | `320` | Left panel fixed width (px) |
| Colour constants | — | Catppuccin Mocha palette (`UI_BG_COLOR`, `UI_ACCENT_COLOR`, …) |

---

## File structure

```
meca_arm_trajectory_planning/
├── main.py                     # Main entry point (GUI)
├── ControlWindow.py            # PyQt6 control window
├── Parameter.py                # All configuration constants
├── Utils.py                    # Shared utilities (motor I/O, UI helpers)
├── keyboard_gimbal_control.py  # Terminal WASD gimbal control
├── dynamixel_motor_control.py  # Low-level Dynamixel test script
├── requirements.txt
├── head_motion_profile/        # IMU CSV files (*.txt, *.csv)
└── eye_motion_profile/         # Eye trajectory CSVs (*.csv)
```
