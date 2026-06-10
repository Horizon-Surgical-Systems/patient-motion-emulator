#ifndef PROJECT_INCLUDE_PARAMETER_H_
#define PROJECT_INCLUDE_PARAMETER_H_

// Runtime parameters loaded from config/parameters.yaml at startup.
// Use Params::Get() to obtain the singleton after calling Params::Load().
//
// YAML parsing is done with a minimal hand-written parser (no third-party
// library required) that handles the flat key: value and sequence formats
// produced by parameters.yaml.

#include <array>
#include <stdexcept>
#include <string>

// Forward-declare the loader so control_window.cpp can include only this header.
class Params {
 public:
  // ─── Meca500 ───────────────────────────────────────────────────────────────
  std::string robot_ip_address;
  std::array<double, 6> robot_head_init_pose;
  std::array<double, 3> head_offset;
  double max_velocity;
  double max_joint_vel_percentage;
  double joint_vel_min;
  double joint_vel_max;

  // ─── Dynamixel ─────────────────────────────────────────────────────────────
  std::string port;
  int baud_rate;
  float protocol;
  int dxl_1;
  int dxl_2;

  int addr_operating_mode;
  int addr_current_limit;
  int addr_torque_enable;
  int addr_goal_velocity;
  int addr_profile_velocity;
  int addr_goal_position;
  int addr_present_position;

  int    position_control_mode;
  int    counts_per_rev;
  double counts_per_deg;
  int    eye_center;
  int    eye_limit_counts;
  int    joint_min_1;
  int    joint_max_1;
  int    joint_min_2;
  int    joint_max_2;

  double eye_force_limit_n;
  double eye_moment_arm_mm;
  int    eye_current_limit_counts;

  // ─── Control ───────────────────────────────────────────────────────────────
  int step_size;
  int loop_hz;

  // ─── IMU ───────────────────────────────────────────────────────────────────
  double head_cf_alpha;
  double head_accel_lpf_beta;
  int    head_baseline_ms;

  // ─── Head mapping ──────────────────────────────────────────────────────────
  double head_pitch_sign;
  double head_roll_sign;

  // ─── Paths ─────────────────────────────────────────────────────────────────
  std::string head_motion_profile_folder;
  std::string eye_motion_profile_folder;
  std::string head_rest_profile;
  std::string head_cough_profile;
  std::string head_clear_throat_profile;

  // ─── UI ────────────────────────────────────────────────────────────────────
  int         ui_font_size;
  int         ui_left_col_width;
  std::string ui_bg_color;
  std::string ui_card_color;
  std::string ui_accent_color;
  std::string ui_fg_color;
  std::string ui_dim_color;
  std::string ui_sep_color;
  std::string ui_btn_purple;
  std::string ui_btn_green;
  std::string ui_btn_red;
  std::string ui_btn_gray;

  // ─── Singleton access ──────────────────────────────────────────────────────

  // Load parameters from a YAML file.  Must be called once before Get().
  // Throws std::runtime_error if the file cannot be opened or a required
  // key is missing.
  static void Load(const std::string& yaml_path);

  // Return the loaded singleton.  Throws if Load() has not been called.
  static const Params& Get();

 private:
  Params() = default;
  static Params& MutableGet();
};

#endif  // PROJECT_INCLUDE_PARAMETER_H_
