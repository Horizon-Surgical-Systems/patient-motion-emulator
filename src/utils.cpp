// utils.cpp — Dynamixel motor I/O, math helpers, and Qt stylesheet builders.
// Also implements Params::Load() / Params::Get() using a minimal YAML parser.

#include "utils.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "parameter.h"

// Pull in the real Dynamixel SDK types at implementation time only.
#ifndef NO_DYNAMIXEL
#include <dynamixel_sdk.h>
#endif

// ─── Params singleton ──────────────────────────────────────────────────────────

namespace {

// Minimal YAML parser: supports only the subset used in parameters.yaml.
//   - Sections:  "section_key:"
//   - Scalars:   "  key: value"  (value is a number, bool, or quoted string)
//   - Sequences: "  key: [v1, v2, ...]"
//   - Comments:  lines starting with '#' are ignored
//   - String values may be surrounded by double-quotes (stripped)

using YamlMap = std::unordered_map<std::string, std::string>;

std::string TrimWhitespace(const std::string& s) {
  const auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

std::string StripQuotes(const std::string& s) {
  if (s.size() >= 2 && s.front() == '"' && s.back() == '"')
    return s.substr(1, s.size() - 2);
  return s;
}

// Parse a YAML sequence like "[0.0, -30.0, 30.0, 90.0, -90.0, 0.0]" into a
// vector of string tokens.
std::vector<std::string> ParseSequence(const std::string& val) {
  std::vector<std::string> tokens;
  auto inner_start = val.find('[');
  auto inner_end   = val.rfind(']');
  if (inner_start == std::string::npos || inner_end == std::string::npos)
    return tokens;
  std::string inner = val.substr(inner_start + 1, inner_end - inner_start - 1);
  std::stringstream ss(inner);
  std::string token;
  while (std::getline(ss, token, ','))
    tokens.push_back(TrimWhitespace(token));
  return tokens;
}

// Build a flat map of "section.key" -> "value" from the YAML file.
YamlMap ParseYaml(const std::string& path) {
  YamlMap map;
  std::ifstream f(path);
  if (!f.is_open())
    throw std::runtime_error("Cannot open YAML file: " + path);

  std::string current_section;
  std::string line;
  while (std::getline(f, line)) {
    // Strip inline comments.
    auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos)
      line = line.substr(0, comment_pos);

    if (TrimWhitespace(line).empty())
      continue;

    // Detect indentation level.
    size_t indent = 0;
    while (indent < line.size() && (line[indent] == ' ' || line[indent] == '\t'))
      ++indent;

    std::string trimmed = TrimWhitespace(line);

    // Locate colon separator.
    auto colon = trimmed.find(':');
    if (colon == std::string::npos)
      continue;

    std::string key   = TrimWhitespace(trimmed.substr(0, colon));
    std::string value = TrimWhitespace(trimmed.substr(colon + 1));

    if (indent == 0) {
      // Top-level section or bare key.
      if (value.empty()) {
        current_section = key;
      } else {
        map[key] = StripQuotes(value);
      }
    } else {
      // Indented key under a section.
      std::string full_key = current_section.empty() ? key : current_section + "." + key;
      map[full_key] = StripQuotes(value);
    }
  }
  return map;
}

double GetDouble(const YamlMap& m, const std::string& key) {
  auto it = m.find(key);
  if (it == m.end())
    throw std::runtime_error("Missing YAML key: " + key);
  return std::stod(it->second);
}

int GetInt(const YamlMap& m, const std::string& key) {
  return static_cast<int>(GetDouble(m, key));
}

std::string GetString(const YamlMap& m, const std::string& key) {
  auto it = m.find(key);
  if (it == m.end())
    throw std::runtime_error("Missing YAML key: " + key);
  return it->second;
}

std::vector<double> GetDoubleSeq(const YamlMap& m, const std::string& key) {
  auto it = m.find(key);
  if (it == m.end())
    throw std::runtime_error("Missing YAML key: " + key);
  auto tokens = ParseSequence(it->second);
  std::vector<double> result;
  result.reserve(tokens.size());
  for (const auto& t : tokens)
    result.push_back(std::stod(t));
  return result;
}

bool g_loaded = false;

}  // namespace

// ─── Params static methods ─────────────────────────────────────────────────────

void Params::Load(const std::string& yaml_path) {
  YamlMap m = ParseYaml(yaml_path);

  Params& p = MutableGet();

  // Robot.
  p.robot_ip_address         = GetString(m, "robot.ip_address");
  {
    auto v = GetDoubleSeq(m, "robot.head_init_pose");
    if (v.size() != 6)
      throw std::runtime_error("robot.head_init_pose must have 6 elements");
    for (size_t i = 0; i < 6; ++i) p.robot_head_init_pose[i] = v[i];
  }
  {
    auto v = GetDoubleSeq(m, "robot.head_offset");
    if (v.size() != 3)
      throw std::runtime_error("robot.head_offset must have 3 elements");
    for (size_t i = 0; i < 3; ++i) p.head_offset[i] = v[i];
  }
  p.max_velocity             = GetDouble(m, "robot.max_velocity");
  p.max_joint_vel_percentage = GetDouble(m, "robot.max_joint_vel_percentage");
  p.joint_vel_min            = GetDouble(m, "robot.joint_vel_min");
  p.joint_vel_max            = GetDouble(m, "robot.joint_vel_max");

  // Gimbal.
  p.port                   = GetString(m, "gimbal.port");
  p.baud_rate              = GetInt(m, "gimbal.baud_rate");
  p.protocol               = static_cast<float>(GetDouble(m, "gimbal.protocol"));
  p.dxl_1                  = GetInt(m, "gimbal.motor_id_1");
  p.dxl_2                  = GetInt(m, "gimbal.motor_id_2");
  p.addr_operating_mode    = GetInt(m, "gimbal.addr_operating_mode");
  p.addr_current_limit     = GetInt(m, "gimbal.addr_current_limit");
  p.addr_torque_enable     = GetInt(m, "gimbal.addr_torque_enable");
  p.addr_goal_velocity     = GetInt(m, "gimbal.addr_goal_velocity");
  p.addr_profile_velocity  = GetInt(m, "gimbal.addr_profile_velocity");
  p.addr_goal_position     = GetInt(m, "gimbal.addr_goal_position");
  p.addr_present_position  = GetInt(m, "gimbal.addr_present_position");
  p.position_control_mode  = GetInt(m, "gimbal.position_control_mode");
  p.counts_per_rev         = GetInt(m, "gimbal.counts_per_rev");
  p.counts_per_deg         = GetDouble(m, "gimbal.counts_per_deg");
  p.eye_center             = GetInt(m, "gimbal.eye_center");
  p.eye_limit_counts       = GetInt(m, "gimbal.eye_limit_counts");
  p.joint_min_1            = GetInt(m, "gimbal.joint_min_1");
  p.joint_max_1            = GetInt(m, "gimbal.joint_max_1");
  p.joint_min_2            = GetInt(m, "gimbal.joint_min_2");
  p.joint_max_2            = GetInt(m, "gimbal.joint_max_2");
  p.eye_force_limit_n      = GetDouble(m, "gimbal.eye_force_limit_n");
  p.eye_moment_arm_mm      = GetDouble(m, "gimbal.eye_moment_arm_mm");
  p.eye_current_limit_counts = GetInt(m, "gimbal.eye_current_limit_counts");

  // Control.
  p.step_size = GetInt(m, "control.step_size");
  p.loop_hz   = GetInt(m, "control.loop_hz");

  // IMU.
  p.head_cf_alpha       = GetDouble(m, "imu.head_cf_alpha");
  p.head_accel_lpf_beta = GetDouble(m, "imu.head_accel_lpf_beta");
  p.head_baseline_ms    = GetInt(m, "imu.head_baseline_ms");

  // Head mapping.
  p.head_pitch_sign = GetDouble(m, "head_mapping.pitch_sign");
  p.head_roll_sign  = GetDouble(m, "head_mapping.roll_sign");

  // Paths.
  p.head_motion_profile_folder = GetString(m, "paths.head_motion_profile_folder");
  p.eye_motion_profile_folder  = GetString(m, "paths.eye_motion_profile_folder");
  p.head_rest_profile          = GetString(m, "paths.head_rest_profile");
  p.head_cough_profile         = GetString(m, "paths.head_cough_profile");
  p.head_clear_throat_profile  = GetString(m, "paths.head_clear_throat_profile");

  // UI.
  p.ui_font_size       = GetInt(m, "ui.font_size");
  p.ui_left_col_width  = GetInt(m, "ui.left_col_width");
  p.ui_bg_color        = GetString(m, "ui.bg_color");
  p.ui_card_color      = GetString(m, "ui.card_color");
  p.ui_accent_color    = GetString(m, "ui.accent_color");
  p.ui_fg_color        = GetString(m, "ui.fg_color");
  p.ui_dim_color       = GetString(m, "ui.dim_color");
  p.ui_sep_color       = GetString(m, "ui.sep_color");
  p.ui_btn_purple      = GetString(m, "ui.btn_purple");
  p.ui_btn_green       = GetString(m, "ui.btn_green");
  p.ui_btn_red         = GetString(m, "ui.btn_red");
  p.ui_btn_gray        = GetString(m, "ui.btn_gray");

  g_loaded = true;
}

// Function-local static — constructed on first call, private constructor is
// accessible because MutableGet() is a static member of Params itself.
Params& Params::MutableGet() {
  static Params instance;
  return instance;
}

const Params& Params::Get() {
  if (!g_loaded)
    throw std::runtime_error("Params::Load() must be called before Params::Get()");
  return MutableGet();
}

// ─── utils namespace implementations ──────────────────────────────────────────

namespace utils {

int ReadPosition(void* port, void* packet, int motor_id) {
#ifndef NO_DYNAMIXEL
  const Params& p = Params::Get();
  auto* ph = static_cast<dynamixel::PortHandler*>(port);
  auto* pk = static_cast<dynamixel::PacketHandler*>(packet);
  uint32_t data = 0;
  uint8_t  error = 0;
  int result = pk->read4ByteTxRx(ph, static_cast<uint8_t>(motor_id),
                                  static_cast<uint16_t>(p.addr_present_position),
                                  &data, &error);
  if (result != COMM_SUCCESS) return -1;
  // XL-330 wraps at 2^32; treat values above 2^31-1 as negative (signed 32-bit).
  int32_t signed_data = static_cast<int32_t>(data);
  return static_cast<int>(signed_data);
#else
  (void)port; (void)packet; (void)motor_id;
  return -1;
#endif
}

void WritePosition(void* port, void* packet, int motor_id, int position) {
#ifndef NO_DYNAMIXEL
  const Params& p = Params::Get();
  auto* ph = static_cast<dynamixel::PortHandler*>(port);
  auto* pk = static_cast<dynamixel::PacketHandler*>(packet);
  uint8_t error = 0;
  pk->write4ByteTxRx(ph, static_cast<uint8_t>(motor_id),
                     static_cast<uint16_t>(p.addr_goal_position),
                     static_cast<uint32_t>(position), &error);
#else
  (void)port; (void)packet; (void)motor_id; (void)position;
#endif
}

void SetupMotor(void* port, void* packet, int motor_id) {
#ifndef NO_DYNAMIXEL
  const Params& p = Params::Get();
  auto* ph = static_cast<dynamixel::PortHandler*>(port);
  auto* pk = static_cast<dynamixel::PacketHandler*>(packet);
  uint8_t error = 0;
  uint8_t id = static_cast<uint8_t>(motor_id);

  // 1. Disable torque (required before writing EEPROM registers).
  pk->write1ByteTxRx(ph, id, static_cast<uint16_t>(p.addr_torque_enable), 0, &error);

  // 2. Set current limit (EEPROM, 2 bytes).
  pk->write2ByteTxRx(ph, id, static_cast<uint16_t>(p.addr_current_limit),
                     static_cast<uint16_t>(p.eye_current_limit_counts), &error);

  // 3. Set position control mode.
  pk->write1ByteTxRx(ph, id, static_cast<uint16_t>(p.addr_operating_mode),
                     static_cast<uint8_t>(p.position_control_mode), &error);

  // 4. Enable torque.
  pk->write1ByteTxRx(ph, id, static_cast<uint16_t>(p.addr_torque_enable), 1, &error);

  // 5. Set profile velocity to 0 (maximum speed).
  pk->write4ByteTxRx(ph, id, static_cast<uint16_t>(p.addr_profile_velocity), 0, &error);
#else
  (void)port; (void)packet; (void)motor_id;
#endif
}

void DisableMotor(void* port, void* packet, int motor_id) {
#ifndef NO_DYNAMIXEL
  const Params& p = Params::Get();
  auto* ph = static_cast<dynamixel::PortHandler*>(port);
  auto* pk = static_cast<dynamixel::PacketHandler*>(packet);
  uint8_t error = 0;
  pk->write1ByteTxRx(ph, static_cast<uint8_t>(motor_id),
                     static_cast<uint16_t>(p.addr_torque_enable), 0, &error);
#else
  (void)port; (void)packet; (void)motor_id;
#endif
}

std::string LightenColor(const std::string& hex_color, int amount) {
  // hex_color must be "#rrggbb".
  if (hex_color.size() < 7 || hex_color[0] != '#') return hex_color;
  auto parse_channel = [&](int offset) {
    int val = std::stoi(hex_color.substr(offset, 2), nullptr, 16);
    return std::min(255, val + amount);
  };
  int r = parse_channel(1);
  int g = parse_channel(3);
  int b = parse_channel(5);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "#%02x%02x%02x", r, g, b);
  return std::string(buf);
}

std::string BtnQss(const std::string& color) {
  const Params& p = Params::Get();
  std::string hover = LightenColor(color);
  int fs            = p.ui_font_size;
  std::string dim   = p.ui_dim_color;

  std::string qss =
      "QPushButton {"
      "  background-color: " + color + ";"
      "  color: black;"
      "  border: none;"
      "  border-radius: 4px;"
      "  padding: 5px 10px;"
      "  font-family: Arial;"
      "  font-size: " + std::to_string(fs) + "pt;"
      "  font-weight: bold;"
      "}"
      "QPushButton:hover { background-color: " + hover + "; }"
      "QPushButton:pressed { background-color: " + color + "; }"
      "QPushButton:disabled { background-color: " + dim + "; color: #888888; }";
  return qss;
}

std::string GlobalQss() {
  const Params& p = Params::Get();
  int         fs   = p.ui_font_size;
  std::string bg   = p.ui_bg_color;
  std::string fg   = p.ui_fg_color;
  std::string sep  = p.ui_sep_color;
  std::string acct = p.ui_accent_color;

  std::string qss =
      "QMainWindow, QWidget {"
      "  background-color: " + bg + ";"
      "  color: " + fg + ";"
      "  font-family: Arial;"
      "  font-size: " + std::to_string(fs) + "pt;"
      "}"
      "QFrame#card {"
      "  background-color: " + bg + ";"
      "  border: none;"
      "}"
      "QLabel { background: transparent; color: " + fg + "; }"
      "QProgressBar {"
      "  background-color: " + sep + ";"
      "  border: none;"
      "  border-radius: 3px;"
      "  max-height: 8px;"
      "}"
      "QProgressBar::chunk { background-color: " + acct + "; border-radius: 3px; }"
      "QSlider::groove:horizontal {"
      "  background: " + sep + ";"
      "  height: 4px;"
      "  border-radius: 2px;"
      "}"
      "QSlider::handle:horizontal {"
      "  background: " + acct + ";"
      "  width: 12px;"
      "  height: 12px;"
      "  margin: -4px 0;"
      "  border-radius: 6px;"
      "}"
      "QSlider::sub-page:horizontal { background: " + acct + "; border-radius: 2px; }"
      "QDoubleSpinBox {"
      "  background-color: " + bg + ";"
      "  color: " + fg + ";"
      "  border: 1px solid " + sep + ";"
      "  border-radius: 3px;"
      "  padding: 2px 4px;"
      "  selection-background-color: " + acct + ";"
      "}"
      "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {"
      "  background-color: " + sep + ";"
      "  border: none;"
      "  width: 14px;"
      "}";
  return qss;
}

}  // namespace utils
