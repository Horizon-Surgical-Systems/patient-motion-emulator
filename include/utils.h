#ifndef PROJECT_INCLUDE_UTILS_H_
#define PROJECT_INCLUDE_UTILS_H_

// Shared utility functions: Dynamixel motor I/O, math helpers, and Qt
// stylesheet builders.  All functions are stateless unless noted.

#include <algorithm>
#include <string>

// Forward-declarations to avoid pulling in the full Dynamixel SDK headers
// everywhere.  Actual implementations include dynamixel_sdk.h directly.
struct PortHandler;
struct PacketHandler;

namespace utils {

// ─── Math helpers ─────────────────────────────────────────────────────────────

// Return value clamped to the closed interval [lo, hi].
template <typename T>
T Clamp(T value, T lo, T hi) {
  return std::max(lo, std::min(hi, value));
}

// ─── Dynamixel motor I/O ──────────────────────────────────────────────────────

// Read the present encoder position of motor_id.
// Returns the position in counts, or -1 on communication failure.
// Converts the raw unsigned 32-bit value to a signed integer (XL-330 wrap).
int ReadPosition(void* port, void* packet, int motor_id);

// Send a goal position command to motor_id.
void WritePosition(void* port, void* packet, int motor_id, int position);

// Initialize motor_id in position-control mode with the physiological force
// limit from Params.  Must be called while torque is disabled (the function
// handles the disable/enable sequence itself).
void SetupMotor(void* port, void* packet, int motor_id);

// Release torque on motor_id.
void DisableMotor(void* port, void* packet, int motor_id);

// ─── UI stylesheet helpers ────────────────────────────────────────────────────

// Return hex_color brightened by amount per channel (capped at 255).
// hex_color must be in the form "#rrggbb".
std::string LightenColor(const std::string& hex_color, int amount = 25);

// Return a Qt stylesheet string (QSS) for a flat QPushButton with the given
// background color.  Font size and disabled color are read from Params.
std::string BtnQss(const std::string& color);

// Return the application-wide Qt stylesheet string.
// Colors and font size are read from Params.
std::string GlobalQss();

}  // namespace utils

#endif  // PROJECT_INCLUDE_UTILS_H_
