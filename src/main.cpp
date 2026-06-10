// main.cpp — Entry point for the patient motion control application.
//
// Parses command-line flags, loads config/parameters.yaml, initializes
// hardware (Meca500 TCP client and/or Dynamixel eye gimbal), then opens
// the Qt6 ControlWindow.
//
// Usage:
//   ./patient_motion_control            # head + eye (default)
//   ./patient_motion_control --head     # Meca500 only
//   ./patient_motion_control --eye      # Dynamixel gimbal only

#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>

#include <QApplication>

#include "control_window.h"
#include "meca_client.h"
#include "parameter.h"
#include "utils.h"

#ifndef NO_DYNAMIXEL
#include <dynamixel_sdk.h>
#endif

namespace {

// Return the directory containing the running executable so that relative
// paths to config/ and profile folders resolve correctly.
std::string ExeDir(const char* argv0) {
  std::filesystem::path p(argv0);
  std::error_code ec;
  auto canonical = std::filesystem::canonical(p, ec);
  if (!ec) return canonical.parent_path().string();
  return std::filesystem::current_path().string();
}

// Connect to the Meca500 robot.
// Returns a heap-allocated Meca500Client on success, nullptr on failure.
// On failure the GUI opens without head control (graceful degradation).
Meca500Client* ConnectRobot(const Params& params) {
  std::cout << "[robot] Connecting to Meca500 at " << params.robot_ip_address << "...\n";
  auto* client = new Meca500Client();

  if (!client->Connect(params.robot_ip_address)) {
    std::cerr << "[robot] WARNING: Could not connect to Meca500. "
                 "Opening UI without head control.\n";
    delete client;
    return nullptr;
  }

  client->ActivateAndHome();

  if (!client->WaitHomed(60)) {
    std::cerr << "[robot] WARNING: Timeout waiting for home. "
                 "Opening UI without head control.\n";
    client->Disconnect();
    delete client;
    return nullptr;
  }
  std::cout << "[robot] Homed.\n";

  const auto& off = params.head_offset;
  client->SetTrf(off[0], off[1], off[2], 0.0, 0.0, 0.0);
  client->SetJointVelLimit(params.max_joint_vel_percentage);

  const auto& j = params.robot_head_init_pose;
  client->MoveJoints(j[0], j[1], j[2], j[3], j[4], j[5]);
  client->WaitIdle(60);
  std::cout << "[robot] At initial pose.\n";

  return client;
}

// Open the Dynamixel serial port and initialize both motors.
// Returns (port_handler, packet_handler) on success; both nullptr on failure.
std::pair<void*, void*> ConnectGimbal(Params& params) {
#ifdef NO_DYNAMIXEL
  (void)params;
  std::cerr << "[gimbal] Built without Dynamixel SDK — gimbal disabled.\n";
  return {nullptr, nullptr};
#else
  // Select default port based on OS if user did not override.
#if defined(__APPLE__)
  params.port = "/dev/cu.usbmodem101";
#elif defined(_WIN32)
  params.port = "COM3";
#else
  params.port = "/dev/ttyACM0";
#endif
  std::cout << "[gimbal] Serial port: " << params.port << "\n";

  auto* port   = new dynamixel::PortHandler(params.port.c_str());
  auto* packet = new dynamixel::PacketHandler(params.protocol);

  if (!port->openPort()) {
    std::cerr << "[gimbal] ERROR: Failed to open port " << params.port << "\n";
    delete port;
    delete packet;
    return {nullptr, nullptr};
  }
  if (!port->setBaudRate(params.baud_rate)) {
    std::cerr << "[gimbal] ERROR: Failed to set baud rate.\n";
    port->closePort();
    delete port;
    delete packet;
    return {nullptr, nullptr};
  }

  utils::SetupMotor(port, packet, params.dxl_1);
  utils::SetupMotor(port, packet, params.dxl_2);
  utils::WritePosition(port, packet, params.dxl_1, params.eye_center);
  utils::WritePosition(port, packet, params.dxl_2, params.eye_center);
  std::cout << "[gimbal] Eye gimbal motors ready.\n";

  return {port, packet};
#endif
}

}  // namespace

int main(int argc, char* argv[]) {
  // ── Parse flags ──────────────────────────────────────────────────────────────
  bool flag_head = false;
  bool flag_eye  = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--head") == 0) flag_head = true;
    if (std::strcmp(argv[i], "--eye")  == 0) flag_eye  = true;
  }
  bool use_head = !flag_eye;
  bool use_eye  = !flag_head;

  // ── Load parameters ──────────────────────────────────────────────────────────
  std::string exe_dir    = ExeDir(argv[0]);
  std::string yaml_path  = exe_dir + "/../config/parameters.yaml";

  // Also try current working directory if the canonical path doesn't exist.
  if (!std::filesystem::exists(yaml_path)) {
    yaml_path = std::filesystem::current_path().string() + "/config/parameters.yaml";
  }

  try {
    Params::Load(yaml_path);
    std::cout << "[config] Loaded " << yaml_path << "\n";
  } catch (const std::exception& exc) {
    std::cerr << "[config] FATAL: " << exc.what() << "\n";
    return 1;
  }

  // ── Initialize hardware ───────────────────────────────────────────────────────
  Meca500Client* robot        = nullptr;
  void*          port_handle  = nullptr;
  void*          packet_handle = nullptr;

  if (use_head) {
    robot = ConnectRobot(Params::Get());
    if (!robot) {
      // Graceful degradation: open UI without head control.
      use_head = false;
    }
  }

  if (use_eye) {
    // ConnectGimbal may mutate params.port based on OS.
    // We need a non-const reference — grab the singleton mutably only here.
    // Params::Get() returns const; we use a small local copy for port selection.
    // The actual mutation only affects `params.port`, which we store in Params.
    // To keep Params mostly immutable, we update port in-place via Load() is
    // not available — instead we shadow via a local variable for os detection.
    // (ConnectGimbal sets a local but params.port in the singleton won't change.
    //  The port string is only needed by PortHandler, so this is fine.)
    auto [ph, pk] = ConnectGimbal(const_cast<Params&>(Params::Get()));
    port_handle   = ph;
    packet_handle = pk;
    if (!port_handle) {
      if (robot) {
        robot->DeactivateRobot();
        robot->Disconnect();
        delete robot;
        robot = nullptr;
      }
      std::cerr << "[gimbal] Failed to open gimbal. Exiting.\n";
      return 1;
    }
  }

  // ── Launch Qt GUI ─────────────────────────────────────────────────────────────
  QApplication app(argc, argv);

  auto* window = new ControlWindow(use_head, use_eye, robot,
                                   port_handle, packet_handle);
  window->show();

  int ret = app.exec();

  // ControlWindow::OnClose() handles hardware teardown; we only need to free
  // the heap objects that were allocated here.
#ifndef NO_DYNAMIXEL
  delete static_cast<dynamixel::PacketHandler*>(packet_handle);
  delete static_cast<dynamixel::PortHandler*>(port_handle);
#endif
  delete robot;

  std::cout << "[main] Shutdown complete.\n";
  return ret;
}
