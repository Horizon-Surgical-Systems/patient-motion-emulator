// meca500_client.cpp — POSIX TCP client for the Meca500 robot controller.
//
// Connection: TCP to robot_ip:10000.
// Protocol: commands sent as null-terminated ASCII strings.
// Responses: lines containing "[CODE] message".
//
// The receiver thread runs until Disconnect() is called.  All WaitXxx() calls
// block the calling thread (must NOT be called from the Qt GUI thread).

#include "meca_client.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

// ─── Construction / destruction ───────────────────────────────────────────────

Meca500Client::Meca500Client()
    : socket_fd_(-1),
      connected_(false),
      stop_receiver_(false),
      homed_flag_(false),
      idle_flag_(false),
      joint_pos_ready_(false) {}

Meca500Client::~Meca500Client() {
  Disconnect();
}

// ─── Connect / Disconnect ─────────────────────────────────────────────────────

bool Meca500Client::Connect(const std::string& host) {
  socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "[Meca500] socket() failed: " << std::strerror(errno) << "\n";
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(10000);
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    std::cerr << "[Meca500] Invalid IP address: " << host << "\n";
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (::connect(socket_fd_,
                reinterpret_cast<sockaddr*>(&addr),
                sizeof(addr)) != 0) {
    std::cerr << "[Meca500] connect() failed: " << std::strerror(errno) << "\n";
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  connected_    = true;
  stop_receiver_ = false;
  homed_flag_   = false;
  idle_flag_    = false;
  joint_pos_ready_ = false;

  receiver_thread_ = std::thread(&Meca500Client::ReceiverLoop, this);
  std::cout << "[Meca500] Connected to " << host << ":10000\n";
  return true;
}

void Meca500Client::Disconnect() {
  stop_receiver_ = true;
  connected_     = false;

  if (socket_fd_ >= 0) {
    ::shutdown(socket_fd_, SHUT_RDWR);
    ::close(socket_fd_);
    socket_fd_ = -1;
  }

  // Wake up any blocking waits so their threads can exit cleanly.
  homed_cv_.notify_all();
  idle_cv_.notify_all();
  joint_pos_cv_.notify_all();

  if (receiver_thread_.joinable())
    receiver_thread_.join();
}

bool Meca500Client::IsConnected() const {
  return connected_.load();
}

// ─── Command helpers ──────────────────────────────────────────────────────────

void Meca500Client::SendCommand(const std::string& cmd) {
  if (!connected_) return;
  // Meca500 protocol: command string followed by a null terminator.
  std::string msg = cmd + '\0';
  ssize_t sent = ::send(socket_fd_, msg.c_str(), msg.size(), 0);
  if (sent < 0) {
    std::cerr << "[Meca500] send() failed: " << std::strerror(errno) << "\n";
  }
}

// ─── High-level commands ──────────────────────────────────────────────────────

void Meca500Client::ActivateRobot() {
  SendCommand("ActivateRobot()");
}

void Meca500Client::ActivateAndHome() {
  homed_flag_ = false;
  SendCommand("ActivateAndHome()");
}

bool Meca500Client::WaitHomed(int timeout_s) {
  std::unique_lock<std::mutex> lock(homed_mutex_);
  return homed_cv_.wait_for(
      lock,
      std::chrono::seconds(timeout_s),
      [this] { return homed_flag_.load() || !connected_.load(); });
}

void Meca500Client::SetTrf(double x, double y, double z,
                            double rx, double ry, double rz) {
  std::ostringstream oss;
  oss << "SetTrf(" << x << "," << y << "," << z << ","
      << rx << "," << ry << "," << rz << ")";
  SendCommand(oss.str());
}

void Meca500Client::SetJointVelLimit(double pct) {
  std::ostringstream oss;
  oss << "SetJointVelLimit(" << pct << ")";
  SendCommand(oss.str());
}

void Meca500Client::MoveJoints(double j1, double j2, double j3,
                                double j4, double j5, double j6) {
  idle_flag_ = false;
  std::ostringstream oss;
  oss << "MoveJoints(" << j1 << "," << j2 << "," << j3 << ","
      << j4 << "," << j5 << "," << j6 << ")";
  SendCommand(oss.str());
}

void Meca500Client::MoveLinRelTrf(double dx, double dy, double dz,
                                   double drx, double dry, double drz) {
  idle_flag_ = false;
  std::ostringstream oss;
  oss << "MoveLinRelTrf(" << dx << "," << dy << "," << dz << ","
      << drx << "," << dry << "," << drz << ")";
  SendCommand(oss.str());
}

void Meca500Client::ClearMotion() {
  SendCommand("ClearMotion()");
}

void Meca500Client::ResumeMotion() {
  SendCommand("ResumeMotion()");
}

bool Meca500Client::WaitIdle(int timeout_s) {
  std::unique_lock<std::mutex> lock(idle_mutex_);
  return idle_cv_.wait_for(
      lock,
      std::chrono::seconds(timeout_s),
      [this] { return idle_flag_.load() || !connected_.load(); });
}

std::array<double, 6> Meca500Client::GetRtTargetJointPos() {
  std::array<double, 6> joints{};

  if (!connected_) return joints;

  {
    std::lock_guard<std::mutex> lock(joint_pos_mutex_);
    joint_pos_ready_ = false;
    joint_pos_line_.clear();
  }

  SendCommand("GetRtTargetJointPos()");

  // Wait up to 3 seconds for the response.
  std::unique_lock<std::mutex> lock(joint_pos_mutex_);
  bool ok = joint_pos_cv_.wait_for(
      lock,
      std::chrono::seconds(3),
      [this] { return joint_pos_ready_.load() || !connected_.load(); });

  if (!ok || joint_pos_line_.empty()) return joints;

  // Parse "(j1,j2,j3,j4,j5,j6)" from the response line.
  auto start = joint_pos_line_.find('(');
  auto end   = joint_pos_line_.rfind(')');
  if (start == std::string::npos || end == std::string::npos) return joints;

  std::string inner = joint_pos_line_.substr(start + 1, end - start - 1);
  std::istringstream ss(inner);
  std::string token;
  int idx = 0;
  while (std::getline(ss, token, ',') && idx < 6) {
    try {
      joints[idx++] = std::stod(token);
    } catch (...) {
      // ignore parse error for this element
    }
  }
  return joints;
}

void Meca500Client::ResetError() {
  SendCommand("ResetError()");
}

void Meca500Client::DeactivateRobot() {
  SendCommand("DeactivateRobot()");
}

// ─── Receiver thread ──────────────────────────────────────────────────────────

void Meca500Client::ReceiverLoop() {
  std::string buf;
  char chunk[1024];

  while (!stop_receiver_) {
    ssize_t n = ::recv(socket_fd_, chunk, sizeof(chunk) - 1, 0);
    if (n <= 0) break;  // socket closed or error

    chunk[n] = '\0';
    buf += chunk;

    // Process all complete lines (terminated by '\0' or '\n').
    size_t pos = 0;
    while (pos < buf.size()) {
      size_t end = buf.find_first_of("\0\n", pos);
      if (end == std::string::npos) break;

      std::string line = buf.substr(pos, end - pos);
      pos = end + 1;

      if (!line.empty()) {
        // Extract numeric code from "[CODE]" at beginning of line.
        if (line.size() > 2 && line[0] == '[') {
          auto close = line.find(']');
          if (close != std::string::npos) {
            try {
              int code = std::stoi(line.substr(1, close - 1));
              HandleEventCode(code, line);
            } catch (...) {}
          }
        }
      }
    }
    buf = buf.substr(pos);
  }

  connected_ = false;
  // Wake all waiting threads.
  homed_cv_.notify_all();
  idle_cv_.notify_all();
  joint_pos_cv_.notify_all();
}

void Meca500Client::HandleEventCode(int code, const std::string& line) {
  switch (code) {
    case 2007:
      // Robot activated.
      std::cout << "[Meca500] Activated.\n";
      break;

    case 2012:
      // Robot homed.
      {
        std::lock_guard<std::mutex> lock(homed_mutex_);
        homed_flag_ = true;
      }
      homed_cv_.notify_all();
      std::cout << "[Meca500] Homed.\n";
      break;

    case 3004:
      // Motion queue cleared.
      std::cout << "[Meca500] Motion cleared.\n";
      break;

    case 3012:
      // End of motion — WaitIdle satisfied.
      {
        std::lock_guard<std::mutex> lock(idle_mutex_);
        idle_flag_ = true;
      }
      idle_cv_.notify_all();
      break;

    default:
      // Any line containing parentheses and commas may be a joint pos response.
      // Check for GetRtTargetJointPos reply: e.g. "[2026] (j1,j2,j3,j4,j5,j6)"
      if (line.find('(') != std::string::npos &&
          line.find(',') != std::string::npos &&
          line.find(')') != std::string::npos) {
        std::lock_guard<std::mutex> lock(joint_pos_mutex_);
        joint_pos_line_  = line;
        joint_pos_ready_ = true;
        joint_pos_cv_.notify_all();
      }
      break;
  }
}
