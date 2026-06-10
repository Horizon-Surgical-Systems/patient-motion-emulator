#ifndef PROJECT_INCLUDE_MECA_CLIENT_H_
#define PROJECT_INCLUDE_MECA_CLIENT_H_

// POSIX TCP client for the Meca500 robot controller.
//
// The Meca500 accepts plain-text commands on port 10000 and responds with
// lines of the form "[CODE] message".  Key event codes used here:
//   2007  — robot activated
//   2012  — robot homed
//   3004  — motion queue cleared
//   3012  — end of motion / WaitIdle satisfied
//
// Commands are sent as null-terminated strings, e.g. "MoveJoints(0,-30,30,90,-90,0)\0".
//
// All blocking waits (WaitHomed, WaitIdle) block the calling thread and use
// std::condition_variable internally; they must NOT be called from the Qt
// main/GUI thread.  Use a std::thread (daemon) for those calls.

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

class Meca500Client {
 public:
  Meca500Client();
  ~Meca500Client();

  // Prevent copying.
  Meca500Client(const Meca500Client&) = delete;
  Meca500Client& operator=(const Meca500Client&) = delete;

  // Open TCP connection to host:10000.
  // Returns true on success, false on failure.
  bool Connect(const std::string& host);

  // Flush any pending motion and close the socket.
  void Disconnect();

  // Returns true if the socket is currently open.
  bool IsConnected() const;

  // Send "ActivateRobot()" command (does not wait for completion).
  void ActivateRobot();

  // Send "ActivateAndHome()" command (does not wait for completion).
  void ActivateAndHome();

  // Block until event code 2012 (homed) is received, or until timeout_s
  // seconds elapse.  Returns true if homed, false on timeout.
  bool WaitHomed(int timeout_s = 60);

  // Set the Tool Reference Frame offset [x, y, z, rx, ry, rz].
  void SetTrf(double x, double y, double z,
              double rx, double ry, double rz);

  // Set the joint velocity limit to pct percent of the maximum.
  void SetJointVelLimit(double pct);

  // Move to absolute joint angles (degrees).
  void MoveJoints(double j1, double j2, double j3,
                  double j4, double j5, double j6);

  // Move relative to the current TRF pose.
  void MoveLinRelTrf(double dx, double dy, double dz,
                     double drx, double dry, double drz);

  // Clear the motion queue (pauses motion; call ResumeMotion afterwards).
  void ClearMotion();

  // Resume motion after ClearMotion.
  void ResumeMotion();

  // Block until event code 3012 (end of motion) is received, or timeout_s
  // seconds elapse.  Returns true on success, false on timeout.
  bool WaitIdle(int timeout_s = 30);

  // Query the robot's current target joint positions.
  // Returns an array of 6 joint angles in degrees, or {0} on failure.
  std::array<double, 6> GetRtTargetJointPos();

  // Send "ResetError()" command.
  void ResetError();

  // Send "DeactivateRobot()" command.
  void DeactivateRobot();

 private:
  // Send a null-terminated command string over the socket.
  void SendCommand(const std::string& cmd);

  // Background thread that reads lines from the socket and parses event codes.
  void ReceiverLoop();

  // Called by ReceiverLoop when an event code is received.
  void HandleEventCode(int code, const std::string& line);

  int socket_fd_;
  std::atomic<bool> connected_;
  std::atomic<bool> stop_receiver_;

  std::thread receiver_thread_;

  // Synchronization for WaitHomed (code 2012).
  std::mutex homed_mutex_;
  std::condition_variable homed_cv_;
  std::atomic<bool> homed_flag_;

  // Synchronization for WaitIdle (code 3012).
  std::mutex idle_mutex_;
  std::condition_variable idle_cv_;
  std::atomic<bool> idle_flag_;

  // For GetRtTargetJointPos: capture the response line.
  std::mutex joint_pos_mutex_;
  std::condition_variable joint_pos_cv_;
  std::atomic<bool> joint_pos_ready_;
  std::string joint_pos_line_;
};

#endif  // PROJECT_INCLUDE_MECA_CLIENT_H_
