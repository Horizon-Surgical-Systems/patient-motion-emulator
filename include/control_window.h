#ifndef PROJECT_INCLUDE_CONTROL_WINDOW_H_
#define PROJECT_INCLUDE_CONTROL_WINDOW_H_

// Main Qt6 control window for the patient motion simulation system.
//
// Provides a dark-themed GUI with:
//   - Gimbal speed slider and WASD key-binding reference
//   - Eye motion profile playback (Bell's reflex, saccadic) with auto-rewind
//   - Head motion playback from IMU CSV files with complementary-filter fusion
//   - Breathing loop with interruptible cough / clear-throat presets
//   - TRF Cartesian jog buttons (X/Y/Z translation and UX/UY/UZ rotation)

#include <atomic>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <QDoubleSpinBox>
#include <QFrame>
#include <QLabel>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "meca_client.h"

class ControlWindow : public QMainWindow {
  Q_OBJECT

 public:
  // Construct the window.
  //   use_head     — whether the Meca500 head robot is available.
  //   use_eye      — whether the Dynamixel eye gimbal is available.
  //   robot        — owning pointer to the Meca500 client (may be nullptr).
  //   port_handle  — opaque Dynamixel PortHandler* (may be nullptr).
  //   packet_handle — opaque Dynamixel PacketHandler* (may be nullptr).
  explicit ControlWindow(bool use_head,
                         bool use_eye,
                         Meca500Client* robot,
                         void* port_handle,
                         void* packet_handle,
                         QWidget* parent = nullptr);

  ~ControlWindow() override;

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void closeEvent(QCloseEvent* event) override;

 private slots:  // NOLINT(whitespace/indent)
  // Main 50 Hz timer tick.
  void OnTick();
  // Jog repeat timer tick.
  void OnRepeatTrfJog();
  // Gimbal speed slider value changed.
  void OnSpeedChanged(int value);

 private:
  // ─── UI construction ────────────────────────────────────────────────────────
  void BuildUi();
  void BuildGimbalSpeedCard(QWidget* parent);
  void BuildKeyBindingsCard(QWidget* parent);
  void BuildEyeProfilesCard(QWidget* parent);
  void BuildMotionPlaybackCard(QWidget* parent);
  void BuildTrfJogCard(QWidget* parent);

  // Widget factories — create and style but do not insert into any layout.
  QFrame* MakeCardFrame(QWidget* parent);
  QLabel* MakeLabel(const std::string& text, int size = -1,
                    bool bold = false, const std::string& color = "");
  QPushButton* MakeButton(const std::string& text,
                          const std::function<void()>& callback,
                          const std::string& color,
                          int min_width = 80);
  QPushButton* MakeJogButton(const std::string& label,
                              const std::string& axis, double sign);
  void AddSpacer(QVBoxLayout* layout, int height = 6);

  // ─── UI state helpers ────────────────────────────────────────────────────────
  void SetIdleUi();
  void SetBusyUi();
  void SetBreathingUi();
  void UpdateVelLabel();

  // ─── Breathing loop ──────────────────────────────────────────────────────────
  void StartBreathing();
  void StopBreathing();
  void StartInterruption(const std::string& keyword);
  void ResumeBreathingFromSaved();

  // ─── Head motion playback ────────────────────────────────────────────────────
  void PlayPreset(const std::string& keyword);
  void BrowseFile();
  void LoadFile(const std::string& path);
  void StartPlayback();
  void StopPlayback();
  void OnStopPressed();
  void StartHeadRewind();
  void WaitHeadReturn();     // Runs in background thread; sets head_return_done_.

  // ─── Control loop ticks ──────────────────────────────────────────────────────
  void TickHeadPlayback();
  void TickGimbalWasd(int step);
  void TickGimbalReset(int step);
  void TickEyeProfile();

  // ─── Eye motion profiles ─────────────────────────────────────────────────────
  std::string FindEyeProfile(const std::string& keyword) const;
  std::vector<std::tuple<double, int, int>> LoadEyeProfile(
      const std::string& keyword);
  void StartEyeProfile(const std::string& keyword);
  void StartEyeRewind();

  // ─── TRF Cartesian jog ───────────────────────────────────────────────────────
  void SendTrfStep(const std::string& axis, double sign);
  void StartTrfJog(const std::string& axis, double sign);
  void StopTrfJog();
  void GoInitPose();
  void SetTrfHome();
  void GoTrfHome();
  void ResetRobotError();
  void GetRobotPose();

  // ─── Shutdown ────────────────────────────────────────────────────────────────
  void OnClose();

  // ─── Hardware ────────────────────────────────────────────────────────────────
  bool use_head_;
  bool use_eye_;
  Meca500Client* robot_;    // not owned; passed in from main
  void* port_handle_;       // Dynamixel PortHandler*
  void* packet_handle_;     // Dynamixel PacketHandler*

  // ─── Gimbal state ────────────────────────────────────────────────────────────
  int pos1_;
  int pos2_;
  bool key_up_;
  bool key_down_;
  bool key_left_;
  bool key_right_;
  bool gimbal_resetting_;

  // ─── Eye profile state ───────────────────────────────────────────────────────
  std::vector<std::tuple<double, int, int>> eye_profile_data_;
  int    eye_play_idx_;
  double eye_play_start_;
  bool   eye_playing_;
  bool   eye_rewinding_;
  std::vector<std::pair<int, int>> eye_rewind_steps_;
  int    eye_rewind_idx_;

  // ─── Head playback state ─────────────────────────────────────────────────────
  // Each sample: (t_seconds, pitch_deg, roll_deg)
  std::vector<std::tuple<double, double, double>> head_playback_data_;
  int    head_play_idx_;
  bool   is_playing_;
  double playback_start_;
  double prev_pitch_;
  double prev_roll_;
  bool   head_rewinding_;
  std::atomic<bool> head_return_done_;
  std::thread rewind_thread_;

  // ─── Breathing state ─────────────────────────────────────────────────────────
  bool   breathing_;
  bool   resume_breath_;
  int    breath_resume_idx_;
  double breath_resume_pitch_;
  double breath_resume_roll_;
  double breath_resume_t_;

  // ─── TRF jog state ───────────────────────────────────────────────────────────
  std::string          trf_jog_axis_;
  double               trf_jog_sign_;
  std::array<double,6> cart_pos_;
  std::array<double,6> home_cart_offset_;
  std::array<double,6> head_home_joints_;

  // ─── Timers ──────────────────────────────────────────────────────────────────
  QTimer* timer_;
  QTimer* jog_timer_;

  // ─── UI widgets ──────────────────────────────────────────────────────────────
  QSlider*        gimbal_speed_slider_;
  QLabel*         slider_value_label_;
  QLabel*         vel_label_;

  QPushButton*    bells_btn_;
  QPushButton*    saccadic_btn_;
  QProgressBar*   eye_progress_bar_;
  QLabel*         eye_progress_label_;

  QPushButton*    breath_start_btn_;
  QPushButton*    stop_breath_btn_;
  std::vector<QPushButton*> interrupt_btns_;

  QLabel*         file_label_;
  QLabel*         info_label_;
  QPushButton*    play_btn_;
  QPushButton*    stop_btn_;
  QProgressBar*   progress_bar_;
  QLabel*         progress_label_;

  QDoubleSpinBox* trf_lin_step_;
  QDoubleSpinBox* trf_ang_step_;
  QLabel*         pose_label_;
};

#endif  // PROJECT_INCLUDE_CONTROL_WINDOW_H_
