// control_window.cpp — Qt6 main control window implementation.
//
// Ports ControlWindow.py 1-for-1.  Every method documented in control_window.h
// is fully implemented here.  Placeholder stubs and TODOs are absent.

#include "control_window.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <QApplication>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFrame>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>
#include <QSizePolicy>
#include <QSlider>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "parameter.h"
#include "utils.h"

#ifndef NO_DYNAMIXEL
#include <dynamixel_sdk.h>
#endif

namespace {

// Return seconds elapsed since an arbitrary epoch using a monotonic clock.
double Now() {
  using namespace std::chrono;
  static const auto kEpoch = steady_clock::now();
  return duration<double>(steady_clock::now() - kEpoch).count();
}

// Glob helper: find all files in folder whose name contains keyword and has one
// of the given extensions.  Returns paths sorted (lexicographic), so we can
// call back() to pick the "latest" (highest name) just like Python's max().
std::vector<std::string> GlobFiles(const std::string& folder,
                                   const std::string& keyword,
                                   const std::vector<std::string>& exts) {
  std::vector<std::string> results;
  try {
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
      if (!entry.is_regular_file()) continue;
      const std::string name = entry.path().filename().string();
      if (name.find(keyword) == std::string::npos) continue;
      const std::string ext = entry.path().extension().string();
      for (const auto& e : exts) {
        if (ext == e) {
          results.push_back(entry.path().string());
          break;
        }
      }
    }
  } catch (...) {}
  std::sort(results.begin(), results.end());
  return results;
}

// Return the directory that contains the application executable.
// On macOS / Linux this gives the project root when run from there, or we fall
// back to the current working directory if that is unavailable.
std::string AppDir() {
  // Use the location of the running executable as a hint.
  // std::filesystem::current_path() is simpler and usually correct when the
  // binary is run from the project root.
  return std::filesystem::current_path().string();
}

}  // namespace

// ─── Constructor ──────────────────────────────────────────────────────────────

ControlWindow::ControlWindow(bool use_head,
                              bool use_eye,
                              Meca500Client* robot,
                              void* port_handle,
                              void* packet_handle,
                              QWidget* parent)
    : QMainWindow(parent),
      use_head_(use_head),
      use_eye_(use_eye),
      robot_(robot),
      port_handle_(port_handle),
      packet_handle_(packet_handle),
      pos1_(Params::Get().eye_center),
      pos2_(Params::Get().eye_center),
      key_up_(false),
      key_down_(false),
      key_left_(false),
      key_right_(false),
      gimbal_resetting_(false),
      eye_play_idx_(0),
      eye_play_start_(0.0),
      eye_playing_(false),
      eye_rewinding_(false),
      eye_rewind_idx_(0),
      head_play_idx_(0),
      is_playing_(false),
      playback_start_(0.0),
      prev_pitch_(0.0),
      prev_roll_(0.0),
      head_rewinding_(false),
      head_return_done_(false),
      breathing_(false),
      resume_breath_(false),
      breath_resume_idx_(0),
      breath_resume_pitch_(0.0),
      breath_resume_roll_(0.0),
      breath_resume_t_(0.0),
      trf_jog_axis_("x"),
      trf_jog_sign_(1.0),
      gimbal_speed_slider_(nullptr),
      slider_value_label_(nullptr),
      vel_label_(nullptr),
      bells_btn_(nullptr),
      saccadic_btn_(nullptr),
      eye_progress_bar_(nullptr),
      eye_progress_label_(nullptr),
      breath_start_btn_(nullptr),
      stop_breath_btn_(nullptr),
      file_label_(nullptr),
      info_label_(nullptr),
      play_btn_(nullptr),
      stop_btn_(nullptr),
      progress_bar_(nullptr),
      progress_label_(nullptr),
      trf_lin_step_(nullptr),
      trf_ang_step_(nullptr),
      pose_label_(nullptr) {
  cart_pos_.fill(0.0);
  home_cart_offset_.fill(0.0);
  const auto& init = Params::Get().robot_head_init_pose;
  for (int i = 0; i < 6; ++i) head_home_joints_[i] = init[i];

  setFocusPolicy(Qt::StrongFocus);

  BuildUi();

  timer_ = new QTimer(this);
  timer_->setInterval(1000 / Params::Get().loop_hz);
  connect(timer_, &QTimer::timeout, this, &ControlWindow::OnTick);
  timer_->start();

  jog_timer_ = new QTimer(this);
  connect(jog_timer_, &QTimer::timeout, this, &ControlWindow::OnRepeatTrfJog);
}

ControlWindow::~ControlWindow() {
  OnClose();
}

// ─── Key events ───────────────────────────────────────────────────────────────

void ControlWindow::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_W: key_up_    = true;  break;
    case Qt::Key_S: key_down_  = true;  break;
    case Qt::Key_A: key_left_  = true;  break;
    case Qt::Key_D: key_right_ = true;  break;
    case Qt::Key_Q:
    case Qt::Key_Escape:
      close();
      break;
    default:
      QMainWindow::keyPressEvent(event);
  }
}

void ControlWindow::keyReleaseEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_W: key_up_    = false; break;
    case Qt::Key_S: key_down_  = false; break;
    case Qt::Key_A: key_left_  = false; break;
    case Qt::Key_D: key_right_ = false; break;
    default:
      QMainWindow::keyReleaseEvent(event);
  }
}

// ─── Close event ──────────────────────────────────────────────────────────────

void ControlWindow::closeEvent(QCloseEvent* event) {
  OnClose();
  event->accept();
}

void ControlWindow::OnClose() {
  timer_->stop();
  jog_timer_->stop();

  eye_playing_   = false;
  eye_rewinding_ = false;

  if (is_playing_) StopPlayback();

  // Join the rewind thread before destroying hardware handles.
  if (rewind_thread_.joinable()) rewind_thread_.join();

  if (use_eye_ && port_handle_) {
    const Params& p = Params::Get();
    utils::DisableMotor(port_handle_, packet_handle_, p.dxl_1);
    utils::DisableMotor(port_handle_, packet_handle_, p.dxl_2);
#ifndef NO_DYNAMIXEL
    static_cast<dynamixel::PortHandler*>(port_handle_)->closePort();
#endif
    std::cout << "[gimbal] Port closed.\n";
  }

  if (use_head_ && robot_) {
    try {
      robot_->ClearMotion();
      robot_->DeactivateRobot();
      robot_->Disconnect();
      std::cout << "[robot] Disconnected.\n";
    } catch (...) {}
  }
}

// ─── Widget factory helpers ───────────────────────────────────────────────────

QFrame* ControlWindow::MakeCardFrame(QWidget* parent) {
  QFrame* card = new QFrame(parent);
  card->setObjectName("card");
  return card;
}

QLabel* ControlWindow::MakeLabel(const std::string& text, int size,
                                   bool bold, const std::string& color) {
  const Params& p = Params::Get();
  int fs = (size > 0) ? size : p.ui_font_size;
  std::string c = color.empty() ? p.ui_fg_color : color;
  std::string weight = bold ? "bold" : "normal";

  QLabel* lbl = new QLabel(QString::fromStdString(text));
  lbl->setStyleSheet(QString::fromStdString(
      "color: " + c + "; font-size: " + std::to_string(fs) +
      "pt; font-weight: " + weight + "; background: transparent;"));
  return lbl;
}

QPushButton* ControlWindow::MakeButton(const std::string& text,
                                        const std::function<void()>& callback,
                                        const std::string& color,
                                        int min_width) {
  QPushButton* btn = new QPushButton(QString::fromStdString(text));
  btn->setStyleSheet(QString::fromStdString(utils::BtnQss(color)));
  btn->setMinimumWidth(min_width);
  btn->setCursor(Qt::PointingHandCursor);
  connect(btn, &QPushButton::clicked, this, [callback](bool) { callback(); });
  return btn;
}

QPushButton* ControlWindow::MakeJogButton(const std::string& label,
                                           const std::string& axis,
                                           double sign) {
  QPushButton* btn = new QPushButton(QString::fromStdString(label));
  btn->setStyleSheet(
      QString::fromStdString(utils::BtnQss(Params::Get().ui_btn_gray)));
  btn->setMinimumWidth(90);
  btn->setCursor(Qt::PointingHandCursor);
  // Capture axis and sign by value.
  connect(btn, &QPushButton::pressed, this,
          [this, axis, sign]() { StartTrfJog(axis, sign); });
  connect(btn, &QPushButton::released, this,
          [this]() { StopTrfJog(); });
  return btn;
}

void ControlWindow::AddSpacer(QVBoxLayout* layout, int height) {
  QWidget* spacer = new QWidget();
  spacer->setFixedHeight(height);
  layout->addWidget(spacer);
}

// ─── UI construction ──────────────────────────────────────────────────────────

void ControlWindow::BuildUi() {
  const Params& p = Params::Get();

  setWindowTitle("Patient Motion Control");
  setWindowFlag(Qt::WindowStaysOnTopHint);
  setStyleSheet(QString::fromStdString(utils::GlobalQss()));

  QWidget* central = new QWidget(this);
  setCentralWidget(central);
  QVBoxLayout* root_layout = new QVBoxLayout(central);
  root_layout->setContentsMargins(0, 0, 0, 8);
  root_layout->setSpacing(0);

  // ── Header banner ───────────────────────────────────────────────────────────
  QFrame* header = new QFrame();
  header->setStyleSheet(
      QString::fromStdString("background-color: " + std::string(p.ui_accent_color) +
                             "; border: none;"));
  QVBoxLayout* h_layout = new QVBoxLayout(header);
  h_layout->setContentsMargins(12, 12, 12, 12);
  h_layout->setSpacing(2);

  std::string parts;
  if (use_head_) {
    parts += "Head  (Meca500)";
  }
  if (use_eye_) {
    if (!parts.empty()) parts += "  +  ";
    parts += "Eye  (Dynamixel)";
  }
  QLabel* sub_lbl = new QLabel(QString::fromStdString(parts));
  sub_lbl->setAlignment(Qt::AlignCenter);
  sub_lbl->setStyleSheet("color: #ddd6fe; font-size: 12pt; background: transparent;");
  h_layout->addWidget(sub_lbl);
  root_layout->addWidget(header);

  // ── Two-column body ─────────────────────────────────────────────────────────
  QWidget* body = new QWidget();
  QHBoxLayout* body_layout = new QHBoxLayout(body);
  body_layout->setContentsMargins(0, 0, 0, 0);
  body_layout->setSpacing(0);
  root_layout->addWidget(body);

  QWidget* left = new QWidget();
  left->setFixedWidth(p.ui_left_col_width);
  QVBoxLayout* left_layout = new QVBoxLayout(left);
  left_layout->setAlignment(Qt::AlignTop);
  left_layout->setContentsMargins(4, 4, 4, 4);
  left_layout->setSpacing(8);

  QWidget* right = new QWidget();
  QVBoxLayout* right_layout = new QVBoxLayout(right);
  right_layout->setAlignment(Qt::AlignTop);
  right_layout->setContentsMargins(4, 4, 4, 4);
  right_layout->setSpacing(8);

  body_layout->addWidget(left);
  body_layout->addWidget(right, 1);

  if (use_eye_)  BuildGimbalSpeedCard(left);
  BuildKeyBindingsCard(left);
  if (use_eye_)  BuildEyeProfilesCard(left);
  if (use_head_) BuildMotionPlaybackCard(left);
  if (use_head_) BuildTrfJogCard(right);

  adjustSize();
  setFixedSize(sizeHint());
}

void ControlWindow::BuildGimbalSpeedCard(QWidget* parent) {
  const Params& p = Params::Get();

  QFrame* card = MakeCardFrame(parent);
  QVBoxLayout* layout = new QVBoxLayout(card);
  layout->setContentsMargins(14, 10, 14, 10);
  layout->setSpacing(4);
  parent->layout()->addWidget(card);

  layout->addWidget(MakeLabel("Gimbal Speed", p.ui_font_size + 1, true));
  AddSpacer(layout);

  QWidget* row = new QWidget();
  QHBoxLayout* row_layout = new QHBoxLayout(row);
  row_layout->setContentsMargins(0, 0, 0, 0);
  row_layout->addWidget(MakeLabel("Step", -1, false, p.ui_dim_color));
  row_layout->addStretch();
  row_layout->addWidget(MakeLabel("counts / tick", -1, false, p.ui_dim_color));
  layout->addWidget(row);

  QWidget* slider_row = new QWidget();
  QHBoxLayout* sr_layout = new QHBoxLayout(slider_row);
  sr_layout->setContentsMargins(0, 0, 0, 0);
  sr_layout->setSpacing(8);

  gimbal_speed_slider_ = new QSlider(Qt::Horizontal);
  gimbal_speed_slider_->setMinimum(1);
  gimbal_speed_slider_->setMaximum(90);
  gimbal_speed_slider_->setValue(p.step_size);
  gimbal_speed_slider_->setMinimumWidth(200);

  slider_value_label_ = new QLabel(QString::number(p.step_size));
  slider_value_label_->setStyleSheet(
      QString::fromStdString("color: " + p.ui_fg_color +
                             "; font-size: 9pt; background: transparent;"
                             " min-width: 24px;"));
  slider_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  sr_layout->addWidget(gimbal_speed_slider_);
  sr_layout->addWidget(slider_value_label_);
  layout->addWidget(slider_row);

  vel_label_ = MakeLabel("", -1, false, p.ui_dim_color);
  layout->addWidget(vel_label_);
  UpdateVelLabel();

  connect(gimbal_speed_slider_, &QSlider::valueChanged,
          this, &ControlWindow::OnSpeedChanged);

  AddSpacer(layout);
  QPushButton* reset_btn = MakeButton(
      "Gimbal Reset",
      [this]() {
        if (!port_handle_ || eye_playing_ || eye_rewinding_) return;
        gimbal_resetting_ = true;
      },
      Params::Get().ui_btn_gray, 110);
  layout->addWidget(reset_btn, 0, Qt::AlignLeft);
}

void ControlWindow::BuildKeyBindingsCard(QWidget* parent) {
  const Params& p = Params::Get();

  QFrame* card = MakeCardFrame(parent);
  QVBoxLayout* layout = new QVBoxLayout(card);
  layout->setContentsMargins(14, 10, 14, 10);
  layout->setSpacing(4);
  parent->layout()->addWidget(card);

  layout->addWidget(MakeLabel("Key Bindings", p.ui_font_size + 1, true));
  AddSpacer(layout);

  struct Binding { std::string keys; std::string action; std::string color; };
  std::vector<Binding> bindings;
  if (use_eye_) {
    bindings.push_back({"W / S",    "Eye  Superior / Inferior", p.ui_accent_color});
    bindings.push_back({"A / D",    "Eye  Temporal / Nasal",    p.ui_accent_color});
  }
  bindings.push_back({"Q / ESC", "Quit", p.ui_btn_red});

  for (const auto& b : bindings) {
    QWidget* row = new QWidget();
    QHBoxLayout* row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(0, 0, 0, 0);
    row_layout->addWidget(MakeLabel(b.keys,   -1, true,  b.color));
    row_layout->addStretch();
    row_layout->addWidget(MakeLabel(b.action, -1, false, p.ui_fg_color));
    layout->addWidget(row);
  }
}

void ControlWindow::BuildEyeProfilesCard(QWidget* parent) {
  const Params& p = Params::Get();

  QFrame* card = MakeCardFrame(parent);
  QVBoxLayout* layout = new QVBoxLayout(card);
  layout->setContentsMargins(14, 10, 14, 10);
  layout->setSpacing(4);
  parent->layout()->addWidget(card);

  layout->addWidget(MakeLabel("Eye Motion Profiles", p.ui_font_size + 1, true));
  AddSpacer(layout);

  QWidget* btn_row = new QWidget();
  QHBoxLayout* btn_layout = new QHBoxLayout(btn_row);
  btn_layout->setContentsMargins(0, 0, 0, 0);
  btn_layout->setSpacing(6);

  bells_btn_ = MakeButton("Bell's Reflex",
                           [this]() { StartEyeProfile("bells"); },
                           p.ui_btn_purple);
  saccadic_btn_ = MakeButton("Saccadic",
                              [this]() { StartEyeProfile("saccadic"); },
                              p.ui_btn_purple);
  bells_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  saccadic_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  btn_layout->addWidget(bells_btn_);
  btn_layout->addWidget(saccadic_btn_);
  layout->addWidget(btn_row);

  eye_progress_bar_ = new QProgressBar();
  eye_progress_bar_->setMaximum(100);
  eye_progress_bar_->setValue(0);
  eye_progress_bar_->setTextVisible(false);
  layout->addWidget(eye_progress_bar_);

  eye_progress_label_ = MakeLabel("Ready", -1, false, p.ui_dim_color);
  layout->addWidget(eye_progress_label_);
}

void ControlWindow::BuildMotionPlaybackCard(QWidget* parent) {
  const Params& p = Params::Get();

  QFrame* card = MakeCardFrame(parent);
  QVBoxLayout* layout = new QVBoxLayout(card);
  layout->setContentsMargins(14, 10, 14, 10);
  layout->setSpacing(4);
  parent->layout()->addWidget(card);

  layout->addWidget(MakeLabel("Motion Playback", p.ui_font_size + 1, true));
  AddSpacer(layout);

  // ── Breathing section ───────────────────────────────────────────────────────
  layout->addWidget(MakeLabel("Breathing", -1, false, p.ui_dim_color));

  breath_start_btn_ = MakeButton(
      "\xe2\x96\xb6  Breathe",   // ▶
      [this]() { StartBreathing(); },
      p.ui_btn_green);
  breath_start_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  layout->addWidget(breath_start_btn_);

  stop_breath_btn_ = MakeButton(
      "\xe2\x96\xa0  Stop Breathing",   // ■
      [this]() { StopBreathing(); },
      p.ui_btn_red);
  stop_breath_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  stop_breath_btn_->setEnabled(false);
  layout->addWidget(stop_breath_btn_);

  AddSpacer(layout);

  // ── Interruptions section ───────────────────────────────────────────────────
  layout->addWidget(MakeLabel("Interruptions", -1, false, p.ui_dim_color));

  struct Preset { std::string label; std::string keyword; };
  std::vector<Preset> presets = {
      {"Cough",        p.head_cough_profile},
      {"Clear Throat", p.head_clear_throat_profile},
  };
  for (const auto& pr : presets) {
    QPushButton* btn = MakeButton(
        pr.label,
        [this, kw = pr.keyword]() { PlayPreset(kw); },
        p.ui_btn_purple);
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    layout->addWidget(btn);
    interrupt_btns_.push_back(btn);
  }

  AddSpacer(layout);

  // ── File picker ─────────────────────────────────────────────────────────────
  QWidget* file_row = new QWidget();
  QHBoxLayout* fl_layout = new QHBoxLayout(file_row);
  fl_layout->setContentsMargins(0, 0, 0, 0);
  file_label_ = MakeLabel("No file loaded", -1, false, p.ui_dim_color);
  fl_layout->addWidget(file_label_, 1);
  QPushButton* browse_btn = MakeButton(
      "Browse\xe2\x80\xa6",   // Browse…
      [this]() { BrowseFile(); },
      p.ui_btn_purple, 70);
  fl_layout->addWidget(browse_btn);
  layout->addWidget(file_row);

  info_label_ = MakeLabel("", -1, false, p.ui_dim_color);
  info_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  layout->addWidget(info_label_);

  // ── Play / Stop ─────────────────────────────────────────────────────────────
  QWidget* ctrl_row = new QWidget();
  QHBoxLayout* ctrl_layout = new QHBoxLayout(ctrl_row);
  ctrl_layout->setContentsMargins(0, 0, 0, 0);
  ctrl_layout->setSpacing(6);

  play_btn_ = MakeButton(
      "\xe2\x96\xb6  Play",   // ▶
      [this]() { StartPlayback(); },
      p.ui_btn_green, 80);
  play_btn_->setEnabled(false);

  stop_btn_ = MakeButton(
      "\xe2\x96\xa0  Stop",   // ■
      [this]() { OnStopPressed(); },
      p.ui_btn_red, 80);
  stop_btn_->setEnabled(false);

  ctrl_layout->addWidget(play_btn_);
  ctrl_layout->addWidget(stop_btn_);
  ctrl_layout->addStretch();
  layout->addWidget(ctrl_row);

  progress_bar_ = new QProgressBar();
  progress_bar_->setMaximum(100);
  progress_bar_->setValue(0);
  progress_bar_->setTextVisible(false);
  layout->addWidget(progress_bar_);

  progress_label_ = MakeLabel("", -1, false, p.ui_dim_color);
  progress_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  layout->addWidget(progress_label_);
}

void ControlWindow::BuildTrfJogCard(QWidget* parent) {
  const Params& p = Params::Get();

  QFrame* card = MakeCardFrame(parent);
  QVBoxLayout* layout = new QVBoxLayout(card);
  layout->setContentsMargins(14, 10, 14, 10);
  layout->setSpacing(4);
  parent->layout()->addWidget(card);

  layout->addWidget(MakeLabel("TRF Cartesian Jog", p.ui_font_size + 1, true));
  AddSpacer(layout);

  QPushButton* go_init_btn = MakeButton(
      "Go Init", [this]() { GoInitPose(); }, p.ui_btn_purple);
  go_init_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  layout->addWidget(go_init_btn);
  AddSpacer(layout);

  // Step-size spinboxes.
  struct StepRow { std::string label; std::string unit; QDoubleSpinBox** field; };
  std::vector<StepRow> step_rows = {
      {"Linear step",  "mm", &trf_lin_step_},
      {"Angular step", "\xc2\xb0", &trf_ang_step_},  // °
  };
  for (auto& sr : step_rows) {
    QWidget* row = new QWidget();
    QHBoxLayout* row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(0, 2, 0, 2);
    row_layout->addWidget(MakeLabel(sr.label, -1, false, p.ui_dim_color));
    row_layout->addStretch();
    row_layout->addWidget(MakeLabel(sr.unit, -1, false, p.ui_dim_color));
    QDoubleSpinBox* spin = new QDoubleSpinBox();
    spin->setRange(0.1, 50.0);
    spin->setSingleStep(0.5);
    spin->setValue(1.0);
    spin->setFixedWidth(65);
    row_layout->addWidget(spin);
    *sr.field = spin;
    layout->addWidget(row);
  }

  // Translation jog rows.
  layout->addWidget(MakeLabel("Translation", -1, true, p.ui_dim_color));
  struct JogRow { std::string axis; std::string neg_lbl; std::string pos_lbl; };
  std::vector<JogRow> tran_rows = {
      {"x",  "Nasal",    "Temporal"},
      {"y",  "Down",     "Up"},
      {"z",  "Inferior", "Superior"},
  };
  for (const auto& jr : tran_rows) {
    QWidget* row = new QWidget();
    QHBoxLayout* row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(0, 3, 0, 3);
    row_layout->addWidget(MakeJogButton("\xe2\x86\x90 " + jr.neg_lbl, jr.axis, -1.0));
    row_layout->addStretch();
    std::string axis_upper = jr.axis;
    std::transform(axis_upper.begin(), axis_upper.end(), axis_upper.begin(),
                   [](unsigned char c) { return std::toupper(c); });
    QLabel* axis_lbl = MakeLabel(axis_upper, p.ui_font_size + 1, true, p.ui_accent_color);
    axis_lbl->setAlignment(Qt::AlignCenter);
    row_layout->addWidget(axis_lbl);
    row_layout->addStretch();
    row_layout->addWidget(MakeJogButton(jr.pos_lbl + " \xe2\x86\x92", jr.axis, +1.0));
    layout->addWidget(row);
  }

  // Rotation jog rows.
  layout->addWidget(MakeLabel("Rotation", -1, true, p.ui_dim_color));
  std::vector<JogRow> rot_rows = {
      {"ux", "Inferior", "Superior"},
      {"uy", "Left",     "Right"},
      {"uz", "Temporal", "Nasal"},
  };
  for (const auto& jr : rot_rows) {
    QWidget* row = new QWidget();
    QHBoxLayout* row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(0, 3, 0, 3);
    row_layout->addWidget(MakeJogButton("\xe2\x86\x90 " + jr.neg_lbl, jr.axis, -1.0));
    row_layout->addStretch();
    QLabel* axis_lbl = MakeLabel(jr.axis, p.ui_font_size + 1, true, p.ui_accent_color);
    axis_lbl->setAlignment(Qt::AlignCenter);
    row_layout->addWidget(axis_lbl);
    row_layout->addStretch();
    row_layout->addWidget(MakeJogButton(jr.pos_lbl + " \xe2\x86\x92", jr.axis, +1.0));
    layout->addWidget(row);
  }

  AddSpacer(layout);

  // Utility buttons.
  struct UtilBtn { std::string text; std::function<void()> cmd; std::string color; };
  std::vector<UtilBtn> util_btns = {
      {"Set Home",       [this]() { SetTrfHome(); },       p.ui_btn_purple},
      {"Go Home",        [this]() { GoTrfHome(); },        p.ui_btn_green},
      {"Reset Error",    [this]() { ResetRobotError(); },  p.ui_btn_red},
      {"Get Robot Pose", [this]() { GetRobotPose(); },     p.ui_btn_gray},
  };
  for (const auto& ub : util_btns) {
    QPushButton* btn = MakeButton(ub.text, ub.cmd, ub.color, 90);
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    layout->addWidget(btn);
  }

  pose_label_ = MakeLabel("", -1, false, p.ui_dim_color);
  pose_label_->setWordWrap(true);
  pose_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  layout->addWidget(pose_label_);
}

// ─── UI state helpers ──────────────────────────────────────────────────────────

void ControlWindow::SetIdleUi() {
  if (breath_start_btn_) breath_start_btn_->setEnabled(true);
  if (stop_breath_btn_)  stop_breath_btn_->setEnabled(false);
  for (auto* btn : interrupt_btns_) btn->setEnabled(true);
  if (play_btn_) play_btn_->setEnabled(!head_playback_data_.empty());
  if (stop_btn_) stop_btn_->setEnabled(false);
}

void ControlWindow::SetBusyUi() {
  if (breath_start_btn_) breath_start_btn_->setEnabled(false);
  if (stop_breath_btn_)  stop_breath_btn_->setEnabled(false);
  for (auto* btn : interrupt_btns_) btn->setEnabled(false);
  if (play_btn_) play_btn_->setEnabled(false);
  if (stop_btn_) stop_btn_->setEnabled(true);
}

void ControlWindow::SetBreathingUi() {
  if (breath_start_btn_) breath_start_btn_->setEnabled(false);
  if (stop_breath_btn_)  stop_breath_btn_->setEnabled(true);
  for (auto* btn : interrupt_btns_) btn->setEnabled(true);
  if (play_btn_) play_btn_->setEnabled(false);
  if (stop_btn_) stop_btn_->setEnabled(true);
}

void ControlWindow::UpdateVelLabel() {
  if (!vel_label_ || !gimbal_speed_slider_) return;
  const Params& p = Params::Get();
  int step             = gimbal_speed_slider_->value();
  double counts_per_sec = step * p.loop_hz;
  double deg_per_sec    = counts_per_sec * 360.0 / p.counts_per_rev;
  double rpm            = counts_per_sec / p.counts_per_rev * 60.0;
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.1f \xc2\xb0/s   \xc2\xb7   %.1f RPM",
                deg_per_sec, rpm);
  vel_label_->setText(QString::fromUtf8(buf));
}

// ─── Speed slider slot ────────────────────────────────────────────────────────

void ControlWindow::OnSpeedChanged(int value) {
  if (slider_value_label_)
    slider_value_label_->setText(QString::number(value));
  UpdateVelLabel();
}

// ─── Main tick ────────────────────────────────────────────────────────────────

void ControlWindow::OnTick() {
  int step = gimbal_speed_slider_ ? gimbal_speed_slider_->value()
                                  : Params::Get().step_size;
  if (use_eye_) {
    if (gimbal_resetting_) {
      TickGimbalReset(step);
    } else {
      TickGimbalWasd(step);
      TickEyeProfile();
    }
  }
  if (use_head_) {
    TickHeadPlayback();
  }
}

// ─── Gimbal ticks ─────────────────────────────────────────────────────────────

void ControlWindow::TickGimbalWasd(int step) {
  const Params& p = Params::Get();

  if (key_up_) {
    int new_pos = utils::Clamp(pos1_ + step, p.joint_min_1, p.joint_max_1);
    if (new_pos != pos1_) {
      pos1_ = new_pos;
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_1, pos1_);
    }
  } else if (key_down_) {
    int new_pos = utils::Clamp(pos1_ - step, p.joint_min_1, p.joint_max_1);
    if (new_pos != pos1_) {
      pos1_ = new_pos;
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_1, pos1_);
    }
  }

  if (key_left_) {
    int new_pos = utils::Clamp(pos2_ + step, p.joint_min_2, p.joint_max_2);
    if (new_pos != pos2_) {
      pos2_ = new_pos;
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_2, pos2_);
    }
  } else if (key_right_) {
    int new_pos = utils::Clamp(pos2_ - step, p.joint_min_2, p.joint_max_2);
    if (new_pos != pos2_) {
      pos2_ = new_pos;
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_2, pos2_);
    }
  }
}

void ControlWindow::TickGimbalReset(int step) {
  const Params& p = Params::Get();

  auto step_toward = [&](int pos, int min_j, int max_j, int motor_id) {
    int diff = p.eye_center - pos;
    if (diff == 0) return pos;
    int delta = (diff > 0 ? 1 : -1) * std::min(step, std::abs(diff));
    int new_pos = utils::Clamp(pos + delta, min_j, max_j);
    utils::WritePosition(port_handle_, packet_handle_, motor_id, new_pos);
    return new_pos;
  };

  pos1_ = step_toward(pos1_, p.joint_min_1, p.joint_max_1, p.dxl_1);
  pos2_ = step_toward(pos2_, p.joint_min_2, p.joint_max_2, p.dxl_2);

  if (pos1_ == p.eye_center && pos2_ == p.eye_center)
    gimbal_resetting_ = false;
}

// ─── Eye profile ──────────────────────────────────────────────────────────────

std::string ControlWindow::FindEyeProfile(const std::string& keyword) const {
  std::string folder = AppDir() + "/" + Params::Get().eye_motion_profile_folder;
  auto matches = GlobFiles(folder, keyword, {".csv"});
  if (matches.empty())
    throw std::runtime_error(
        "No eye profile CSV matching '*" + keyword + "*.csv' in " + folder);
  return matches.back();
}

std::vector<std::tuple<double, int, int>>
ControlWindow::LoadEyeProfile(const std::string& keyword) {
  const Params& p = Params::Get();
  std::string path = FindEyeProfile(keyword);
  std::vector<std::tuple<double, int, int>> data;

  std::ifstream f(path);
  if (!f.is_open())
    throw std::runtime_error("Cannot open eye profile: " + path);

  std::string header_line;
  std::getline(f, header_line);  // skip header

  std::string line;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string tok_t, tok_x, tok_y;
    if (!std::getline(ss, tok_t, ',')) continue;
    if (!std::getline(ss, tok_x, ',')) continue;
    if (!std::getline(ss, tok_y, ',')) continue;
    try {
      double t  = std::stod(tok_t);
      int m1 = utils::Clamp(
          static_cast<int>(std::round(p.eye_center - std::stod(tok_x) * p.counts_per_deg)),
          p.joint_min_1, p.joint_max_1);
      int m2 = utils::Clamp(
          static_cast<int>(std::round(p.eye_center - std::stod(tok_y) * p.counts_per_deg)),
          p.joint_min_2, p.joint_max_2);
      data.emplace_back(t, m1, m2);
    } catch (...) {}
  }
  return data;
}

void ControlWindow::StartEyeProfile(const std::string& keyword) {
  if (eye_playing_ || eye_rewinding_ || gimbal_resetting_) return;
  const Params& p = Params::Get();

  std::vector<std::tuple<double, int, int>> data;
  try {
    data = LoadEyeProfile(keyword);
  } catch (const std::exception& exc) {
    std::cerr << "[eye profile] " << exc.what() << "\n";
    return;
  }

  utils::WritePosition(port_handle_, packet_handle_, p.dxl_1, p.eye_center);
  utils::WritePosition(port_handle_, packet_handle_, p.dxl_2, p.eye_center);
  pos1_ = p.eye_center;
  pos2_ = p.eye_center;

  eye_profile_data_ = std::move(data);
  eye_play_idx_     = 0;
  eye_play_start_   = Now();
  eye_playing_      = true;
  eye_rewinding_    = false;

  if (bells_btn_)        bells_btn_->setEnabled(false);
  if (saccadic_btn_)     saccadic_btn_->setEnabled(false);
  if (eye_progress_bar_) eye_progress_bar_->setValue(0);
  if (eye_progress_label_) {
    eye_progress_label_->setText("Playing\xe2\x80\xa6");
    eye_progress_label_->setStyleSheet(
        QString::fromStdString("color: " + Params::Get().ui_fg_color +
                               "; background: transparent;"));
  }
}

void ControlWindow::StartEyeRewind() {
  const Params& p = Params::Get();
  int n_steps = std::max(1, static_cast<int>(std::round(1.5 * p.loop_hz)));

  eye_rewind_steps_.clear();
  eye_rewind_steps_.reserve(n_steps);
  for (int i = 1; i <= n_steps; ++i) {
    int m1 = static_cast<int>(std::round(
        pos1_ + (static_cast<double>(i) / n_steps) * (p.eye_center - pos1_)));
    int m2 = static_cast<int>(std::round(
        pos2_ + (static_cast<double>(i) / n_steps) * (p.eye_center - pos2_)));
    eye_rewind_steps_.emplace_back(m1, m2);
  }
  eye_rewind_idx_ = 0;
  eye_rewinding_  = true;

  if (eye_progress_label_) {
    eye_progress_label_->setText("Rewinding\xe2\x80\xa6");
    eye_progress_label_->setStyleSheet(
        QString::fromStdString("color: " + Params::Get().ui_dim_color +
                               "; background: transparent;"));
  }
}

void ControlWindow::TickEyeProfile() {
  const Params& p = Params::Get();

  if (eye_playing_ && !eye_rewinding_) {
    double elapsed = Now() - eye_play_start_;
    int n_total    = static_cast<int>(eye_profile_data_.size());

    while (eye_play_idx_ < n_total) {
      auto [t, m1, m2] = eye_profile_data_[eye_play_idx_];
      if (t > elapsed) break;
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_1, m1);
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_2, m2);
      pos1_ = m1;
      pos2_ = m2;
      ++eye_play_idx_;
    }

    if (eye_play_idx_ >= n_total) {
      StartEyeRewind();
    } else {
      if (eye_progress_bar_)
        eye_progress_bar_->setValue(
            static_cast<int>(50.0 * eye_play_idx_ / n_total));
      if (eye_progress_label_)
        eye_progress_label_->setText(
            QString("Playing\xe2\x80\xa6  %1/%2")
                .arg(eye_play_idx_)
                .arg(n_total));
    }

  } else if (eye_rewinding_) {
    int n_rewind = static_cast<int>(eye_rewind_steps_.size());
    if (eye_rewind_idx_ < n_rewind) {
      auto [m1, m2] = eye_rewind_steps_[eye_rewind_idx_];
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_1, m1);
      utils::WritePosition(port_handle_, packet_handle_, p.dxl_2, m2);
      pos1_ = m1;
      pos2_ = m2;
      ++eye_rewind_idx_;
      if (eye_progress_bar_)
        eye_progress_bar_->setValue(
            50 + static_cast<int>(50.0 * eye_rewind_idx_ / n_rewind));
    } else {
      eye_playing_   = false;
      eye_rewinding_ = false;
      if (eye_progress_bar_)  eye_progress_bar_->setValue(100);
      if (eye_progress_label_) {
        eye_progress_label_->setText("Done \xe2\x9c\x93");
        eye_progress_label_->setStyleSheet(
            QString::fromStdString("color: " + p.ui_btn_green +
                                   "; background: transparent;"));
      }
      if (bells_btn_)    bells_btn_->setEnabled(true);
      if (saccadic_btn_) saccadic_btn_->setEnabled(true);
    }
  }
}

// ─── Breathing loop ───────────────────────────────────────────────────────────

void ControlWindow::StartBreathing() {
  if (breathing_ || is_playing_) return;
  const Params& p = Params::Get();

  std::string folder = AppDir() + "/" + p.head_motion_profile_folder;
  auto matches = GlobFiles(folder, p.head_rest_profile, {".txt", ".csv"});
  if (matches.empty()) {
    if (progress_label_) {
      progress_label_->setText(
          QString::fromStdString("No '" + p.head_rest_profile + "' profile found"));
      progress_label_->setStyleSheet(
          QString::fromStdString("color: " + p.ui_btn_red + "; background: transparent;"));
    }
    return;
  }
  LoadFile(matches.back());
  if (head_playback_data_.empty() || !robot_) return;

  breathing_      = true;
  head_play_idx_  = 0;
  prev_pitch_     = 0.0;
  prev_roll_      = 0.0;
  playback_start_ = Now();
  is_playing_     = true;
  SetBreathingUi();

  if (progress_bar_)   progress_bar_->setValue(0);
  if (progress_label_) {
    progress_label_->setText("Breathing\xe2\x80\xa6");
    progress_label_->setStyleSheet(
        QString::fromStdString("color: " + p.ui_fg_color + "; background: transparent;"));
  }
}

void ControlWindow::StopBreathing() {
  breathing_     = false;
  resume_breath_ = false;
  if (!head_rewinding_ && is_playing_)
    StartHeadRewind();
}

void ControlWindow::StartInterruption(const std::string& keyword) {
  const Params& p = Params::Get();

  // Save current position for resume.
  breath_resume_idx_   = head_play_idx_;
  breath_resume_pitch_ = prev_pitch_;
  breath_resume_roll_  = prev_roll_;
  if (head_play_idx_ < static_cast<int>(head_playback_data_.size())) {
    breath_resume_t_ = std::get<0>(head_playback_data_[head_play_idx_]);
  } else if (!head_playback_data_.empty()) {
    breath_resume_t_ = std::get<0>(head_playback_data_.back());
  } else {
    breath_resume_t_ = 0.0;
  }

  breathing_     = false;
  resume_breath_ = true;

  if (robot_) {
    try { robot_->ClearMotion(); } catch (...) {}
    try { robot_->ResumeMotion(); } catch (...) {}
  }

  std::string folder = AppDir() + "/" + p.head_motion_profile_folder;
  auto matches = GlobFiles(folder, keyword, {".txt", ".csv"});
  if (matches.empty()) {
    // Fall back to continuing breathing.
    breathing_     = true;
    resume_breath_ = false;
    SetBreathingUi();
    return;
  }
  LoadFile(matches.back());

  head_play_idx_  = 0;
  prev_pitch_     = 0.0;
  prev_roll_      = 0.0;
  playback_start_ = Now();
  is_playing_     = true;
  SetBusyUi();
  if (progress_label_) {
    progress_label_->setText(
        QString::fromStdString("Playing " + keyword + "\xe2\x80\xa6"));
    progress_label_->setStyleSheet(
        QString::fromStdString("color: " + p.ui_fg_color + "; background: transparent;"));
  }
}

void ControlWindow::ResumeBreathingFromSaved() {
  const Params& p = Params::Get();

  std::string folder = AppDir() + "/" + p.head_motion_profile_folder;
  auto matches = GlobFiles(folder, p.head_rest_profile, {".txt", ".csv"});
  if (matches.empty() || !robot_) {
    StopPlayback();
    return;
  }
  LoadFile(matches.back());

  double rp = breath_resume_pitch_;
  double rr = breath_resume_roll_;
  if (std::abs(rp) > 0.01 || std::abs(rr) > 0.01) {
    try {
      robot_->MoveLinRelTrf(0, 0, 0,
                             p.head_pitch_sign * rp,
                             0,
                             p.head_roll_sign  * rr);
    } catch (const std::exception& exc) {
      std::cerr << "[breath resume] " << exc.what() << "\n";
    }
  }

  head_play_idx_  = breath_resume_idx_;
  prev_pitch_     = rp;
  prev_roll_      = rr;
  playback_start_ = Now() - breath_resume_t_;
  is_playing_     = true;
  breathing_      = true;
  SetBreathingUi();
  if (progress_label_) {
    progress_label_->setText("Breathing (resumed)\xe2\x80\xa6");
    progress_label_->setStyleSheet(
        QString::fromStdString("color: " + p.ui_fg_color + "; background: transparent;"));
  }
}

// ─── Head motion preset ───────────────────────────────────────────────────────

void ControlWindow::PlayPreset(const std::string& keyword) {
  if (breathing_) {
    StartInterruption(keyword);
    return;
  }
  const Params& p = Params::Get();
  std::string folder = AppDir() + "/" + p.head_motion_profile_folder;
  auto matches = GlobFiles(folder, keyword, {".txt", ".csv"});
  if (matches.empty()) {
    if (file_label_) {
      file_label_->setText(
          QString::fromStdString("No '" + keyword + "' profile found"));
      file_label_->setStyleSheet(
          QString::fromStdString("color: " + p.ui_btn_red + "; background: transparent;"));
    }
    return;
  }
  LoadFile(matches.back());
  StartPlayback();
}

void ControlWindow::BrowseFile() {
  const Params& p = Params::Get();
  std::string motion_dir = AppDir() + "/" + p.head_motion_profile_folder;
  QString path = QFileDialog::getOpenFileName(
      this,
      "Select head motion profile",
      QString::fromStdString(motion_dir),
      "Text/CSV files (*.txt *.csv);;All files (*.*)");
  if (!path.isEmpty())
    LoadFile(path.toStdString());
}

void ControlWindow::LoadFile(const std::string& path) {
  const Params& p = Params::Get();

  // ── Parse raw CSV ────────────────────────────────────────────────────────────
  // Columns: time(sample_count), gyro0, gyro1, gyro2, acc0, acc1, acc2
  struct RawSample {
    double sample_count, g0, g1, g2, a0, a1, a2;
  };
  std::vector<RawSample> raw;

  std::ifstream f(path);
  if (!f.is_open()) {
    if (file_label_) {
      file_label_->setText(
          QString::fromStdString("Error: cannot open " + path));
      file_label_->setStyleSheet(
          QString::fromStdString("color: " + p.ui_btn_red + "; background: transparent;"));
    }
    return;
  }

  std::string line;
  std::getline(f, line);  // skip header

  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
      try { vals.push_back(std::stod(tok)); } catch (...) {}
    }
    if (vals.size() < 7) continue;
    raw.push_back({vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]});
  }

  if (raw.empty()) {
    if (file_label_) {
      file_label_->setText("No valid data found");
      file_label_->setStyleSheet(
          QString::fromStdString("color: " + p.ui_btn_red + "; background: transparent;"));
    }
    return;
  }

  // ── Complementary filter fusion ──────────────────────────────────────────────
  double alpha = p.head_cf_alpha;
  double beta  = p.head_accel_lpf_beta;

  double t0_s  = raw[0].sample_count * 0.001;
  double ax_f  = raw[0].a0;
  double ay_f  = raw[0].a1;
  double az_f  = raw[0].a2;

  double az_safe  = (std::abs(az_f) > 1e-6) ? az_f : 1e-6;
  double pitch_ref = std::atan2(ax_f,  az_safe);
  double roll_ref  = std::atan2(-ay_f, az_safe);

  double pitch_deg = 0.0;
  double roll_deg  = 0.0;
  double t_prev_s  = t0_s;

  std::vector<std::tuple<double, double, double>> result;
  result.reserve(raw.size());
  result.emplace_back(0.0, 0.0, 0.0);

  for (size_t idx = 1; idx < raw.size(); ++idx) {
    const auto& s = raw[idx];

    ax_f = beta * ax_f + (1.0 - beta) * s.a0;
    ay_f = beta * ay_f + (1.0 - beta) * s.a1;
    az_f = beta * az_f + (1.0 - beta) * s.a2;

    az_safe = (std::abs(az_f) > 1e-6) ? az_f : 1e-6;
    double pitch_accel = std::atan2(ax_f,  az_safe) - pitch_ref;
    double roll_accel  = std::atan2(-ay_f, az_safe) - roll_ref;
    pitch_accel = pitch_accel * 180.0 / M_PI;
    roll_accel  = roll_accel  * 180.0 / M_PI;

    double t_s = s.sample_count * 0.001 - t0_s;
    double dt  = std::max(t_s - t_prev_s, 1e-6);

    double pitch_gyro = pitch_deg + s.g1 * dt;
    double roll_gyro  = roll_deg  + s.g0 * dt;

    pitch_deg = alpha * pitch_gyro + (1.0 - alpha) * pitch_accel;
    roll_deg  = alpha * roll_gyro  + (1.0 - alpha) * roll_accel;

    result.emplace_back(t_s, pitch_deg, roll_deg);
    t_prev_s = t_s;
  }

  head_playback_data_ = std::move(result);
  head_play_idx_      = 0;
  is_playing_         = false;

  double duration_s = std::get<0>(head_playback_data_.back());

  if (file_label_) {
    file_label_->setText(
        QString::fromStdString(
            std::filesystem::path(path).filename().string()));
    file_label_->setStyleSheet(
        QString::fromStdString("color: " + p.ui_fg_color + "; background: transparent;"));
  }
  if (info_label_) {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "%zu samples  \xc2\xb7  %.1f s  \xc2\xb7  CF \xce\xb1=%.2f  LPF \xce\xb2=%.2f",
                  head_playback_data_.size(), duration_s, alpha, beta);
    info_label_->setText(QString::fromUtf8(buf));
  }
  if (play_btn_)      play_btn_->setEnabled(true);
  if (progress_label_) progress_label_->setText("");
  if (progress_bar_)   progress_bar_->setValue(0);
}

void ControlWindow::StartPlayback() {
  if (head_playback_data_.empty() || !robot_) return;
  head_play_idx_  = 0;
  prev_pitch_     = 0.0;
  prev_roll_      = 0.0;
  playback_start_ = Now();
  is_playing_     = true;
  SetBusyUi();
  if (progress_bar_)   progress_bar_->setValue(0);
  if (progress_label_) {
    progress_label_->setText("Playing\xe2\x80\xa6");
    progress_label_->setStyleSheet(
        QString::fromStdString("color: " + Params::Get().ui_fg_color +
                               "; background: transparent;"));
  }
  std::cout << "[playback] samples=" << head_playback_data_.size()
            << "  use_head=" << use_head_ << "\n";
}

void ControlWindow::StopPlayback() {
  is_playing_     = false;
  head_rewinding_ = false;
  breathing_      = false;
  resume_breath_  = false;
  if (progress_bar_) {
    progress_bar_->setMinimum(0);
    progress_bar_->setMaximum(100);
  }
  if (robot_) {
    try { robot_->ClearMotion(); } catch (...) {}
    try { robot_->ResumeMotion(); } catch (...) {}
  }
  SetIdleUi();
  if (progress_label_) {
    progress_label_->setText("Stopped");
    progress_label_->setStyleSheet(
        QString::fromStdString("color: " + Params::Get().ui_dim_color +
                               "; background: transparent;"));
  }
}

void ControlWindow::OnStopPressed() {
  if (head_rewinding_) {
    StopPlayback();
  } else if (is_playing_) {
    breathing_     = false;
    resume_breath_ = false;
    StartHeadRewind();
  }
}

void ControlWindow::StartHeadRewind() {
  const Params& p = Params::Get();

  if (robot_) {
    try { robot_->ClearMotion(); } catch (...) {}
    try { robot_->ResumeMotion(); } catch (...) {}
    try {
      robot_->MoveJoints(head_home_joints_[0], head_home_joints_[1],
                          head_home_joints_[2], head_home_joints_[3],
                          head_home_joints_[4], head_home_joints_[5]);
    } catch (const std::exception& exc) {
      std::cerr << "[head return] " << exc.what() << "\n";
    }
  }

  head_return_done_ = false;
  head_rewinding_   = true;

  if (progress_bar_) {
    progress_bar_->setMinimum(0);
    progress_bar_->setMaximum(0);  // indeterminate / busy animation
  }
  if (progress_label_) {
    progress_label_->setText("Returning home\xe2\x80\xa6");
    progress_label_->setStyleSheet(
        QString::fromStdString("color: " + p.ui_dim_color + "; background: transparent;"));
  }

  // Join any previous rewind thread before starting a new one.
  if (rewind_thread_.joinable()) rewind_thread_.join();
  rewind_thread_ = std::thread(&ControlWindow::WaitHeadReturn, this);
}

void ControlWindow::WaitHeadReturn() {
  try {
    if (robot_) robot_->WaitIdle(30);
  } catch (const std::exception& exc) {
    std::cerr << "[head return wait] " << exc.what() << "\n";
  }
  head_return_done_ = true;
}

void ControlWindow::TickHeadPlayback() {
  if (!is_playing_ || !robot_) return;
  const Params& p = Params::Get();

  // ── Rewind check ─────────────────────────────────────────────────────────────
  if (head_rewinding_) {
    if (head_return_done_.load()) {
      head_rewinding_   = false;
      head_return_done_ = false;
      if (resume_breath_) {
        resume_breath_ = false;
        ResumeBreathingFromSaved();
      } else {
        StopPlayback();
        if (progress_bar_) {
          progress_bar_->setMinimum(0);
          progress_bar_->setMaximum(100);
          progress_bar_->setValue(0);
        }
        if (progress_label_) {
          progress_label_->setText("Done \xe2\x9c\x93");
          progress_label_->setStyleSheet(
              QString::fromStdString("color: " + p.ui_btn_green +
                                     "; background: transparent;"));
        }
      }
    }
    return;
  }

  // ── Forward playback ──────────────────────────────────────────────────────────
  double elapsed_s = Now() - playback_start_;
  int n_total      = static_cast<int>(head_playback_data_.size());
  double acc_pitch = 0.0;
  double acc_roll  = 0.0;

  while (head_play_idx_ < n_total) {
    auto [t_s, pitch, roll] = head_playback_data_[head_play_idx_];
    if (t_s > elapsed_s) break;

    double d_pitch = pitch - prev_pitch_;
    double d_roll  = roll  - prev_roll_;
    // Skip large jumps (noise / discontinuities).
    if (std::abs(d_pitch) < 5.0 && std::abs(d_roll) < 5.0) {
      acc_pitch += d_pitch;
      acc_roll  += d_roll;
    }
    prev_pitch_ = pitch;
    prev_roll_  = roll;
    ++head_play_idx_;
  }

  if (std::abs(acc_pitch) > 0.01 || std::abs(acc_roll) > 0.01) {
    try {
      robot_->MoveLinRelTrf(0, 0, 0,
                             p.head_pitch_sign * acc_pitch,
                             0,
                             p.head_roll_sign  * acc_roll);
    } catch (const std::exception& exc) {
      std::cerr << "[head playback] " << exc.what() << "\n";
    }
  }

  if (head_play_idx_ >= n_total) {
    if (breathing_) {
      // Seamless loop.
      head_play_idx_  = 0;
      prev_pitch_     = 0.0;
      prev_roll_      = 0.0;
      playback_start_ = Now();
    } else {
      StartHeadRewind();
    }
  } else {
    int pct = static_cast<int>(100.0 * head_play_idx_ / n_total);
    if (progress_bar_) progress_bar_->setValue(pct);
    if (progress_label_) {
      char buf[64];
      if (breathing_) {
        std::snprintf(buf, sizeof(buf),
                      "Breathing\xe2\x80\xa6  %.1f s  (%d%%)", elapsed_s, pct);
      } else {
        std::snprintf(buf, sizeof(buf), "%.1f s  (%d%%)", elapsed_s, pct);
      }
      progress_label_->setText(QString::fromUtf8(buf));
    }
  }
}

// ─── TRF Cartesian jog ────────────────────────────────────────────────────────

namespace {
constexpr int kAxisX  = 0;
constexpr int kAxisY  = 1;
constexpr int kAxisZ  = 2;
constexpr int kAxisUx = 3;
constexpr int kAxisUy = 4;
constexpr int kAxisUz = 5;

int AxisIndex(const std::string& axis) {
  if (axis == "x")  return kAxisX;
  if (axis == "y")  return kAxisY;
  if (axis == "z")  return kAxisZ;
  if (axis == "ux") return kAxisUx;
  if (axis == "uy") return kAxisUy;
  if (axis == "uz") return kAxisUz;
  return -1;
}
}  // namespace

void ControlWindow::SendTrfStep(const std::string& axis, double sign) {
  if (!robot_) return;
  double lin = (trf_lin_step_ ? trf_lin_step_->value() : 1.0) * sign;
  double ang = (trf_ang_step_ ? trf_ang_step_->value() : 1.0) * sign;

  double dx = 0, dy = 0, dz = 0, drx = 0, dry = 0, drz = 0;
  bool is_linear = true;
  if      (axis == "x")  { dx  = lin; }
  else if (axis == "y")  { dy  = lin; }
  else if (axis == "z")  { dz  = lin; }
  else if (axis == "ux") { drx = ang; is_linear = false; }
  else if (axis == "uy") { dry = ang; is_linear = false; }
  else if (axis == "uz") { drz = ang; is_linear = false; }
  else return;

  try {
    robot_->MoveLinRelTrf(dx, dy, dz, drx, dry, drz);
    int idx = AxisIndex(axis);
    if (idx >= 0) cart_pos_[idx] += is_linear ? lin : ang;
  } catch (const std::exception& exc) {
    std::cerr << "[trf] " << exc.what() << "\n";
  }
}

void ControlWindow::StartTrfJog(const std::string& axis, double sign) {
  trf_jog_axis_ = axis;
  trf_jog_sign_ = sign;
  SendTrfStep(axis, sign);
  jog_timer_->setInterval(400);
  jog_timer_->start();
}

void ControlWindow::OnRepeatTrfJog() {
  SendTrfStep(trf_jog_axis_, trf_jog_sign_);
  jog_timer_->setInterval(200);
}

void ControlWindow::StopTrfJog() {
  jog_timer_->stop();
}

void ControlWindow::GoInitPose() {
  if (!robot_) return;
  try {
    const auto& j = Params::Get().robot_head_init_pose;
    robot_->MoveJoints(j[0], j[1], j[2], j[3], j[4], j[5]);
    cart_pos_.fill(0.0);
  } catch (const std::exception& exc) {
    std::cerr << "[init] " << exc.what() << "\n";
  }
}

void ControlWindow::SetTrfHome() {
  home_cart_offset_ = cart_pos_;
  if (robot_) {
    try {
      std::array<double, 6> joints = robot_->GetRtTargetJointPos();
      head_home_joints_ = joints;
      std::cout << "[home] Joint home set: ";
      for (int i = 0; i < 6; ++i)
        std::cout << std::round(joints[i] * 100) / 100.0 << " ";
      std::cout << "\n";
    } catch (const std::exception& exc) {
      std::cerr << "[home] Could not read joints: " << exc.what() << "\n";
    }
  }
}

void ControlWindow::GoTrfHome() {
  if (!robot_) return;
  std::array<double, 6> delta{};
  bool any_nonzero = false;
  for (int i = 0; i < 6; ++i) {
    delta[i] = home_cart_offset_[i] - cart_pos_[i];
    if (std::abs(delta[i]) >= 0.001) any_nonzero = true;
  }
  if (!any_nonzero) return;
  try {
    robot_->MoveLinRelTrf(delta[0], delta[1], delta[2],
                           delta[3], delta[4], delta[5]);
    cart_pos_ = home_cart_offset_;
  } catch (const std::exception& exc) {
    std::cerr << "[home] " << exc.what() << "\n";
  }
}

void ControlWindow::ResetRobotError() {
  if (!robot_) return;
  try {
    robot_->ResetError();
    robot_->ResumeMotion();
    std::cout << "[robot] Error reset.\n";
  } catch (const std::exception& exc) {
    std::cerr << "[robot] Reset failed: " << exc.what() << "\n";
  }
}

void ControlWindow::GetRobotPose() {
  if (!robot_ || !pose_label_) return;
  try {
    std::array<double, 6> joints = robot_->GetRtTargetJointPos();
    std::string text;
    for (int i = 0; i < 6; ++i) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "J%d: %.1f\xc2\xb0", i + 1, joints[i]);
      if (i > 0) text += "  ";
      text += buf;
    }
    pose_label_->setText(QString::fromUtf8(text.c_str()));
    pose_label_->setStyleSheet(
        QString::fromStdString("color: " + Params::Get().ui_fg_color +
                               "; background: transparent;"));
    std::cout << "[pose] " << text << "\n";
  } catch (const std::exception& exc) {
    pose_label_->setText(
        QString::fromStdString("Error: " + std::string(exc.what())));
    pose_label_->setStyleSheet(
        QString::fromStdString("color: " + Params::Get().ui_btn_red +
                               "; background: transparent;"));
  }
}

// ─── Jog repeat timer slot ────────────────────────────────────────────────────

// (implemented inline above as OnRepeatTrfJog)
