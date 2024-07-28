#pragma once

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <hal/Types.h>
#include <units/time.h>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/core.hpp>
#include <cameraserver/CameraServer.h>

#include "autos/drive_auto.h"
#include "autos/five_piece_auto.h"
#include "autos/one_piece_auto.h"
#include "frc/DigitalInput.h"
#include "frc846/fstore.h"
#include "frc846/util/pref.h"
#include "subsystems/robot_container.h"

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class FunkyRobot : public frc::RobotBase, public frc846::base::Loggable {
 public:
  static constexpr auto kPeriod = 20_ms;  // 50hz

  FunkyRobot();

  ~FunkyRobot() override;

  void StartCompetition() override;
  void EndCompetition() override;

  // static void VisionThread(){
  //   cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  //   camera.SetResolution(240, 240);

  //   cs::CvSink cvSink = frc::CameraServer::GetVideo();
  //   cs::CvSource outputStream =
  //       frc::CameraServer::PutVideo("Rectangle", 240, 240);

  //   cv::Mat mat;

  //   while (true) {
  //     if (cvSink.GrabFrame(mat) == 0) {
  //       outputStream.NotifyError(cvSink.GetError());
  //       continue;
  //     }
  //     outputStream.PutFrame(mat);
  //   }
  // }

  void InitTeleopDefaults();
  void InitTeleopTriggers();

  void InitTestDefaults();
  void InitTestTriggers();

  void VerifyHardware();

 private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  units::microsecond_t next_loop_time_;

  Mode last_mode_;

 private:
  RobotContainer container_;

  frc846::Grapher<int> time_remaining_graph_{*this, "time"};

  frc846::Grapher<int> warnings_graph_{*this, "warnings"};
  frc846::Grapher<int> errors_graph_{*this, "errors"};

  frc846::Grapher<double> can_usage_graph_{*this, "CAN_usage"};
  frc846::Grapher<units::millisecond_t> loop_time_graph_{*this, "loop_time"};

  frc2::Command* auto_command_ = nullptr;
  frc::SendableChooser<frc2::Command*> auto_chooser_;

  // TODO: Find fix for this
  // Separate blue vs red because command is generated prior to the alliance
  // color being changed

  // Autos
  frc2::CommandPtr drive_auto_ = DriveAuto{container_, false}.ToPtr();

  frc2::CommandPtr five_piece_auto_red =
      FivePieceAuto{container_, false}.ToPtr();

  frc2::CommandPtr five_piece_auto_blue =
      FivePieceAuto{container_, true}.ToPtr();

  frc2::CommandPtr one_piece_auto_0 =
      OnePieceAuto{container_, -60_deg, "left red"}.ToPtr();
  frc2::CommandPtr one_piece_auto_1 =
      OnePieceAuto{container_, 60_deg, "right red"}.ToPtr();
  frc2::CommandPtr one_piece_auto_2 =
      OnePieceAuto{container_, -60_deg + 180_deg, "left blue"}.ToPtr();
  frc2::CommandPtr one_piece_auto_3 =
      OnePieceAuto{container_, 60_deg + 180_deg, "right blue"}.ToPtr();

  frc::DigitalInput homing_switch_{0};
  frc::DigitalInput coasting_switch_{1};

  frc846::FunkyStore robotStore{};

  int coast_counter_ = 0;
};
