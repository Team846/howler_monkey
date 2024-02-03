#ifndef y2024_FUNKY_ROBOT_H_
#define y2024_FUNKY_ROBOT_H_

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <hal/Types.h>
#include <units/time.h>

#include "frcLib846/pref.h"
#include "autos/drive_auto.h"
#include "autos/five_piece_auto.h"
#include "subsystems/robot_container.h"

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class FunkyRobot : public frc::RobotBase, public frcLib846::Loggable {
 public:
  static constexpr auto kPeriod = 20_ms;  // 50hz

  FunkyRobot();

  ~FunkyRobot() override;

  void StartCompetition() override;
  void EndCompetition() override;

  void InitTeleopDefaults();
  void InitTeleopTriggers();

  void InitTestDefaults();

  void VerifyHardware();

 private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  units::microsecond_t next_loop_time_;

  Mode last_mode_;

 private:
  RobotContainer container_;

  frcLib846::Grapher<int> time_remaining_graph_{*this, "time"};

  frcLib846::Grapher<int> warnings_graph_{*this, "warnings"};
  frcLib846::Grapher<int> errors_graph_{*this, "errors"};

  frcLib846::Grapher<double> can_usage_graph_{*this, "CAN_usage"};
  frcLib846::Grapher<units::millisecond_t> loop_time_graph_{*this, "loop_time"};

  frc2::Command* auto_command_ = nullptr;
  frc::SendableChooser<frc2::Command*> auto_chooser_;

  // TODO: Find fix for this
  // Seperate blue vs red because command is generated prior to the alliance
  // color being changed

  // Autos
  frc2::CommandPtr drive_auto_ =
      DriveAuto{container_, true}.ToPtr();
      
  // frc2::CommandPtr five_piece_auto_ = 
  //     FivePieceAuto(container_, true);
};

#endif  // y2024_FUNKY_ROBOT_H_