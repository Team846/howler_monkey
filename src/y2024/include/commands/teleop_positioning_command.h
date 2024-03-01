#ifndef y2024_COMMANDS_TELEOP_POSITIONING_COMMAND_H_
#define y2024_COMMANDS_TELEOP_POSITIONING_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "subsystems/driver.h"
#include "subsystems/operator.h"
#include "subsystems/wrist.h"
#include "subsystems/drivetrain.h"
#include "subsystems/telescope.h"
#include "subsystems/pivot.h"
#include "subsystems/robot_container.h"
#include "frc846/control/motion.h"

class TeleopPositioningCommand
    : public frc2::CommandHelper<frc2::Command, TeleopPositioningCommand> {
 public:
  TeleopPositioningCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriverSubsystem& driver_;
  OperatorSubsystem& operator_;
  DrivetrainSubsystem& drivetrain_;
  PivotSubsystem& pivot_;
  TelescopeSubsystem& telescope_;
  WristSubsystem& wrist_;

  std::vector<frc846::motion::MotionSnapshot> snapshot_;

  bool pivotHasRun = false;
  bool telescopeHasRun = false;
  bool wristHasRun = false;
  bool lastPivotManual = false;
  bool lastTeleManual = false;
  bool lastWristManual = false;

  double mpiv_adj = 0.0;
  double mtele_adj = 0.0;
  double mwr_adj = 0.0;

  double mx_adj = 0.0;
  double mu_adj = 0.0;
  double ms_adj = 0.0;
  double pmx_adj = 0.0;
  double pmu_adj = 0.0;
  double pms_adj = 0.0;
};

#endif  // y2024_COMMANDS_TELEOP_POSITIONING_COMMAND_H_