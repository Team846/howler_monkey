#ifndef y2024_COMMANDS_TELEOP_POSITIONING_COMMAND_H_
#define y2024_COMMANDS_TELEOP_POSITIONING_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "subsystems/driver.h"
#include "subsystems/wrist.h"
#include "subsystems/drivetrain.h"
#include "subsystems/telescope.h"
#include "subsystems/pivot.h"
#include "subsystems/robot_container.h"

class TeleopPositioningCommand
    : public frc2::CommandHelper<frc2::Command, TeleopPositioningCommand> {
 public:
  TeleopPositioningCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriverSubsystem& driver_;
  DrivetrainSubsystem& drivetrain_;
  PivotSubsystem& pivot_;
  TelescopeSubsystem& telescope_;
  WristSubsystem& wrist_;

  bool pivotHasRun = false;
  bool telescopeHasRun = false;
  bool wristHasRun = false;

  bool firstIntakeRound = true;
};

#endif  // y2024_COMMANDS_TELEOP_POSITIONING_COMMAND_H_