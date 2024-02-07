#ifndef y2024_COMMANDS_DRIVE_COMMAND_H_
#define y2024_COMMANDS_DRIVE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "subsystems/driver.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"

class DriveCommand
    : public frc2::CommandHelper<frc2::Command, DriveCommand> {
 public:
  DriveCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriverSubsystem& driver_;
  DrivetrainSubsystem& drivetrain_;
};

#endif  // y2024_COMMANDS_DRIVE_COMMAND_H_