#pragma once

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/robot_container.h"

class CloseDriveAmpCommand
    : public frc2::CommandHelper<frc2::Command, CloseDriveAmpCommand>,
      public frc846::Loggable {
 public:
  CloseDriveAmpCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;
};
