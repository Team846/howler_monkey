#pragma once

#include <frc2/command/CommandHelper.h>

#include "constants.h"
#include "subsystems/abstract/driver.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/robot_container.h"

class DriveCommand : public frc2::CommandHelper<frc2::Command, DriveCommand> {
 public:
  DriveCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  DrivetrainSubsystem& drivetrain_;
  SuperStructureSubsystem& super_;
  VisionSubsystem& vision_;
  ShooterSubsystem& shooter_;

  ShootingCalculator shooting_calculator{};

  double driver_adjust_ = 0.0;
};
