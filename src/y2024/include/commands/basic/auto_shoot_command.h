#pragma once

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/robot_container.h"

class AutoShootCommand
    : public frc2::CommandHelper<frc2::Command, AutoShootCommand>,
      public frc846::base::Loggable {
 public:
  AutoShootCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;
  ShooterSubsystem& shooter_;
  ControlInputSubsystem& control_input_;

  bool is_done_ = false;
};
