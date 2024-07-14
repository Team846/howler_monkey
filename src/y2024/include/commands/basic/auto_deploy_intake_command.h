#pragma once

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/pivot.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"
#include "subsystems/robot_container.h"

class AutoDeployIntakeCommand
    : public frc2::CommandHelper<frc2::Command, AutoDeployIntakeCommand>,
      public frc846::Loggable {
 public:
  AutoDeployIntakeCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;
  ShooterSubsystem& shooter_;
  SuperStructureSubsystem& super_;

  bool is_done_ = false;
};
