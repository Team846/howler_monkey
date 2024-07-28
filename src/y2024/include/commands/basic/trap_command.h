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

class TrapCommand : public frc2::CommandHelper<frc2::Command, TrapCommand>,
                    public frc846::base::Loggable {
 public:
  TrapCommand(RobotContainer& container, int stage);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;
  ShooterSubsystem& shooter_;
  SuperStructureSubsystem& super_;

  int stage_;
};
