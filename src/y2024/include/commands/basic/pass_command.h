#pragma once

#include <frc2/command/CommandHelper.h>

#include "constants.h"
#include "frc846/util/math.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/pivot.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"
#include "subsystems/robot_container.h"

class PassCommand : public frc2::CommandHelper<frc2::Command, PassCommand>,
                    public frc846::Loggable {
 public:
  PassCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;
  ShooterSubsystem& shooter_;
  SuperStructureSubsystem& super_;
  ControlInputSubsystem& control_input_;
};
