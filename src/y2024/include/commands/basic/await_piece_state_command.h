#pragma once

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/robot_container.h"

class AwaitPieceStateCommand
    : public frc2::CommandHelper<frc2::Command, AwaitPieceStateCommand>,
      public frc846::base::Loggable {
 public:
  AwaitPieceStateCommand(RobotContainer& container, bool has_piece);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;
  bool has_piece_;
};
