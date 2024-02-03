#include "commands/stow_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/math.h"
#include "frc846/wpilib/time.h"

StowCommand::StowCommand(
    RobotContainer& container)
    : frc846::Loggable{"stow_command"},
      shintake_(container.shintake_), arm_(container.arm_) {
  AddRequirements({&shintake_, &arm_});
  SetName("stow_command");
}

void StowCommand::Initialize() {
  Log("Stow Command Initialize");
}

void StowCommand::Execute() {
  shintake_.SetTarget(shintake_.ZeroTarget());
  arm_.SetTarget(arm_.ZeroTarget());

  is_done_ = true;
}

void StowCommand::End(bool interrupted) {
  Log("Stow Command Finished");
}

bool StowCommand::IsFinished() {
  return is_done_;
}