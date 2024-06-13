#include "commands/basic/stow_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

StowCommand::StowCommand(RobotContainer& container)
    : frc846::Loggable{"stow_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      super_(container.super_structure_) {
  AddRequirements({&intake_, &shooter_, &container.pivot_,
                   &container.telescope_, &container.wrist_});
  SetName("stow_command");
}

void StowCommand::Initialize() { Log("Stow Command Initialize"); }

void StowCommand::Execute() {
  // intake_.SetTarget(intake_.ZeroTarget());
  // shooter_.SetTarget(shooter_.ZeroTarget());

  super_.SetTargetSetpoint(super_.getStowSetpoint());
}

void StowCommand::End(bool interrupted) { Log("Stow Command Finished"); }

bool StowCommand::IsFinished() {
  return super_.hasReachedSetpoint(super_.getStowSetpoint());
}