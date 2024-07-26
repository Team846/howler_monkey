#include "commands/teleop/stow_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

StowCommand::StowCommand(RobotContainer& container)
    : frc846::Loggable{"stow_command"}, super_(container.super_structure_) {
  AddRequirements({&super_});
  SetName("stow_command");
}

void StowCommand::Initialize() { Log("Stow Command Initialize"); }

void StowCommand::Execute() {
  super_.SetTargetSetpoint(super_.getStowSetpoint());
}

void StowCommand::End(bool interrupted) { Log("Stow Command Finished"); }

bool StowCommand::IsFinished() { return false; }