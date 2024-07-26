#include "commands/basic/wrist_zero_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

WristZeroCommand::WristZeroCommand(RobotContainer& container)
    : frc846::Loggable{"wrist_zero_command"},
      super_(container.super_structure_) {
  AddRequirements(
      {&super_});
  SetName("wrist_zero_command");
}

void WristZeroCommand::Initialize() {
  Log("Wrist Zero Command Initialize");
  super_.wrist_->DeZero();
}

void WristZeroCommand::Execute() {
  super_.HomeWrist();
  super_.SetTargetSetpoint(super_.getStowSetpoint());
}

void WristZeroCommand::End(bool interrupted) {
  Log("Wrist Zero Command Finished");
}

bool WristZeroCommand::IsFinished() { return super_.wrist_->GetHasZeroed(); }