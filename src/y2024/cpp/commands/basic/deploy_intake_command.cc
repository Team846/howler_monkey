#include "commands/basic/deploy_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

DeployIntakeCommand::DeployIntakeCommand(RobotContainer& container)
    : frc846::Loggable{"deploy_intake_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      super_(container.super_structure_) {
  AddRequirements(
      {&intake_, &container.pivot_, &container.wrist_, &container.telescope_});
  SetName("deploy_intake_command");
}

void DeployIntakeCommand::Initialize() {
  Log("Deploy Intake Command Initialize");
}

void DeployIntakeCommand::Execute() {
  intake_.SetTarget({kIntake});
  shooter_.SetTarget({kIdle});

  super_.SetTargetSetpoint(super_.getIntakeSetpoint());
}

void DeployIntakeCommand::End(bool interrupted) {
  Log("Deploy Intake Command Finished");
  intake_.SetTarget(intake_.ZeroTarget());
}

bool DeployIntakeCommand::IsFinished() {
  return super_.hasReachedSetpoint(super_.getIntakeSetpoint()) &&
         intake_.GetHasPiece();
}