#include "commands/deploy_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

DeployIntakeCommand::DeployIntakeCommand(RobotContainer& container)
    : frc846::Loggable{"deploy_intake_command"},
      scorer_(container.scorer_),
      pivot_(container.pivot_),
      telescope_(container.telescope_),
      wrist_(container.wrist_),
      super_(container.super_structure_) {
  AddRequirements({&scorer_, &pivot_, &telescope_, &wrist_});
  SetName("deploy_intake_command");
}

void DeployIntakeCommand::Initialize() {
  Log("Deploy Intake Command Initialize");
}

void DeployIntakeCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(kIntake));

  pivot_.SetTarget(pivot_.MakeTarget(super_.getIntakeSetpoint().pivot));
  telescope_.SetTarget(
      telescope_.MakeTarget(super_.getIntakeSetpoint().telescope));
  wrist_.SetTarget(wrist_.MakeTarget(super_.getIntakeSetpoint().wrist));

  is_done_ = true;
}

void DeployIntakeCommand::End(bool interrupted) {
  Log("Deploy Intake Command Finished");
}

bool DeployIntakeCommand::IsFinished() { return is_done_; }