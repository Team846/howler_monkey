#include "commands/deploy_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frcLib846/math.h"
#include "frcLib846/wpilib/time.h"

DeployIntakeCommand::DeployIntakeCommand(
    RobotContainer& container)
    : frcLib846::Loggable{"deploy_intake_command"},
      scorer_(container.scorer_), scoring_positioner_(container.scoring_positioner_) {
  AddRequirements({&scorer_, &scoring_positioner_});
  SetName("deploy_intake_command");
}

void DeployIntakeCommand::Initialize() {
  Log("Deploy Intake Command Initialize");
}

void DeployIntakeCommand::Execute() {
  scorer_.SetTarget(scorer_.ZeroTarget());
  scoring_positioner_.SetTarget(scoring_positioner_.ZeroTarget());

  is_done_ = true;
}

void DeployIntakeCommand::End(bool interrupted) {
  Log("Deploy Intake Command Finished");
}

bool DeployIntakeCommand::IsFinished() {
  return is_done_;
}