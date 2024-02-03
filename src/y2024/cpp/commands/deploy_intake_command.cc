#include "commands/deploy_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/math.h"
#include "frc846/wpilib/time.h"

DeployIntakeCommand::DeployIntakeCommand(
    RobotContainer& container)
    : frc846::Loggable{"deploy_intake_command"},
      shintake_(container.shintake_), arm_(container.arm_) {
  AddRequirements({&shintake_, &arm_});
  SetName("deploy_intake_command");
}

void DeployIntakeCommand::Initialize() {
  Log("Deploy Intake Command Initialize");
}

void DeployIntakeCommand::Execute() {
  shintake_.SetTarget(shintake_.ZeroTarget());
  arm_.SetTarget(arm_.ZeroTarget());

  is_done_ = true;
}

void DeployIntakeCommand::End(bool interrupted) {
  Log("Deploy Intake Command Finished");
}

bool DeployIntakeCommand::IsFinished() {
  return is_done_;
}