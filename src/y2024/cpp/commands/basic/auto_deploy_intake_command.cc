#include "commands/basic/auto_deploy_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

AutoDeployIntakeCommand::AutoDeployIntakeCommand(RobotContainer& container)
    : frc846::Loggable{"deploy_intake_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      super_(container.super_structure_) {
  AddRequirements({&intake_, &shooter_, &container.pivot_, &container.wrist_,
                   &container.telescope_});
  SetName("auto_deploy_intake_command");
}

void AutoDeployIntakeCommand::Initialize() {
  Log("Auto Deploy Intake Command Initialize");

  is_done_ = false;
}

void AutoDeployIntakeCommand::Execute() {
  intake_.SetTarget({kIntake});
  shooter_.SetTarget({kIdle});

  super_.SetTargetSetpoint(super_.getIntakeSetpoint());

  is_done_ = true;
}

void AutoDeployIntakeCommand::End(bool interrupted) {
  Log("Auto Deploy Intake Command Finished");
}

bool AutoDeployIntakeCommand::IsFinished() { return is_done_; }