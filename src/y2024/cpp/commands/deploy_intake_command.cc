#include "commands/deploy_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/setpoints.h"

DeployIntakeCommand::DeployIntakeCommand(
    RobotContainer& container)
    : frc846::Loggable{"deploy_intake_command"},
      scorer_(container.scorer_), pivot_(container.pivot_),
        telescope_(container.telescope_), wrist_(container.wrist_) {
  AddRequirements({&scorer_, &pivot_, &telescope_, &wrist_});
  SetName("deploy_intake_command");
}

void DeployIntakeCommand::Initialize() {
  Log("Deploy Intake Command Initialize");
}

void DeployIntakeCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(kIntake));

  double nextPivotTarget = setpoints::kIntake(0).value();
  pivot_.SetTarget(pivot_.MakeTarget(units::degree_t(nextPivotTarget)));
  double nextTelescopeTarget = setpoints::kIntake(1).value();
  telescope_.SetTarget(telescope_.MakeTarget(units::inch_t(nextTelescopeTarget)));
  double nextWristTarget = setpoints::kIntake(2).value();
  wrist_.SetTarget(wrist_.MakeTarget(units::degree_t(nextWristTarget)));

  is_done_ = true;
}

void DeployIntakeCommand::End(bool interrupted) {
  Log("Deploy Intake Command Finished");
}

bool DeployIntakeCommand::IsFinished() {
  return is_done_;
}