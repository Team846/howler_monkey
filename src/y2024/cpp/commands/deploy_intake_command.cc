#include "commands/deploy_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

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
  scorer_.SetTarget(scorer_.MakeTarget(true, false, 0.0_tps));
  pivot_.SetTarget(pivot_.MakeTarget(pivot_.intake_setpoint_pivot.value()));
  telescope_.SetTarget(telescope_.MakeTarget(telescope_.intake_setpoint_tele_.value()));
  wrist_.SetTarget(wrist_.MakeTarget(wrist_.intake_setpoint_wrist_.value()));

  is_done_ = true;
}

void DeployIntakeCommand::End(bool interrupted) {
  Log("Deploy Intake Command Finished");
}

bool DeployIntakeCommand::IsFinished() {
  return is_done_;
}