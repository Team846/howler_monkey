#include "commands/stow_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

StowCommand::StowCommand(
    RobotContainer& container)
    : frc846::Loggable{"stow_command"},
      scorer_(container.scorer_), pivot_(container.pivot_),
        telescope_(container.telescope_), wrist_(container.wrist_) {
  AddRequirements({&scorer_, &pivot_, &telescope_, &wrist_});
  SetName("stow_command");
}

void StowCommand::Initialize() {
  Log("Stow Command Initialize");
}

void StowCommand::Execute() {
  scorer_.SetTarget(scorer_.ZeroTarget());
  pivot_.SetTarget(pivot_.MakeTarget(pivot_.stow_setpoint_pivot.value()));
  telescope_.SetTarget(telescope_.MakeTarget(telescope_.stow_setpoint_tele_.value()));
  wrist_.SetTarget(wrist_.MakeTarget(wrist_.stow_setpoint_wrist_.value()));

  is_done_ = true;
}

void StowCommand::End(bool interrupted) {
  Log("Stow Command Finished");
}

bool StowCommand::IsFinished() {
  return is_done_;
}