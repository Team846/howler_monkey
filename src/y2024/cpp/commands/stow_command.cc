#include "commands/stow_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

#include "subsystems/setpoints.h"

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

  double nextPivotTarget = setpoints::kStow(0);
  pivot_.SetTarget(pivot_.MakeTarget(units::degree_t(nextPivotTarget)));
  double nextTelescopeTarget = setpoints::kStow(1);
  telescope_.SetTarget(telescope_.MakeTarget(units::inch_t(nextTelescopeTarget)));
  double nextWristTarget = setpoints::kStow(2);
  wrist_.SetTarget(wrist_.MakeTarget(units::degree_t(nextWristTarget)));

  is_done_ = true;
}

void StowCommand::End(bool interrupted) {
  Log("Stow Command Finished");
}

bool StowCommand::IsFinished() {
  return is_done_;
}