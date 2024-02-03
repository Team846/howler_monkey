#include "commands/stow_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frcLib846/math.h"
#include "frcLib846/wpilib/time.h"

StowCommand::StowCommand(
    RobotContainer& container)
    : frcLib846::Loggable{"stow_command"},
      scorer_(container.scorer_), scoring_positioner_(container.scoring_positioner_) {
  AddRequirements({&scorer_, &scoring_positioner_});
  SetName("stow_command");
}

void StowCommand::Initialize() {
  Log("Stow Command Initialize");
}

void StowCommand::Execute() {
  scorer_.SetTarget(scorer_.ZeroTarget());
  scoring_positioner_.SetTarget(scoring_positioner_.ZeroTarget());

  is_done_ = true;
}

void StowCommand::End(bool interrupted) {
  Log("Stow Command Finished");
}

bool StowCommand::IsFinished() {
  return is_done_;
}