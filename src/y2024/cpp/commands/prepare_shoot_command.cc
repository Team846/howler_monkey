#include "commands/prepare_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frcLib846/math.h"
#include "frcLib846/wpilib/time.h"

PrepareShootCommand::PrepareShootCommand(
    RobotContainer& container)
    : frcLib846::Loggable{"prepare_shoot_command"},
      scorer_(container.scorer_), scoring_positioner_(container.scoring_positioner_) {
  AddRequirements({&scoring_positioner_}); //don't add scorer, only set target for scorer once during initialize
  SetName("prepare_shoot_command");
}

void PrepareShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShootCommand::Execute() {
  scorer_.SetTarget(scorer_.ZeroTarget());
  scoring_positioner_.SetTarget(scoring_positioner_.ZeroTarget());

  is_done_ = true;
}

void PrepareShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShootCommand::IsFinished() {
  return is_done_;
}