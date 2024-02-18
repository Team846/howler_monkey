#include "commands/shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

ShootCommand::ShootCommand(
    RobotContainer& container)
    : frc846::Loggable{"shoot_command"},
      scorer_(container.scorer_) {
  AddRequirements({&scorer_});
  SetName("shoot_command");
}

void ShootCommand::Initialize() {
  Log("Shoot Command Initialize");
}

void ShootCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(false, true, scorer_.shooter_speed_.value()));

  is_done_ = true;
}

void ShootCommand::End(bool interrupted) {
  Log("Shoot Command Finished");
}

bool ShootCommand::IsFinished() {
  return is_done_;
}