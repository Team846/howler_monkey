#include "commands/prepare_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/math.h"
#include "frc846/wpilib/time.h"

PrepareShootCommand::PrepareShootCommand(
    RobotContainer& container)
    : frc846::Loggable{"prepare_shoot_command"},
      shintake_(container.shintake_), arm_(container.arm_) {
  AddRequirements({&arm_}); //don't add shintake, only set target for shintake once during initialize
  SetName("prepare_shoot_command");
}

void PrepareShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShootCommand::Execute() {
  shintake_.SetTarget(shintake_.ZeroTarget());
  arm_.SetTarget(arm_.ZeroTarget());

  is_done_ = true;
}

void PrepareShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShootCommand::IsFinished() {
  return is_done_;
}