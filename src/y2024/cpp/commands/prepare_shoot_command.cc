#include "commands/prepare_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

PrepareShootCommand::PrepareShootCommand(
    RobotContainer& container)
    : frc846::Loggable{"prepare_shoot_command"},
      scorer_(container.scorer_), pivot_(container.pivot_), 
        telescope_(container.telescope_), wrist_(container.wrist_) {
  AddRequirements({&pivot_, &telescope_, &wrist_}); //don't add scorer, only set target for scorer once during initialize
  SetName("prepare_shoot_command");
}

void PrepareShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShootCommand::Execute() {
  scorer_.SetTarget(scorer_.ZeroTarget());
  pivot_.SetTarget(pivot_.ZeroTarget());
  telescope_.SetTarget(telescope_.ZeroTarget());
  wrist_.SetTarget(wrist_.ZeroTarget());

  is_done_ = true;
}

void PrepareShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShootCommand::IsFinished() {
  return is_done_;
}