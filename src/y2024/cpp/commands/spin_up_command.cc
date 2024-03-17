#include "commands/spin_up_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

SpinUpCommand::SpinUpCommand(
    RobotContainer& container)
    : frc846::Loggable{"spin_up_command"},
      scorer_(container.scorer_) {
  AddRequirements({&scorer_});
  SetName("shoot_command");
}

void SpinUpCommand::Initialize() {
  Log("Spin Up Command Initialize");
}

void SpinUpCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(kSpinUp));

  is_done_ = true;
}

void SpinUpCommand::End(bool interrupted) {
  Log("Shoot Command Finished");
}

bool SpinUpCommand::IsFinished() {
  return is_done_;
}