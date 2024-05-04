#include "commands/stow_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

StowCommand::StowCommand(RobotContainer& container)
    : frc846::Loggable{"stow_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      pivot_(container.pivot_),
      telescope_(container.telescope_),
      wrist_(container.wrist_),
      super_(container.super_structure_) {
  AddRequirements({&intake_, &shooter_, &pivot_, &telescope_, &wrist_});
  SetName("stow_command");
}

void StowCommand::Initialize() { Log("Stow Command Initialize"); }

void StowCommand::Execute() {
  intake_.SetTarget(intake_.ZeroTarget());
  shooter_.SetTarget(shooter_.ZeroTarget());

  pivot_.SetTarget(pivot_.MakeTarget(super_.getStowSetpoint().pivot));
  telescope_.SetTarget(
      telescope_.MakeTarget(super_.getStowSetpoint().telescope));
  wrist_.SetTarget(wrist_.MakeTarget(super_.getStowSetpoint().wrist));

  is_done_ = true;
}

void StowCommand::End(bool interrupted) { Log("Stow Command Finished"); }

bool StowCommand::IsFinished() { return is_done_; }