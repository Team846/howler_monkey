#include "commands/wait_till_spin_up_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

WaitTillSpinUpCommand::WaitTillSpinUpCommand(
    RobotContainer& container)
    : frc846::Loggable{"wait_till_spin_up_command"},
      shooter_(container.shooter_), 
      super_ (container.super_structure_) {
  AddRequirements({&shooter_});
  SetName("wait_till_spin_up_command");
}

void WaitTillSpinUpCommand::Initialize() {
  Log("WaitTillSpinUp Command Initialize");
}

void WaitTillSpinUpCommand::Execute() {
}

void WaitTillSpinUpCommand::End(bool interrupted) {
  Log("Wait Till Spin Up Finished");
}

bool WaitTillSpinUpCommand::IsFinished() {
return std::abs(shooter_.readings().kLeftErrorPercent) < super_.shooter_speed_tolerance_.value();
}