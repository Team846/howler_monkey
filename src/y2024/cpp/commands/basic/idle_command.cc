#include "commands/basic/idle_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

IdleCommand::IdleCommand(RobotContainer& container, bool onlyIntake)
    : frc846::Loggable{"idle_command"},
      intake_(container.intake_),
      
      onlyIntake_(onlyIntake) {
  AddRequirements({&intake_});
  SetName("idle_command");
}

void IdleCommand::Initialize() {
  Log("Idle Command Initialize");
  is_done_ = false;
}

void IdleCommand::Execute() {
  intake_.SetTarget({IntakeState::kHold});
  
  is_done_ = true;
}

void IdleCommand::End(bool interrupted) { Log("Idle Command Finished"); }

bool IdleCommand::IsFinished() { return is_done_; }