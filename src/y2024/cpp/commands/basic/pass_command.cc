#include "commands/basic/pass_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/abstract/super_structure.h"

PassCommand::PassCommand(RobotContainer& container)
    : frc846::base::Loggable{"pass_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      super_(container.super_structure_),
      control_input_(container.control_input_) {
  AddRequirements({&super_, &intake_, &shooter_});
  SetName("pass_command");
}

void PassCommand::Initialize() { Log("Pass Command Initialize"); }

void PassCommand::Execute() {
  intake_.SetTarget({IntakeState::kHold});
  shooter_.SetTarget({ShooterState::kRun});

  super_.SetTargetSetpoint(super_.getPassSetpoint());
}

void PassCommand::End(bool interrupted) { Log("Pass Command Finished"); }

bool PassCommand::IsFinished() { return false; }