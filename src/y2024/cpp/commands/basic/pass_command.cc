#include "commands/basic/pass_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/abstract/super_structure.h"

PassCommand::PassCommand(RobotContainer& container)
    : frc846::Loggable{"pass_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      super_(container.super_structure_),
      control_input_(container.control_input_) {
  AddRequirements({&container.pivot_, &container.wrist_, &container.telescope_,
                   &intake_, &shooter_});
  SetName("pass_command");
}

void PassCommand::Initialize() { Log("Pass Command Initialize"); }

void PassCommand::Execute() {
  intake_.SetTarget({IntakeState::kHold});
  shooter_.SetTarget({ShooterState::kRun});

  // if (!super_.pivot_->WithinTolerance(shootSetpoint.pivot)) {
  //   // shootSetpoint.wrist = super_.getStowSetpoint().wrist;
  //   shootSetpoint.wrist -= 30_deg;
  // }

  super_.SetTargetSetpoint(super_.getPassSetpoint());
}

void PassCommand::End(bool interrupted) {
  intake_.SetTarget({IntakeState::kHold});
  shooter_.SetTarget({ShooterState::kIdle});

  Log("Pass Command Finished");
}

bool PassCommand::IsFinished() {
  return !control_input_.readings().running_pass;

  // return super_.hasReachedSetpoint(super_.getShootSetpoint()) &&
  //        shooter_.readings().error_percent <=
  //            shooter_.shooter_speed_tolerance_.value();
}