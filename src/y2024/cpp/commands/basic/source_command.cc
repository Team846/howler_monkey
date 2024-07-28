#include "commands/basic/source_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

SourceCommand::SourceCommand(RobotContainer& container)
    : frc846::base::Loggable{"source_command"},
      super_(container.super_structure_),
      intake_(container.intake_) {
  AddRequirements({&super_, &intake_});
  SetName("source_command");
}

void SourceCommand::Initialize() { Log("Source Command Initialize"); }

void SourceCommand::Execute() {
  intake_.SetTarget({IntakeState::kIntake});
  super_.SetTargetSetpoint(super_.getSourceSetpoint());
}

void SourceCommand::End(bool interrupted) { Log("Source Command Finished"); }

bool SourceCommand::IsFinished() { return false; }