#include "commands/basic/source_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

SourceCommand::SourceCommand(RobotContainer& container)
    : frc846::Loggable{"source_command"},
      super_(container.super_structure_),
      intake_(container.intake_) {
  AddRequirements({&container.pivot_, &container.telescope_, &container.wrist_,
                   &container.intake_});
  SetName("source_command");
}

void SourceCommand::Initialize() { Log("Source Command Initialize"); }

void SourceCommand::Execute() {
  intake_.SetTarget({IntakeState::kIntake});
  super_.SetTargetSetpoint(super_.getSourceSetpoint());
}

void SourceCommand::End(bool interrupted) {
  Log("Source Command Finished");
  intake_.SetTarget(intake_.ZeroTarget());
}

bool SourceCommand::IsFinished() {
  return super_.hasReachedSetpoint(super_.getSourceSetpoint()) &&
         intake_.GetHasPiece();
}