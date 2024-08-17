#include "commands/basic/source_command.h"

SourceCommand::SourceCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, SourceCommand>{
          container, "source_command"} {
  AddRequirements({&container_.super_structure_, &container_.intake_});
}

void SourceCommand::OnInit() {}

void SourceCommand::Periodic() {
  container_.intake_.SetTarget({IntakeState::kIntake});
  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getSourceSetpoint());
}

void SourceCommand::OnEnd(bool interrupted) {}

bool SourceCommand::IsFinished() { return false; }