#include "commands/teleop/stow_command.h"

StowCommand::StowCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, StowCommand>{
          container, "stow_command"} {
  AddRequirements({&container_.super_structure_});
}

void StowCommand::OnInit() {}

void StowCommand::Periodic() {
  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getStowSetpoint());
}

void StowCommand::OnEnd(bool interrupted) {}

bool StowCommand::IsFinished() { return false; }