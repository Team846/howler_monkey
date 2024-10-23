#include "commands/basic/pass_command.h"

PassCommand::PassCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, PassCommand>{
          container, "pass_command"} {
  AddRequirements({&container_.super_structure_});
}

void PassCommand::OnInit() {}

void PassCommand::Periodic() {
  container_.shooter_.SetTarget({ShooterState::kRun});

  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getPassSetpoint());
}

void PassCommand::OnEnd(bool interrupted) {}

bool PassCommand::IsFinished() { return false; }