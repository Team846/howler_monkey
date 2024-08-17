#include "commands/basic/pass_command.h"

PassCommand::PassCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, PassCommand>{
          container, "pass_command"} {
  AddRequirements({&container_.super_structure_, &container_.intake_,
                   &container_.shooter_});
}

void PassCommand::OnInit() {}

void PassCommand::Periodic() {
  container_.intake_.SetTarget({IntakeState::kHold});
  container_.shooter_.SetTarget({ShooterState::kRun});

  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getPassSetpoint());
}

void PassCommand::OnEnd(bool interrupted) {}

bool PassCommand::IsFinished() { return false; }