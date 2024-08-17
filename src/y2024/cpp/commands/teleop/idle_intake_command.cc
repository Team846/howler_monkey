#include "commands/teleop/idle_intake_command.h"

IdleIntakeCommand::IdleIntakeCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, IdleIntakeCommand>{
          container, "idle_intake_command"} {
  AddRequirements({&container_.intake_});
}

void IdleIntakeCommand::OnInit() {}

void IdleIntakeCommand::Periodic() {
  container_.intake_.SetTarget({IntakeState::kHold});
}

void IdleIntakeCommand::OnEnd(bool interrupted) {}

bool IdleIntakeCommand::IsFinished() { return false; }