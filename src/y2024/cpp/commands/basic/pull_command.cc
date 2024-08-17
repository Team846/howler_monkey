#include "commands/basic/pull_command.h"

PullCommand::PullCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, PullCommand>{
          container, "pull_command"} {
  AddRequirements({&container_.intake_, &container_.shooter_});
}

void PullCommand::OnInit() {}

void PullCommand::Periodic() {
  container_.intake_.SetTarget({IntakeState::kPull});
  container_.shooter_.SetTarget({ShooterState::kIdle});
}

void PullCommand::OnEnd(bool interrupted) {}

bool PullCommand::IsFinished() { return false; }