#include "commands/basic/spin_up_command.h"

SpinUpCommand::SpinUpCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, SpinUpCommand>{
          container, "spin_up_command"} {
  AddRequirements({&container_.shooter_});
}

void SpinUpCommand::OnInit() {}

void SpinUpCommand::Periodic() {
  container_.shooter_.SetTarget({ShooterState::kRun});
}

void SpinUpCommand::OnEnd(bool interrupted) {}

bool SpinUpCommand::IsFinished() { return false; }