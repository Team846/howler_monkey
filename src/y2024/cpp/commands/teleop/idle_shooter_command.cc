#include "commands/teleop/idle_shooter_command.h"

IdleShooterCommand::IdleShooterCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, IdleShooterCommand>{
          container, "idle_shooter_command"} {
  AddRequirements({&container_.shooter_});
}

void IdleShooterCommand::OnInit() {}

void IdleShooterCommand::Periodic() {
  container_.shooter_.SetTarget({ShooterState::kIdle});
}

void IdleShooterCommand::OnEnd(bool interrupted) {}

bool IdleShooterCommand::IsFinished() { return false; }