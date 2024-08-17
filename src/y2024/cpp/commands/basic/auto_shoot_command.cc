#include "commands/basic/auto_shoot_command.h"

AutoShootCommand::AutoShootCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, AutoShootCommand>{
          container, "auto_shoot_command"} {
  AddRequirements({&container_.intake_, &container_.shooter_});
}

void AutoShootCommand::OnInit() {}

void AutoShootCommand::Periodic() {
  if (container_.shooter_.GetReadings().error_percent <
      container_.shooter_.shooter_speed_tolerance_.value()) {
    container_.intake_.SetTarget({IntakeState::kFeed});
  } else {
    container_.intake_.SetTarget({IntakeState::kHold});
  }
  container_.shooter_.SetTarget({ShooterState::kRun});
}

void AutoShootCommand::OnEnd(bool interrupted) {
  Log("Auto Shoot Command Finished");
}

bool AutoShootCommand::IsFinished() {
  return !container_.intake_.GetHasPiece();
}