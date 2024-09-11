#include "commands/basic/shoot_command.h"

ShootCommand::ShootCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, ShootCommand>{
          container, "shoot_command"} {
  AddRequirements({&container_.intake_, &container_.shooter_});
}

void ShootCommand::OnInit() { num_loops_ = 0; }

void ShootCommand::Periodic() {
  container_.intake_.SetTarget({IntakeState::kFeed});
  container_.shooter_.SetTarget({ShooterState::kRun});
}

void ShootCommand::OnEnd(bool interrupted) {
  frc846::util::ShareTables::SetBoolean("ready_to_shoot", false);
}

bool ShootCommand::IsFinished() {
  if (!container_.intake_.GetHasPiece()) {
    num_loops_ += 1;
  } else {
    num_loops_ = 0;
  }

  return num_loops_ > 20;
}