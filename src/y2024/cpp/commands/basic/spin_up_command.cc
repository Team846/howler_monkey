#include "commands/basic/spin_up_command.h"

SpinUpCommand::SpinUpCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, SpinUpCommand>{
          container, "spin_up_command"} {
  AddRequirements({&container_.shooter_});
}

void SpinUpCommand::OnInit() {
  has_spinned_up_ = false;
  start_time = frc846::wpilib::CurrentFPGATime();
}

void SpinUpCommand::Periodic() {
  container_.shooter_.SetTarget({ShooterState::kRun});
  if (container_.shooter_.GetReadings().error_percent <
      container_.shooter_.shooter_speed_tolerance_.value()) {
    if (has_spinned_up_ == false) {
      Log("Shooter took {}s to spin up.",
          frc846::wpilib::CurrentFPGATime() - start_time);
      has_spinned_up_ = true;
    }
  }
}

void SpinUpCommand::OnEnd(bool interrupted) {}

bool SpinUpCommand::IsFinished() { return false; }