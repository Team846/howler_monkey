#include "commands/basic/wrist_zero_command.h"

WristZeroCommand::WristZeroCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, WristZeroCommand>{
          container, "wrist_zero_command"} {
  AddRequirements({&container_.super_structure_});
}

void WristZeroCommand::OnInit() {
  Log("Init Wrist Zero Command.");
  container_.super_structure_.wrist_->HomeSubsystem();
}

void WristZeroCommand::Periodic() {
  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getStowSetpoint());
}

void WristZeroCommand::OnEnd(bool interrupted) {}

bool WristZeroCommand::IsFinished() {
  return container_.super_structure_.wrist_->GetHasZeroed();
}