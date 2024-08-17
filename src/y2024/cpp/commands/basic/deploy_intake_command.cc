#include "commands/basic/deploy_intake_command.h"

DeployIntakeCommand::DeployIntakeCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, DeployIntakeCommand>{
          container, "deploy_intake_command"} {
  AddRequirements({&container_.intake_, &container_.super_structure_});
}

void DeployIntakeCommand::OnInit() {}

void DeployIntakeCommand::Periodic() {
  container_.intake_.SetTarget({kIntake});
  container_.shooter_.SetTarget({kIdle});

  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getIntakeSetpoint());
}

void DeployIntakeCommand::OnEnd(bool interrupted) {}

bool DeployIntakeCommand::IsFinished() { return false; }