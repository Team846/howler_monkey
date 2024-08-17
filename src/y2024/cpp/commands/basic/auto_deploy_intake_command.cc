#include "commands/basic/auto_deploy_intake_command.h"

AutoDeployIntakeCommand::AutoDeployIntakeCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, AutoDeployIntakeCommand>{
          container, "auto_deploy_intake_command"} {
  AddRequirements({&container_.intake_, &container_.shooter_,
                   &container_.super_structure_});
}

void AutoDeployIntakeCommand::OnInit() {}

void AutoDeployIntakeCommand::Periodic() {
  container_.intake_.SetTarget({kIntake});

  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getIntakeSetpoint());
}

void AutoDeployIntakeCommand::OnEnd(bool interrupted) {}

bool AutoDeployIntakeCommand::IsFinished() {
  return container_.super_structure_.hasReachedSetpoint(
      container_.super_structure_.getIntakeSetpoint());
}