#include "commands/basic/trap_command.h"

TrapCommand::TrapCommand(RobotContainer& container, int stage)
    : frc846::robot::GenericCommand<RobotContainer,
                                    TrapCommand>{container, "trap_command"},
      stage_{stage} {
  AddRequirements({&container_.super_structure_});
}

void TrapCommand::OnInit() {}

void TrapCommand::Periodic() {
  if (stage_ == 1) {
    container_.super_structure_.SetTargetSetpoint(
        container_.super_structure_.getPreClimbSetpoint());
  }
  if (stage_ == 2) {
    auto pullClimbTarget = container_.super_structure_.getPreClimbSetpoint();
    pullClimbTarget.pivot =
        container_.super_structure_.pivot_pull_target_.value();
    container_.super_structure_.SetTargetSetpoint(pullClimbTarget, true);
  }
  if (stage_ == 3) {
    container_.super_structure_.SetTargetSetpoint(
        container_.super_structure_.getPreScoreSetpoint());
  }
  if (stage_ == 4) {
    container_.super_structure_.SetTargetSetpoint(
        container_.super_structure_.getTrapScoreSetpoint());
  }
  if (stage_ == 5) {
    container_.super_structure_.SetTargetSetpoint(
        container_.super_structure_.getTrapScoreSetpoint());
  }
  if (stage_ == 6) {
    container_.super_structure_.SetTargetSetpoint(
        container_.super_structure_.getPostScoreSetpoint());
  }
}

void TrapCommand::OnEnd(bool interrupted) {}

bool TrapCommand::IsFinished() { return false; }