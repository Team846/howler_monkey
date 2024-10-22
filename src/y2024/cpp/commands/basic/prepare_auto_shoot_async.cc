#include "commands/basic/prepare_auto_shoot_async.h"

PrepareAutoShootAsyncCommand::PrepareAutoShootAsyncCommand(
    RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer,
                                    PrepareAutoShootAsyncCommand>{
          container, "prepare_auto_shoot_async_command"} {
  AddRequirements({&container_.super_structure_, &container_.intake_,
                   &container_.shooter_});
}

void PrepareAutoShootAsyncCommand::OnInit() { Periodic(); }

void PrepareAutoShootAsyncCommand::Periodic() {
  auto defaultShootSetpoint{container_.super_structure_.getAutoShootSetpoint()};

  container_.intake_.SetTarget({IntakeState::kHold});
  container_.shooter_.SetTarget({ShooterState::kRun});

  defaultShootSetpoint.wrist += defaultShootSetpoint.pivot;

  container_.super_structure_.SetTargetSetpoint(defaultShootSetpoint);
}

void PrepareAutoShootAsyncCommand::OnEnd(bool interrupted) {}

bool PrepareAutoShootAsyncCommand::IsFinished() { return true; }