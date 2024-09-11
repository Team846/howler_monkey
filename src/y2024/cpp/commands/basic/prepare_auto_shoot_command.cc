#include "commands/basic/prepare_auto_shoot_command.h"

PrepareAutoShootCommand::PrepareAutoShootCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, PrepareAutoShootCommand>{
          container, "prepare_auto_shoot_command"} {
  AddRequirements({&container_.super_structure_, &container_.intake_,
                   &container_.shooter_});
}

void PrepareAutoShootCommand::OnInit() {}

void PrepareAutoShootCommand::Periodic() {
  // VisionReadings vis_readings_ = container_.vision_.GetReadings();

  auto defaultShootSetpoint{container_.super_structure_.getAutoShootSetpoint()};

  container_.intake_.SetTarget({IntakeState::kHold});
  container_.shooter_.SetTarget({ShooterState::kRun});

  // if (theta <= 1_deg) {
  // theta = defaultShootSetpoint.wrist;
  // }

  defaultShootSetpoint.wrist += defaultShootSetpoint.pivot;

  container_.super_structure_.SetTargetSetpoint(defaultShootSetpoint);
}

void PrepareAutoShootCommand::OnEnd(bool interrupted) {}

bool PrepareAutoShootCommand::IsFinished() {
  return container_.shooter_.GetReadings().error_percent <=
             container_.shooter_.shooter_speed_tolerance_.value() &&
         container_.super_structure_.pivot_->WithinTolerance(
             container_.super_structure_.getAutoShootSetpoint().pivot);
}