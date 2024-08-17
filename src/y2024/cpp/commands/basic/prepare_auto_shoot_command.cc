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

  // auto theta =
  //     shooting_calculator_
  //         .calculateLaunchAngles(
  //             shooter_.shooting_exit_velocity_.value(),
  //             vis_readings_.est_dist_from_speaker.to<double>() +
  //                 super_.auto_shooter_x_.value().to<double>() / 12.0,
  //             0.0, 0.0, super_.auto_shooter_height_.value().to<double>()
  //             / 12.0, defaultShootSetpoint.wrist.to<double>())
  //         .launch_angle;

  container_.intake_.SetTarget({IntakeState::kHold});
  container_.shooter_.SetTarget({ShooterState::kRun});

  // if (theta <= 1_deg) {
  // theta = defaultShootSetpoint.wrist;
  // }

  defaultShootSetpoint.wrist += defaultShootSetpoint.pivot;

  // Log("Theta {}", theta);

  container_.super_structure_.SetTargetSetpoint(defaultShootSetpoint);
}

void PrepareAutoShootCommand::OnEnd(bool interrupted) {}

bool PrepareAutoShootCommand::IsFinished() {
  return container_.shooter_.GetReadings().error_percent <=
             container_.shooter_.shooter_speed_tolerance_.value() &&
         container_.super_structure_.pivot_->WithinTolerance(
             container_.super_structure_.getAutoShootSetpoint().pivot);
}