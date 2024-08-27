#include "commands/basic/prepare_shoot_command.h"

PrepareShootCommand::PrepareShootCommand(RobotContainer& container,
                                         bool super_shot)
    : frc846::robot::GenericCommand<
          RobotContainer, PrepareShootCommand>{container,
                                               "prepare_shoot_command"},
      super_shot_(super_shot) {
  AddRequirements({&container_.super_structure_});
}

void PrepareShootCommand::OnInit() {}

void PrepareShootCommand::Periodic() {
  VisionReadings vis_readings_ = container_.vision_.GetReadings();

  auto shootSetpoint = container_.super_structure_.getShootSetpoint();

  // container_.intake_.SetTarget({IntakeState::kHold});
  // container_.shooter_.SetTarget({ShooterState::kRun});

  if (super_shot_) {
    auto theta =
        shooting_calculator_
            .calculateLaunchAngles(
                container_.shooter_.shooting_exit_velocity_.value(),
                vis_readings_.est_dist_from_speaker.to<double>() +
                    container_.super_structure_.teleop_shooter_x_.value()
                            .to<double>() /
                        12.0,
                vis_readings_.velocity_in_component,
                vis_readings_.velocity_orth_component,
                container_.super_structure_.teleop_shooter_height_.value()
                        .to<double>() /
                    12.0,
                shootSetpoint.wrist.to<double>())
            .launch_angle;

    shootSetpoint.wrist = theta + shootSetpoint.pivot;
  } else {
    shootSetpoint.wrist += shootSetpoint.pivot;
  }

  if (container_.shooter_.GetReadings().error_percent <=
      container_.shooter_.shooter_speed_tolerance_.value()) {
    frc846::util::ShareTables::SetBoolean("ready_to_shoot", true);
  } else {
    frc846::util::ShareTables::SetBoolean("ready_to_shoot", false);
  }

  container_.super_structure_.SetTargetSetpoint(shootSetpoint);
}

void PrepareShootCommand::OnEnd(bool interrupted) {}

bool PrepareShootCommand::IsFinished() { return false; }