#include "commands/basic/prepare_shoot_command.h"

PrepareShootCommand::PrepareShootCommand(RobotContainer& container,
                                         bool super_shot)
    : frc846::robot::GenericCommand<
          RobotContainer, PrepareShootCommand>{container,
                                               "prepare_shoot_command"},
      super_shot_(super_shot) {
  AddRequirements({&container_.super_structure_});

  ShootingCalculatorConstants sc_constants{78.0_in};
  shooting_calculator_.setConstants(sc_constants);
}

void PrepareShootCommand::OnInit() {}

void PrepareShootCommand::Periodic() {
  VisionReadings vis_readings_ = container_.vision_.GetReadings();

  auto shootSetpoint = container_.super_structure_.getShootSetpoint();

  // container_.intake_.SetTarget({IntakeState::kHold});
  // container_.shooter_.SetTarget({ShooterState::kRun});

  if (super_shot_) {
    ShootingCalculatorInput sc_input = {
        vis_readings_.est_dist_from_speaker +
            container_.super_structure_.teleop_shooter_x_.value(),
        vis_readings_.velocity_in_component * 1_fps,
        vis_readings_.velocity_orth_component * 1_fps,
        container_.super_structure_.teleop_shooter_height_.value(),
        shootSetpoint.wrist,
        container_.shooter_.shooting_exit_velocity_.value() * 1_fps};

    auto theta = shooting_calculator_.calculate(sc_input).launch_angle;

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