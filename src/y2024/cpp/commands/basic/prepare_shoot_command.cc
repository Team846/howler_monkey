#include "commands/basic/prepare_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/abstract/super_structure.h"

PrepareShootCommand::PrepareShootCommand(RobotContainer& container,
                                         bool super_shot)
    : frc846::Loggable{"prepare_shoot_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      vision_(container.vision_),
      super_(container.super_structure_),
      control_input_(container.control_input_),
      super_shot_(super_shot) {
  AddRequirements({&super_,
                   &intake_, &shooter_});
  SetName("prepare_shoot_command");
}

void PrepareShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShootCommand::Execute() {
  VisionReadings vis_readings_ = vision_.readings();

  auto shootSetpoint = super_.getShootSetpoint();

  intake_.SetTarget({IntakeState::kHold});
  shooter_.SetTarget({ShooterState::kRun});

  if (super_shot_) {
    auto theta =
      shooting_calculator_
          .calculateLaunchAngles(
              shooter_.shooting_exit_velocity_.value(),
              vis_readings_.est_dist_from_speaker.to<double>() +
                  super_.teleop_shooter_x_.value().to<double>() / 12.0,
              vis_readings_.velocity_in_component,
              vis_readings_.velocity_orth_component,
              super_.teleop_shooter_height_.value().to<double>() / 12.0,
              shootSetpoint.wrist.to<double>())
          .launch_angle;

    shootSetpoint.wrist = theta + shootSetpoint.pivot;
  } else {
    shootSetpoint.wrist += shootSetpoint.pivot;
  }

  if (shooter_.readings().error_percent <=
      shooter_.shooter_speed_tolerance_.value()) {
    frc846::util::ShareTables::SetBoolean("ready_to_shoot", true);
  } else {
    frc846::util::ShareTables::SetBoolean("ready_to_shoot", false);
  }

  super_.SetTargetSetpoint(shootSetpoint);
}

void PrepareShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShootCommand::IsFinished() { return false; }