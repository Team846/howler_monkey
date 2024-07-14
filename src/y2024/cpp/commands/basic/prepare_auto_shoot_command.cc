#include "commands/basic/prepare_auto_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/abstract/super_structure.h"

PrepareAutoShootCommand::PrepareAutoShootCommand(RobotContainer& container)
    : frc846::Loggable{"prepare_auto_shoot_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      vision_(container.vision_),
      super_(container.super_structure_) {
  AddRequirements({&container.pivot_, &container.wrist_, &container.telescope_,
                   &intake_, &shooter_});
  SetName("prepare_auto_shoot_command");
}

void PrepareAutoShootCommand::Initialize() {
  Log("Prepare Auto Shoot Command Initialize");
}

void PrepareAutoShootCommand::Execute() {
  VisionReadings vis_readings_ = vision_.readings();

  // auto theta =
  //     shooting_calculator_
  //         .calculateLaunchAngles(
  //             shooter_.shooting_exit_velocity_.value(),
  //             vis_readings_.est_dist_from_speaker.to<double>(), 0.0, 0.0,
  //             super_.auto_shooter_height_.value().to<double>() / 12.0)
  //         .launch_angle;

  intake_.SetTarget({IntakeState::kHold});
  shooter_.SetTarget({ShooterState::kRun});

  auto defaultShootSetpoint{super_.getAutoShootSetpoint()};

  // if (theta >= 1_deg) {
  //   defaultShootSetpoint.wrist = theta + defaultShootSetpoint.pivot;
  // }

  // Log("Theta {}", theta);

  super_.SetTargetSetpoint(defaultShootSetpoint);
}

void PrepareAutoShootCommand::End(bool interrupted) {
  Log("Prepare Auto Shoot Command Finished");
}

bool PrepareAutoShootCommand::IsFinished() {
  return shooter_.readings().error_percent <=
             shooter_.shooter_speed_tolerance_.value() &&
         super_.pivot_->WithinTolerance(super_.getAutoShootSetpoint().pivot);
}