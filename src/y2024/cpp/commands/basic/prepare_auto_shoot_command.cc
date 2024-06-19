#include "commands/basic/prepare_auto_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/abstract/super_structure.h"

PrepareAutoShootCommand::PrepareAutoShootCommand(RobotContainer& container)
    : frc846::Loggable{"prepare_auto_shoot_command"},
      intake_(container.intake_),
      
      vision_(container.vision_),
      super_(container.super_structure_) {
  AddRequirements({&container.pivot_, &container.wrist_, &container.telescope_,
                   &intake_});
  SetName("prepare_auto_shoot_command");
}

void PrepareAutoShootCommand::Initialize() {
  Log("Prepare Auto Shoot Command Initialize");
}

void PrepareAutoShootCommand::Execute() {
  VisionReadings vis_readings_ = vision_.readings();

  auto theta = shooting_calculator_
                   .calculateLaunchAngles(
                       intake_.shooting_exit_velocity_.value(),
                       vis_readings_.est_dist_from_speaker.to<double>(),
                       vis_readings_.velocity_in_component,
                       vis_readings_.velocity_orth_component,
                       super_.auto_shooter_height_.value().to<double>() / 12.0)
                   .launch_angle;

  intake_.SetTarget({IntakeState::kHold});
  

  auto defaultShootSetpoint{super_.getAutoShootSetpoint()};

  if (theta >= 1_deg) {
    defaultShootSetpoint.wrist = 90_deg + theta -
                                 super_.wrist_->wrist_home_offset_.value() +
                                 (defaultShootSetpoint.pivot -
                                  super_.pivot_->pivot_home_offset_.value());
  }

  super_.SetTargetSetpoint(defaultShootSetpoint);
}

void PrepareAutoShootCommand::End(bool interrupted) {
  Log("Prepare Auto Shoot Command Finished");
}

bool PrepareAutoShootCommand::IsFinished() {
  return true &&
         super_.pivot_->WithinTolerance(super_.getAutoShootSetpoint().pivot);
}