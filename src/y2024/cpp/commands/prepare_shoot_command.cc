#include "commands/prepare_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/abstract/super_structure.h"

PrepareShootCommand::PrepareShootCommand(RobotContainer& container)
    : frc846::Loggable{"prepare_shoot_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      pivot_(container.pivot_),
      telescope_(container.telescope_),
      wrist_(container.wrist_),
      vision_(container.vision_),
      super_(container.super_structure_) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("prepare_shoot_command");
}

void PrepareShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShootCommand::Execute() {
  VisionReadings vis_readings_ = vision_.readings();

  auto theta = shooting_calculator_
                   .calculateLaunchAngles(
                       shooter_.shooting_exit_velocity_.value(),
                       vis_readings_.est_dist_from_speaker.to<double>(),
                       vis_readings_.velocity_in_component,
                       vis_readings_.velocity_orth_component,
                       super_.auto_shooter_height_.value().to<double>() / 12.0)
                   .launch_angle;

  intake_.SetTarget(intake_.MakeTarget(IntakeState::kHold));
  shooter_.SetTarget(shooter_.MakeTarget(ShooterState::kRun));

  pivot_.SetTarget(pivot_.MakeTarget(super_.getAutoShootSetpoint().pivot));
  telescope_.SetTarget(
      telescope_.MakeTarget(super_.getAutoShootSetpoint().telescope));

  wrist_.SetTarget(wrist_.MakeTarget(theta));

  is_done_ = false;
}

void PrepareShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShootCommand::IsFinished() { return is_done_; }