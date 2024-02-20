#include "commands/prepare_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

PrepareShootCommand::PrepareShootCommand(
    RobotContainer& container, units::foot_t shooting_distance)
    : frc846::Loggable{"prepare_shoot_command"},
      scorer_(container.scorer_), pivot_(container.pivot_), 
        telescope_(container.telescope_), wrist_(container.wrist_), 
          dist_(shooting_distance.to<double>()) {
  AddRequirements({&pivot_, &telescope_, &wrist_}); //don't add scorer, only set target for scorer once during initialize
  SetName("prepare_shoot_command");
}

void PrepareShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShootCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(false, false, scorer_.shooter_speed_.value()));

  pivot_.SetTarget(pivot_.MakeTarget(11_deg));
  telescope_.SetTarget(telescope_.MakeTarget(4.7_in));

  double SPEAKER_HEIGHT = 4.7;
  double LAUNCH_VELOCITY = 27.0;
  double GRAVITY = 32.0;

  units::degree_t theta = units::radian_t(std::abs(std::acos((dist_/
  (std::sqrt(2*SPEAKER_HEIGHT / GRAVITY)))/LAUNCH_VELOCITY)));

  wrist_.SetTarget(wrist_.MakeTarget(theta));

  is_done_ = true;
}

void PrepareShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShootCommand::IsFinished() {
  return is_done_;
}