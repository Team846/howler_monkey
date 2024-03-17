#include "commands/prepare_short_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "subsystems/setpoints.h"

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

PrepareShortShootCommand::PrepareShortShootCommand(
    RobotContainer& container, double shooting_distance)
    : frc846::Loggable{"prepare_shoot_command"},
      scorer_(container.scorer_), pivot_(container.pivot_), 
        telescope_(container.telescope_), wrist_(container.wrist_), 
          dist_(shooting_distance) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("prepare_shoot_command");
}

void PrepareShortShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShortShootCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(kSpinUp));

  pivot_.SetTarget(pivot_.MakeTarget(units::degree_t(setpoints::kAutoShortShoot(0))));
  telescope_.SetTarget(telescope_.MakeTarget(0_in));

  // units::degree_t theta = units::degree_t(ShootingCalculator::calculate(scorer_.shooting_exit_velocity_.value(),
  //   dist_ + 1.4, 0.0, 0.0));

  wrist_.SetTarget(wrist_.MakeTarget(units::degree_t(setpoints::kAutoShortShoot(2))));//110_deg - wrist_.wrist_home_offset_.value() + theta));

  is_done_ = true;
}

void PrepareShortShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShortShootCommand::IsFinished() {
  return is_done_;
}