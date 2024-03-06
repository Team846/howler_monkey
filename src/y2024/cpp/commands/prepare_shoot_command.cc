#include "commands/prepare_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

PrepareShootCommand::PrepareShootCommand(
    RobotContainer& container, double shooting_distance)
    : frc846::Loggable{"prepare_shoot_command"},
      scorer_(container.scorer_), pivot_(container.pivot_), 
        telescope_(container.telescope_), wrist_(container.wrist_), 
          dist_(shooting_distance) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("prepare_shoot_command");
}

void PrepareShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareShootCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(kSpinUp));

  pivot_.SetTarget(pivot_.MakeTarget(38.0_deg));
  telescope_.SetTarget(telescope_.MakeTarget(0_in));

  std::cout << "DIST" << dist_ << std::endl;

  units::degree_t theta = units::degree_t(ShootingCalculator::calculate(scorer_.shooting_exit_velocity_.value(),
    dist_ + 1.4, 0.0, 0.0));

  std::cout << "T" << theta.to<double>() << std::endl;

  wrist_.SetTarget(wrist_.MakeTarget(112_deg));//110_deg - wrist_.wrist_home_offset_.value() + theta));

  is_done_ = true;
}

void PrepareShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareShootCommand::IsFinished() {
  return is_done_;
}