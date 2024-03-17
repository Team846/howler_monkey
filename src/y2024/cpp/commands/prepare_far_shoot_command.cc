#include "commands/prepare_far_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "subsystems/setpoints.h"

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

PrepareFarShootCommand::PrepareFarShootCommand(
    RobotContainer& container, double shooting_distance)
    : frc846::Loggable{"prepare_shoot_command"},
      scorer_(container.scorer_), pivot_(container.pivot_), 
        telescope_(container.telescope_), wrist_(container.wrist_), 
          dist_(shooting_distance) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("prepare_shoot_command");
}

void PrepareFarShootCommand::Initialize() {
  Log("Prepare Shoot Command Initialize");
}

void PrepareFarShootCommand::Execute() {
  scorer_.SetTarget(scorer_.MakeTarget(kSpinUp));

  pivot_.SetTarget(pivot_.MakeTarget(units::degree_t(setpoints::kAutoFarShoot(0))));
  telescope_.SetTarget(telescope_.MakeTarget(0_in));
  wrist_.SetTarget(wrist_.MakeTarget(units::degree_t(setpoints::kAutoFarShoot(2))));//110_deg - wrist_.wrist_home_offset_.value() + theta));

  is_done_ = true;
}

void PrepareFarShootCommand::End(bool interrupted) {
  Log("Prepare Shoot Command Finished");
}

bool PrepareFarShootCommand::IsFinished() {
  return is_done_;
}