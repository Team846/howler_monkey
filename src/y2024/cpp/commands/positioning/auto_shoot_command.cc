#include "commands/positioning/auto_Shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

#include "subsystems/setpoints.h"

AutoShootCommand::AutoShootCommand(
    RobotContainer& container, double pivot_target, double wrist_target, double tele_target)
    : frc846::Loggable{"auto_Shoot_command"}, pivot_(container.pivot_),
        telescope_(container.telescope_), wrist_(container.wrist_), pivot_target_(pivot_target), wrist_target_(wrist_target), tele_target_(tele_target) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("auto_Shoot_command");
}

void AutoShootCommand::Initialize() {
  Log("AutoShoot Command Initialize");
}

void AutoShootCommand::Execute() {

  units::degree_t nextPivotTarget = units::degree_t(pivot_target_);
  units::inch_t nextTelescopeTarget = units::inch_t(tele_target_);
  units::degree_t nextWristTarget = units::degree_t(wrist_target_);

  telescope_.SetTarget({nextTelescopeTarget});
  pivot_.SetTarget({nextPivotTarget});
  wrist_.SetTarget({nextWristTarget});

  //TODO Adjustments
  
  //TODO: Intermediates?

  bool pivot_done_=units::math::abs(pivot_.readings().pivot_position-nextPivotTarget)<pivot_.pivot_position_tolerance_.value();
  bool telescope_done_=units::math::abs(telescope_.readings().extension-nextTelescopeTarget)<telescope_.telescope_position_tolerance_.value();
  bool wrist_done_=units::math::abs(wrist_.readings().wrist_position-nextWristTarget)<wrist_.wrist_position_tolerance_.value();

  is_done_ = pivot_done_&&telescope_done_&&wrist_done_;
}

void AutoShootCommand::End(bool interrupted) {
  Log("AutoShoot Command Finished");
}

bool AutoShootCommand::IsFinished() {
  return false;
}