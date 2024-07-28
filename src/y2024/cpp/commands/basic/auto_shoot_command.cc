#include "commands/basic/auto_shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

AutoShootCommand::AutoShootCommand(RobotContainer& container)
    : frc846::base::Loggable{"auto_shoot_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      control_input_(container.control_input_) {
  AddRequirements({&intake_, &shooter_});
  SetName("shoot_command");
}

void AutoShootCommand::Initialize() {
  Log("Auto Shoot Command Initialize");
  is_done_ = false;
}

void AutoShootCommand::Execute() {
  intake_.SetTarget({IntakeState::kFeed});
  shooter_.SetTarget({ShooterState::kRun});

  is_done_ = true;
}

void AutoShootCommand::End(bool interrupted) {
  Log("Auto Shoot Command Finished");
}

bool AutoShootCommand::IsFinished() { return !intake_.GetHasPiece(); }