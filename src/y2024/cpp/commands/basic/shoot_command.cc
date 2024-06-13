#include "commands/basic/shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

ShootCommand::ShootCommand(RobotContainer& container)
    : frc846::Loggable{"shoot_command"},
      intake_(container.intake_),
      shooter_(container.shooter_) {
  AddRequirements({&intake_, &shooter_});
  SetName("shoot_command");
}

void ShootCommand::Initialize() { Log("Shoot Command Initialize"); }

void ShootCommand::Execute() {
  intake_.SetTarget({IntakeState::kFeed});
  shooter_.SetTarget({ShooterState::kRun});
}

void ShootCommand::End(bool interrupted) { Log("Shoot Command Finished"); }

bool ShootCommand::IsFinished() { return intake_.GetHasPiece(); }