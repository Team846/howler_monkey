#include "commands/basic/shoot_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

ShootCommand::ShootCommand(RobotContainer& container)
    : frc846::Loggable{"shoot_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      control_input_(container.control_input_) {
  AddRequirements({&intake_, &shooter_});
  SetName("shoot_command");
}

void ShootCommand::Initialize() { Log("Shoot Command Initialize"); }

void ShootCommand::Execute() {
  intake_.SetTarget({IntakeState::kFeed});
  shooter_.SetTarget({ShooterState::kRun});
}

void ShootCommand::End(bool interrupted) {
  frc846::util::ShareTables::SetBoolean("ready_to_shoot", false);
  
  intake_.SetTarget({IntakeState::kHold});
  shooter_.SetTarget({ShooterState::kIdle});

  Log("Shoot Command Finished");
}

bool ShootCommand::IsFinished() { return !control_input_.readings().shooting; }