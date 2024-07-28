#include "commands/basic/pull_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

PullCommand::PullCommand(RobotContainer& container)
    : frc846::base::Loggable{"pull_command"},
      intake_(container.intake_),
      shooter_(container.shooter_) {
  AddRequirements({&intake_, &shooter_});
  SetName("pull_command");
}

void PullCommand::Initialize() { Log("Pull Command Initialize"); }

void PullCommand::Execute() {
  intake_.SetTarget({IntakeState::kPull});
  shooter_.SetTarget({ShooterState::kIdle});
}

void PullCommand::End(bool interrupted) { Log("Pull Command Finished"); }

bool PullCommand::IsFinished() { return false; }