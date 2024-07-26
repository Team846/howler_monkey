#include "commands/basic/spin_up_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

SpinUpCommand::SpinUpCommand(RobotContainer& container)
    : frc846::Loggable{"spin_up_command"}, shooter_(container.shooter_) {
  AddRequirements({&shooter_});
  SetName("spin_up_command");
}

void SpinUpCommand::Initialize() {
  Log("Spin Up Command Initialize");
}

void SpinUpCommand::Execute() {
  shooter_.SetTarget({ShooterState::kRun});
}

void SpinUpCommand::End(bool interrupted) { Log("Spin Up Command Finished"); }

bool SpinUpCommand::IsFinished() { return false; }