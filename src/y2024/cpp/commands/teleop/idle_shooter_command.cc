#include "commands/teleop/idle_shooter_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

IdleShooterCommand::IdleShooterCommand(RobotContainer& container)
    : frc846::Loggable{"idle_shooter_command"}, shooter_(container.shooter_) {
  AddRequirements({&shooter_});
  SetName("idle_shooter_command");
}

void IdleShooterCommand::Initialize() { Log("Idle Intake Command Initialize"); }

void IdleShooterCommand::Execute() {
  shooter_.SetTarget({ShooterState::kIdle});
}

void IdleShooterCommand::End(bool interrupted) {
  Log("Idle Shooter Command Finished");
}

bool IdleShooterCommand::IsFinished() { return false; }