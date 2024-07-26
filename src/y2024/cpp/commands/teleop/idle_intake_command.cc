#include "commands/teleop/idle_intake_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

IdleIntakeCommand::IdleIntakeCommand(RobotContainer& container)
    : frc846::Loggable{"idle_intake_command"}, intake_(container.intake_) {
  AddRequirements({&intake_});
  SetName("idle_intake_command");
}

void IdleIntakeCommand::Initialize() { Log("Idle Intake Command Initialize"); }

void IdleIntakeCommand::Execute() { intake_.SetTarget({IntakeState::kHold}); }

void IdleIntakeCommand::End(bool interrupted) {
  Log("Idle Intake Command Finished");
}

bool IdleIntakeCommand::IsFinished() { return false; }