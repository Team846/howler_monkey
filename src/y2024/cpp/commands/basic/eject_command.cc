#include "commands/basic/eject_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

EjectCommand::EjectCommand(RobotContainer& container)
    : frc846::Loggable{"eject_command"}, intake_(container.intake_) {
  AddRequirements({&intake_});
  SetName("eject_command");
}

void EjectCommand::Initialize() { Log("Eject Command Initialize"); }

void EjectCommand::Execute() { intake_.SetTarget({IntakeState::kRelease}); }

void EjectCommand::End(bool interrupted) { Log("Eject Command Finished"); }

bool EjectCommand::IsFinished() { return false; }