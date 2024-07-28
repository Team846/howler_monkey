#include "commands/basic/amp_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

AmpCommand::AmpCommand(RobotContainer& container)
    : frc846::base::Loggable{"amp_command"},
      super_(container.super_structure_) {
  AddRequirements({&super_});
  SetName("amp_command");
}

void AmpCommand::Initialize() { Log("Amp Command Initialize"); }

void AmpCommand::Execute() {
  super_.SetTargetSetpoint(super_.getAmpSetpoint());
}

void AmpCommand::End(bool interrupted) { Log("Amp Command Finished"); }

bool AmpCommand::IsFinished() { return false; }