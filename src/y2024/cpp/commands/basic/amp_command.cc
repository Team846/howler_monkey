#include "commands/basic/amp_command.h"

AmpCommand::AmpCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, AmpCommand>{container,
                                                                "amp_command"} {
  AddRequirements({&container_.super_structure_});
}

void AmpCommand::OnInit() {}

void AmpCommand::Periodic() {
  container_.super_structure_.SetTargetSetpoint(
      container_.super_structure_.getAmpSetpoint());
}

void AmpCommand::OnEnd(bool interrupted) {}

bool AmpCommand::IsFinished() { return false; }