#include "commands/basic/eject_command.h"

EjectCommand::EjectCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, EjectCommand>{
          container, "eject_command"} {
  AddRequirements({&container_.intake_});
}

void EjectCommand::OnInit() {}

void EjectCommand::Periodic() {
  container_.intake_.SetTarget({IntakeState::kRelease});
}

void EjectCommand::OnEnd(bool interrupted) {}

bool EjectCommand::IsFinished() { return false; }