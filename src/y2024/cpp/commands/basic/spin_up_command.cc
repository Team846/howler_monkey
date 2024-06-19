#include "commands/basic/spin_up_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

SpinUpCommand::SpinUpCommand(RobotContainer& container)
    : frc846::Loggable{"spin_up_command"} {
  
  SetName("spin_up_command");
}

void SpinUpCommand::Initialize() {
  Log("Spin Up Command Initialize");
  is_done_ = false;
}

void SpinUpCommand::Execute() {
  //er_.SetTarget({ShooterState::kRun}); //flag
  is_done_ = true;
}

void SpinUpCommand::End(bool interrupted) { Log("Spin Up Command Finished"); }

bool SpinUpCommand::IsFinished() { return is_done_; }