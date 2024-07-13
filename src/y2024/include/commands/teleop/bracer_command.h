#pragma once

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/hardware/bracer.h"
#include "subsystems/robot_container.h"

class BracerCommand : public frc2::CommandHelper<frc2::Command, BracerCommand> {
 public:
  BracerCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  int counter = 120;

  ControlInputSubsystem& control_input_;
  BracerSubsystem& bracer_;
  SuperStructureSubsystem& super_;

  ControlInputReadings prev_ci_readings_{};
};
