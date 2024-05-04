#ifndef y2024_COMMANDS_CONTROL_INPUT_COMMAND_H_
#define y2024_COMMANDS_CONTROL_INPUT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/driver.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/robot_container.h"

class ControlInputCommand
    : public frc2::CommandHelper<frc2::Command, ControlInputCommand> {
 public:
  ControlInputCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriverSubsystem& driver_;
  OperatorSubsystem& operator_;
  ControlInputSubsystem& control_input_;
  SuperStructureSubsystem& super_;

  DriverReadings previous_driver_;
  OperatorReadings previous_operator_;

  ControlInputReadings ci_readings_;
};

#endif  // y2024_COMMANDS_CONTROL_INPUT_COMMAND_H_