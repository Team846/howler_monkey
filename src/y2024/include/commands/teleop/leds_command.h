#ifndef y2024_COMMANDS_LEDS_COMMAND_H_
#define y2024_COMMANDS_LEDS_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/hardware/leds.h"
#include "subsystems/robot_container.h"

class LEDsCommand : public frc2::CommandHelper<frc2::Command, LEDsCommand> {
 public:
  LEDsCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  LEDsSubsystem& leds_;
  SuperStructureSubsystem& super_;
};

#endif  // y2024_COMMANDS_LEDS_COMMAND_H_