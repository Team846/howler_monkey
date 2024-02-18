#ifndef y2024_COMMANDS_LEDS_COMMAND_H_
#define y2024_COMMANDS_LEDS_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/leds.h"
#include "subsystems/scorer.h"
#include "subsystems/operator.h"
#include "subsystems/robot_container.h"

class LEDsCommand
    : public frc2::CommandHelper<frc2::Command, LEDsCommand>,
      public frc846::Loggable {
 public:
  LEDsCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;


 private:
  LEDsSubsystem& leds_;
  ScorerSubsystem& scorer_;
  OperatorSubsystem& operator_;
};

#endif  // y2024_COMMANDS_STOW_COMMAND_H_