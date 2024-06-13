#ifndef y2024_COMMANDS_OPERATOR_CONTROL_COMMAND_H_
#define y2024_COMMANDS_OPERATOR_CONTROL_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "frc846/control/sequencer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/pivot.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/wrist.h"
#include "subsystems/robot_container.h"

class OperatorControlCommand
    : public frc2::CommandHelper<frc2::Command, OperatorControlCommand> {
 public:
  OperatorControlCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  SuperStructureSubsystem& super_;
};

#endif  // y2024_COMMANDS_OPERATOR_CONTROL_COMMAND_H_