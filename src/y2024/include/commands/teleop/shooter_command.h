#ifndef y2024_COMMANDS_SHOOTER_COMMAND_H_
#define y2024_COMMANDS_SHOOTER_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/robot_container.h"

class ShooterCommand
    : public frc2::CommandHelper<frc2::Command, ShooterCommand> {
 public:
  ShooterCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  ShooterSubsystem& shooter_;
  SuperStructureSubsystem& super_;

  ControlInputReadings prev_ci_readings_{};
};

#endif  // y2024_COMMANDS_TELESCOPE_COMMAND_H_