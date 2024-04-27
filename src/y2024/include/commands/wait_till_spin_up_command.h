#ifndef y2024_COMMANDS_WAIT_TILL_SPIN_UP_COMMAND_H_
#define y2024_COMMANDS_WAIT_TILL_SPIN_UP_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/shooter.h"
#include "subsystems/intake.h"
#include "subsystems/super_structure.h"
#include "subsystems/robot_container.h"

class WaitTillSpinUpCommand
    : public frc2::CommandHelper<frc2::Command, WaitTillSpinUpCommand>,
      public frc846::Loggable {
 public:
  WaitTillSpinUpCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem& shooter_;
  SuperStructureSubsystem& super_;
};

#endif  // y2024_COMMANDS_WAIT_TILL_SPIN_UP_COMMAND_H_