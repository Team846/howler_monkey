#ifndef y2024_COMMANDS_EJECT_COMMAND_H_
#define y2024_COMMANDS_EJECT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/robot_container.h"

class EjectCommand : public frc2::CommandHelper<frc2::Command, EjectCommand>,
                     public frc846::Loggable {
 public:
  EjectCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_EJECT_COMMAND_H_