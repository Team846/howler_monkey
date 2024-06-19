#ifndef y2024_COMMANDS_IDLE_COMMAND_H_
#define y2024_COMMANDS_IDLE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/hardware/intake.h"

#include "subsystems/robot_container.h"

class IdleCommand : public frc2::CommandHelper<frc2::Command, IdleCommand>,
                    public frc846::Loggable {
 public:
  IdleCommand(RobotContainer& container, bool onlyIntake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;


  bool onlyIntake_;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_IDLE_COMMAND_H_