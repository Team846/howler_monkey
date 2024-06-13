#ifndef y2024_COMMANDS_SHOOT_COMMAND_H_
#define y2024_COMMANDS_SHOOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/robot_container.h"

class ShootCommand : public frc2::CommandHelper<frc2::Command, ShootCommand>,
                     public frc846::Loggable {
 public:
  ShootCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;
  ShooterSubsystem& shooter_;
};

#endif  // y2024_COMMANDS_SHOOT_COMMAND_H_