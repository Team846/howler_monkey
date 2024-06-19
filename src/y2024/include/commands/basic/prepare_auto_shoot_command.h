#ifndef y2024_COMMANDS_PREPARE_AUTO_SHOOT_COMMAND_H_
#define y2024_COMMANDS_PREPARE_AUTO_SHOOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "constants.h"
#include "frc846/util/math.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/pivot.h"

#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"
#include "subsystems/robot_container.h"

class PrepareAutoShootCommand
    : public frc2::CommandHelper<frc2::Command, PrepareAutoShootCommand>,
      public frc846::Loggable {
 public:
  PrepareAutoShootCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem& intake_;

  VisionSubsystem& vision_;
  SuperStructureSubsystem& super_;

  ShootingCalculator shooting_calculator_;
};

#endif  // y2024_COMMANDS_PREPARE_AUTO_SHOOT_COMMAND_H_