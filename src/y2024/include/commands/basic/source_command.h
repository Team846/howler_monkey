#ifndef y2024_COMMANDS_SOURCE_COMMAND_H_
#define y2024_COMMANDS_SOURCE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/pivot.h"

#include "subsystems/hardware/telescope.h"
#include "subsystems/hardware/wrist.h"
#include "subsystems/robot_container.h"

class SourceCommand : public frc2::CommandHelper<frc2::Command, SourceCommand>,
                      public frc846::Loggable {
 public:
  SourceCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SuperStructureSubsystem& super_;
  IntakeSubsystem& intake_;
};

#endif  // y2024_COMMANDS_SOURCE_COMMAND_H_