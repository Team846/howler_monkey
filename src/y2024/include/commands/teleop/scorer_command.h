#ifndef y2024_COMMANDS_SCORER_COMMAND_H_
#define y2024_COMMANDS_SCORER_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "subsystems/control_input.h"
#include "subsystems/robot_container.h"
#include "subsystems/scorer.h"
#include "subsystems/super_structure.h"

class ScorerCommand : public frc2::CommandHelper<frc2::Command, ScorerCommand> {
 public:
  ScorerCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  ScorerSubsystem& scorer_;
  SuperStructureSubsystem& super_;

  ControlInputReadings prev_ci_readings_{};
};

#endif  // y2024_COMMANDS_TELESCOPE_COMMAND_H_