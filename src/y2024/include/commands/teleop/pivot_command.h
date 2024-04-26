#ifndef y2024_COMMANDS_PIVOT_COMMAND_H_
#define y2024_COMMANDS_PIVOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "subsystems/control_input.h"
#include "subsystems/pivot.h"
#include "subsystems/robot_container.h"
#include "subsystems/super_structure.h"

class PivotCommand : public frc2::CommandHelper<frc2::Command, PivotCommand> {
 public:
  PivotCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  PivotSubsystem& pivot_;
  SuperStructureSubsystem& super_;

  ControlInputReadings prev_ci_readings_{};

  bool pivotHasRun = false;

  units::degree_t mpiv_adj = 0.0_deg;
};

#endif  // y2024_COMMANDS_PIVOT_COMMAND_H_