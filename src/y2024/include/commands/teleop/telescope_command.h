#ifndef y2024_COMMANDS_TELESCOPE_COMMAND_H_
#define y2024_COMMANDS_TELESCOPE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "subsystems/control_input.h"
#include "subsystems/robot_container.h"
#include "subsystems/super_structure.h"
#include "subsystems/telescope.h"

class TelescopeCommand
    : public frc2::CommandHelper<frc2::Command, TelescopeCommand> {
 public:
  TelescopeCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  TelescopeSubsystem& telescope_;
  SuperStructureSubsystem& super_;

  ControlInputReadings prev_ci_readings_{};

  bool telescopeHasRun = false;

  units::inch_t mtele_adj = 0.0_in;
};

#endif  // y2024_COMMANDS_TELESCOPE_COMMAND_H_