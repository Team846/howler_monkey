#include "commands/teleop/operator_control.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/swerve_module.h"

OperatorControlCommand::OperatorControlCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      super_(container.super_structure_) {
  AddRequirements({&control_input_});
  SetName("operator_control_command");
}

void OperatorControlCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  PTWSetpoint manual_adjustment_additions{};

  if (std::abs(ci_readings_.wrist_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    manual_adjustment_additions.wrist =
        super_.wrist_->max_adjustment_rate_.value() / 50.0_Hz *
        ci_readings_.wrist_manual_adjust;
  }

  if (std::abs(ci_readings_.telescope_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    manual_adjustment_additions.telescope =
        super_.telescope_->max_adjustment_rate_.value() / 50.0_Hz *
        ci_readings_.telescope_manual_adjust;
  }

  if (std::abs(ci_readings_.pivot_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    manual_adjustment_additions.pivot =
        super_.pivot_->max_adjustment_rate_.value() / 50.0_Hz *
        ci_readings_.pivot_manual_adjust;
  }

  super_.AdjustSetpoint(manual_adjustment_additions);
}

bool OperatorControlCommand::IsFinished() { return false; }