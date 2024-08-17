#include "commands/teleop/operator_control.h"

OperatorControlCommand::OperatorControlCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, OperatorControlCommand>{
          container, "operator_control_command"} {
  AddRequirements({&container_.control_input_});
}

void OperatorControlCommand::OnInit() {}

void OperatorControlCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  PTWSetpoint manual_adjustment_additions{};

  if (std::abs(ci_readings_.wrist_manual_adjust) >
      container_.super_structure_.manual_control_deadband_.value()) {
    manual_adjustment_additions.wrist =
        container_.super_structure_.wrist_->max_adjustment_rate_.value() /
        50.0_Hz * ci_readings_.wrist_manual_adjust;
  }

  if (std::abs(ci_readings_.telescope_manual_adjust) >
      container_.super_structure_.manual_control_deadband_.value()) {
    manual_adjustment_additions.telescope =
        container_.super_structure_.telescope_->max_adjustment_rate_.value() /
        50.0_Hz * ci_readings_.telescope_manual_adjust;
  }

  if (std::abs(ci_readings_.pivot_manual_adjust) >
      container_.super_structure_.manual_control_deadband_.value()) {
    manual_adjustment_additions.pivot =
        container_.super_structure_.pivot_->max_adjustment_rate_.value() /
        50.0_Hz * ci_readings_.pivot_manual_adjust;
  }

  container_.super_structure_.AdjustSetpoint(manual_adjustment_additions);
}

void OperatorControlCommand::OnEnd(bool interrupted) {}

bool OperatorControlCommand::IsFinished() { return false; }