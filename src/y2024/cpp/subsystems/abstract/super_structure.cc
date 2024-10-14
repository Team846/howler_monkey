#include "subsystems/abstract/super_structure.h"

#include "frc846/util/share_tables.h"

void SuperStructureSubsystem::Setup() {};

SuperStructureTarget SuperStructureSubsystem::ZeroTarget() const {
  SuperStructureTarget target;
  return target;
}

bool SuperStructureSubsystem::VerifyHardware() { return true; }

SuperStructureReadings SuperStructureSubsystem::ReadFromHardware() {
  return {};
}

void SuperStructureSubsystem::WriteToHardware(SuperStructureTarget target) {
  PTWSetpoint targetPos = currentSetpoint + manualAdjustments;

  pivot_->SetTarget({targetPos.pivot});
  telescope_->SetTarget({targetPos.telescope});

  wrist_->SetTarget({targetPos.wrist});

  auto target_end_effector_positions = arm_kinematics_calculator.calculate(
      {targetPos.pivot, targetPos.telescope, targetPos.wrist});

  intake_point_x_graph_.Graph(target_end_effector_positions.intake_position[0]);
  intake_point_y_graph_.Graph(target_end_effector_positions.intake_position[1]);

  shooter_point_x_graph_.Graph(
      target_end_effector_positions.shooter_position[0]);
  shooter_point_y_graph_.Graph(
      target_end_effector_positions.shooter_position[1]);
}
