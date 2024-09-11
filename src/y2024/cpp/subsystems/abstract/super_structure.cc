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

  if (homing_wrist) {
    wrist_->SetTarget({wrist_->homing_speed_.value(), true});
    if (wrist_->GetReadings().wrist_velocity <=
        wrist_->homing_velocity_tolerance_.value()) {
      wrist_home_counter++;
    } else {
      wrist_home_counter = 0;
    }
    if (wrist_home_counter >= 30) {
      wrist_->ZeroSubsystem();
      homing_wrist = false;
      wrist_home_counter = 0;
    }
  } else {
    wrist_->SetTarget({targetPos.wrist});
    wrist_home_counter = 0;
  }

  auto target_end_effector_positions = arm_kinematics_calculator.calculate(
      {targetPos.pivot, targetPos.telescope, targetPos.wrist});

  intake_point_x_graph_.Graph(target_end_effector_positions.intake_position[0]);
  intake_point_y_graph_.Graph(target_end_effector_positions.intake_position[1]);

  shooter_point_x_graph_.Graph(
      target_end_effector_positions.shooter_position[0]);
  shooter_point_y_graph_.Graph(
      target_end_effector_positions.shooter_position[1]);
}
