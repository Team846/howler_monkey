#include "calculators/arm_kinematics_calculator.h"

#include <units/length.h>

#include "frc846/math/constants.h"
#include "frc846/math/vectors.h"

ArmKinematicsCalculator::ArmKinematicsCalculator() {
  AKCalculatorConstants constants{};

  constants.pivot_offset = {-9.25_in, 16.25_in};
  constants.pivot_to_wrist_joint = {20.5_in, -4.25_in};
  constants.wrist_joint_to_shooter = {0.0_in, 10_in};
  constants.shooter_to_intake = {12.0_in, 0.0_in};

  constants.max_extension_from_center =
      14_in + 1_ft;  // Half robot length + 1ft

  constants.max_height = 4_ft;

  constants_ = constants;

  setConstants(constants);
}

AKCalculatorOutput ArmKinematicsCalculator::calculate(AKCalculatorInput input) {
  frc846::math::Vector2D arm_vector_reference =
      constants_.pivot_to_wrist_joint +
      frc846::math::Vector2D{input.telescope_extension, 0_in};
  auto arm_vector = arm_vector_reference.rotate(input.pivot_angle, false);

  auto joint_position = constants_.pivot_offset + arm_vector;

  auto shooter_position =
      joint_position + constants_.wrist_joint_to_shooter.rotate(
                           input.pivot_angle - input.wrist_angle, false);

  auto intake_position =
      shooter_position + constants_.shooter_to_intake.rotate(
                             input.pivot_angle + input.wrist_angle, false);

  return {shooter_position, intake_position};
}

bool ArmKinematicsCalculator::WithinBounds(
    AKCalculatorOutput end_effector_positions) {
  return end_effector_positions.shooter_position[0] <
             constants_.max_extension_from_center &&
         end_effector_positions.shooter_position[0] >
             -constants_.max_extension_from_center &&
         end_effector_positions.shooter_position[1] < constants_.max_height &&
         end_effector_positions.shooter_position[1] > 0_in &&
         end_effector_positions.intake_position[0] <
             constants_.max_extension_from_center &&
         end_effector_positions.intake_position[0] >
             -constants_.max_extension_from_center &&
         end_effector_positions.intake_position[1] < constants_.max_height &&
         end_effector_positions.intake_position[1] > 0_in;
}