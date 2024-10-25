#pragma once

#include <units/length.h>
#include <units/math.h>

#include "frc846/math/calculator.h"
#include "frc846/math/vectors.h"

struct AKCalculatorInput {
  units::degree_t pivot_angle;
  units::inch_t telescope_extension;
  units::degree_t wrist_angle;
};

struct AKCalculatorOutput {
  frc846::math::Vector2D shooter_position;
  frc846::math::Vector2D intake_position;
};

struct AKCalculatorConstants {
  frc846::math::Vector2D pivot_to_wrist_joint;
  frc846::math::Vector2D wrist_joint_to_shooter;
  frc846::math::Vector2D shooter_to_intake;

  frc846::math::Vector2D pivot_offset;

  units::inch_t max_extension_from_center;
  units::inch_t max_height;
};

class ArmKinematicsCalculator
    : public frc846::math::Calculator<AKCalculatorInput, AKCalculatorOutput,
                                      AKCalculatorConstants> {
 public:
  ArmKinematicsCalculator();

  AKCalculatorOutput calculate(AKCalculatorInput input) override;

  bool WithinBounds(AKCalculatorOutput end_effector_positions);

 private:
  AKCalculatorConstants constants_;
};