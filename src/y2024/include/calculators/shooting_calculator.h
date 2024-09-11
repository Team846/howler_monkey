#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

#include "frc846/math/calculator.h"

struct ShootingCalculatorInput {
  units::foot_t shot_distance;
  units::feet_per_second_t rv_i;
  units::feet_per_second_t rv_o;

  units::foot_t shooter_height;

  units::degree_t
      default_setpoint;  // Not in constants so that these can be changed live
                         // (without restarting robot code)
  units::feet_per_second_t shot_speed;
};

struct ShootingCalculatorOutput {
  units::degree_t launch_angle;
  units::degree_t twist_angle;
};

struct ShootingCalculatorConstants {
  units::foot_t speaker_height;
};

class ShootingCalculator
    : public frc846::math::IterativeCalculator<ShootingCalculatorInput,
                                               ShootingCalculatorOutput,
                                               ShootingCalculatorConstants> {
 public:
  ShootingCalculatorOutput calculateIteration(
      ShootingCalculatorInput input,
      ShootingCalculatorOutput prev_output) override;
};