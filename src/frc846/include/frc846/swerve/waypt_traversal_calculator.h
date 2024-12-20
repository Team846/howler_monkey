#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

#include "frc846/math/calculator.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/math/vectors.h"

namespace frc846 {
using Waypoint = frc846::math::FieldPoint;
};

namespace frc846::swerve {

struct WTCInput {
  frc846::math::VectorND<units::foot_t, 2> start_pos;
  frc846::math::VectorND<units::foot_t, 2> target_pos;
  frc846::math::VectorND<units::foot_t, 2> current_pos;

  units::degree_t current_bearing;
  units::degree_t target_bearing;

  units::feet_per_second_t current_vel;
  units::feet_per_second_t target_vel;
};

struct WTCOutput {
  bool crossed_waypt;
  units::degrees_per_second_t rotational_vel;
  frc846::math::VectorND<units::feet_per_second_t, 2> target_vel;
};

struct WTCConstants {
  units::feet_per_second_t max_speed;
  units::feet_per_second_squared_t max_acceleration;
  units::feet_per_second_squared_t max_deceleration;

  units::feet_per_second_t true_max_spd;
  units::feet_per_second_squared_t true_max_acc;
  units::feet_per_second_squared_t true_max_dec;

  units::inch_t loc_tolerance;

  units::millisecond_t loop_time;
};

class WayptTraversalCalculator
    : public frc846::math::Calculator<WTCInput, WTCOutput, WTCConstants> {
 public:
  bool HasCrossedWaypt(WTCInput input);

  WTCOutput calculate(WTCInput input) override;

 private:
  units::feet_per_second_t acceleration_to_dc(
      units::feet_per_second_squared_t acc, units::feet_per_second_t vel);

  frc846::math::VectorND<units::foot_t, 2> prev_dir_vec;
};

};  // namespace frc846::swerve