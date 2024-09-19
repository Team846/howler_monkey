#include "frc846/swerve/waypt_traversal_calculator.h"

namespace frc846::swerve {

bool WayptTraversalCalculator::HasCrossedWaypt(WTCInput input) {
  auto disp_vec = (input.current_pos - input.start_pos);

  auto target_vec = (input.target_pos - input.start_pos);

  if (target_vec.magnitude() <= constants_.loc_tolerance) {
    return true;
  }

  return disp_vec.projectOntoAnother(target_vec).magnitude() >=
         target_vec.magnitude();
};

WTCOutput WayptTraversalCalculator::calculate(WTCInput input) {
  WTCOutput result{};
  result.target_vel = {0.0_fps, 0.0_fps};
  result.bearing = 0_deg;

  auto delta_vec = (input.target_pos - input.current_pos);

  if (delta_vec.magnitude() < constants_.loc_tolerance) {
    result.crossed_waypt = true;

    return result;
  }
  result.crossed_waypt = HasCrossedWaypt(input);

  if (result.crossed_waypt) return result;

  auto dir_vec = delta_vec.unit();

  units::second_t stopping_time =
      units::math::abs(input.current_vel - input.target_vel) /
      constants_.max_acceleration;
  units::foot_t stopping_distance =
      (input.current_vel + input.target_vel) / 2 * stopping_time;

  result.bearing = (input.target_bearing - input.current_bearing) *
                   constants_.loop_time / stopping_time;

  auto target_vel_mag = constants_.max_speed;

  if (stopping_distance > delta_vec.magnitude() - constants_.loc_tolerance) {
    target_vel_mag =
        input.current_vel + (input.target_vel - input.current_vel) *
                                constants_.loop_time / stopping_time;

  } else if (input.current_vel < constants_.max_speed) {
    target_vel_mag =
        input.current_vel + constants_.max_acceleration * constants_.loop_time;
  }
  target_vel_mag = units::math::min(target_vel_mag, constants_.max_speed);

  result.target_vel = {dir_vec[0].to<double>() * target_vel_mag,
                       dir_vec[1].to<double>() * target_vel_mag};

  return result;
}

};  // namespace frc846::swerve