#include "frc846/swerve/waypt_traversal_calculator.h"

namespace frc846::swerve {

bool WayptTraversalCalculator::HasCrossedWaypt(WTCInput input) {
  auto full_disp_vec = (input.target_pos - input.start_pos);

  auto current_disp_vec = (input.current_pos - input.start_pos);

  if (full_disp_vec.magnitude() <= constants_.loc_tolerance) {
    return true;
  }

  return current_disp_vec.magnitude() >= full_disp_vec.magnitude();
};

WTCOutput WayptTraversalCalculator::calculate(WTCInput input) {
  WTCOutput result{};
  result.target_vel = {0.0_fps, 0.0_fps};
  result.rotational_vel = units::degrees_per_second_t{0.0};
  result.crossed_waypt = false;

  auto delta_vec = (input.target_pos - input.current_pos);

  if (delta_vec.magnitude() < constants_.loc_tolerance) {
    result.crossed_waypt = true;

    return result;
  }
  result.crossed_waypt = HasCrossedWaypt(input);

  if (result.crossed_waypt) return result;

  auto dir_vec = delta_vec.unit();

  if (delta_vec.magnitude() < 2 * constants_.loc_tolerance) {
    dir_vec = prev_dir_vec;
  }

  prev_dir_vec = dir_vec;

  units::second_t reach_time_min =
      delta_vec.magnitude() /
      units::math::abs(input.current_vel - input.target_vel);
  units::feet_per_second_squared_t required_deceleration =
      units::math::abs(input.current_vel - input.target_vel) / reach_time_min;

  result.rotational_vel =
      (input.target_bearing - input.current_bearing) / reach_time_min;

  units::feet_per_second_squared_t target_acc{0.0};

  if (required_deceleration >= constants_.max_deceleration) {
    target_acc = -required_deceleration;
  } else if (input.current_vel < constants_.max_speed) {
    target_acc = constants_.max_acceleration;
  }

  units::feet_per_second_t target_dc =
      acceleration_to_dc(target_acc, input.current_vel);

  result.target_vel = {dir_vec[0].to<double>() * target_dc,
                       dir_vec[1].to<double>() * target_dc};

  return result;
}

units::feet_per_second_t WayptTraversalCalculator::acceleration_to_dc(
    units::feet_per_second_squared_t acc, units::feet_per_second_t vel) {
  double current_vel_pct = vel / constants_.true_max_spd;

  double acc_pct = acc / ((acc >= units::feet_per_second_squared_t(0.0))
                              ? constants_.true_max_acc
                              : constants_.true_max_dec);

  double dc = current_vel_pct + acc_pct;

  if (dc <= 0.0) dc = 0.0;

  return units::math::min(constants_.true_max_spd,
                          dc * constants_.true_max_spd);
}

};  // namespace frc846::swerve