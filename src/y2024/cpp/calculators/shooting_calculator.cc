#include "calculators/shooting_calculator.h"

#include "frc846/math/constants.h"
#include "frc846/math/vectors.h"

ShootingCalculatorOutput ShootingCalculator::calculateIteration(
    ShootingCalculatorInput input, ShootingCalculatorOutput prev_output) {
  if (input.shot_distance <= 4.0_ft) {
    return {input.default_setpoint, 0.0_deg};
  }

  /*
   * rv_o = the portion of the robot's velocity is perpendicular to the line
   * between the robot and the speaker.
   *
   * Finding the amount that the robot has to 'twist' to cancel out rv_o
   */
  units::degree_t twist_angle =
      units::math::asin(input.rv_o / input.shot_speed);

  /*
   * Finding the time it takes for the shot to reach the speaker
   *
   * t = time for shot to reach speaker
   * shot_distance = distance to speaker along ground
   *
   * t = shot_distance / ground_speed
   * ground_speed = robot_velocity + shot_speed_along_ground
   */
  auto t = input.shot_distance /
           (input.rv_i + input.shot_speed * units::math::cos(twist_angle) *
                             units::math::cos(prev_output.launch_angle));

  auto height_difference = constants_.speaker_height - input.shooter_height;

  /*
   * Finding the angle the shot has to be launched at to reach the speaker
   */
  auto launch_angle = units::math::asin(
      (height_difference + 0.5 * frc846::math::constants::physics::g * t * t) /
      (input.shot_speed * t));

  return {launch_angle, twist_angle};
}