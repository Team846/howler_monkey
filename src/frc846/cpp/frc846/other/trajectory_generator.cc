#include "frc846/other/trajectory_generator.h"

#include <algorithm>

#include "frc846/base/loggable.h"

namespace frc846 {

std::vector<frc846::math::FieldPoint> InterpolatePoints(
    frc846::math::VectorND<units::foot_t, 2> start,
    frc846::math::VectorND<units::foot_t, 2> end, units::degree_t start_bearing,
    units::degree_t end_bearing, units::foot_t cut) {
  auto distance = (end - start).magnitude();
  int n = std::max(units::math::ceil(distance / cut).to<int>(), 1);

  std::vector<frc846::math::FieldPoint> points(n);
  for (int i = 0; i < n; ++i) {
    double weight = (double)(i) / n;
    double bearingWeightMultiplier = 1.15;
    points[i] = {{
                     start[0] * (1 - weight) + end[0] * weight,
                     start[1] * (1 - weight) + end[1] * weight,
                 },
                 start_bearing * (1 - (weight * bearingWeightMultiplier)) +
                     end_bearing * (weight * bearingWeightMultiplier),
                 {0.0_fps, 0.0_fps}};

    if (start_bearing <= end_bearing) {
      points[i].bearing = units::math::min(end_bearing, points[i].bearing);
    } else {
      points[i].bearing = units::math::max(end_bearing, points[i].bearing);
    }
  }

  return points;
}

Trajectory GenerateTrajectory(
    std::vector<Waypoint> input_points, units::feet_per_second_t robot_max_v,
    units::feet_per_second_squared_t robot_max_acceleration,
    units::feet_per_second_squared_t robot_max_deceleration,
    units::inch_t cut) {
  frc846::base::Loggable loggable{"trajectory_generator"};

  if (input_points.size() < 2) {
    loggable.Error("Not enough input points! {} points given",
                   input_points.size());
    return {};
  }

  // TODO check that first v > 0

  Trajectory trajectory;

  for (unsigned int i = input_points.size() - 1; i > 0; --i) {
    auto interpolated_points = InterpolatePoints(
        input_points[i].point, input_points[i - 1].point,
        input_points[i].bearing, input_points[i - 1].bearing, cut);
    interpolated_points.erase(interpolated_points.begin());

    trajectory.push_back({input_points[i]});

    for (auto point : interpolated_points) {
      point.velocity[0] = robot_max_v;
      trajectory.push_back({point});
    }
  }
  trajectory.push_back({input_points[0]});

  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    auto delta_pos =
        (trajectory[i].point - trajectory[i - 1].point).magnitude();

    // v₂² = v₁² + 2aΔx
    // 2aΔx = v₂² - v₁²
    // a = (v₂² - v₁²) / (2Δx)
    auto deceleration =
        (units::math::pow<2>(trajectory[i].velocity.magnitude()) -
         units::math::pow<2>(trajectory[i - 1].velocity.magnitude())) /
        (2 * delta_pos);
    if (deceleration > robot_max_deceleration) {
      // v₂² = v₁² + 2aΔx
      // v₂² = sqrt(v₁² + 2aΔx)

      trajectory[i].velocity[0] = units::math::sqrt(
          units::math::pow<2>(trajectory[i - 1].velocity.magnitude()) +
          2 * robot_max_deceleration * delta_pos);
    }
  }

  std::reverse(trajectory.begin(), trajectory.end());

  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    auto delta_pos =
        (trajectory[i].point - trajectory[i - 1].point).magnitude();

    // v₂² = v₁² + 2aΔx
    // 2aΔx = v₂² - v₁²
    // a = (v₂² - v₁²) / (2Δx)
    auto acceleration =
        (units::math::pow<2>(trajectory[i].velocity.magnitude()) -
         units::math::pow<2>(trajectory[i - 1].velocity.magnitude())) /
        (2 * delta_pos);
    if (acceleration > robot_max_acceleration) {
      // v₂² = v₁² + 2aΔx
      // v₂² = sqrt(v₁² + 2aΔx)
      trajectory[i].velocity[0] = units::math::sqrt(
          units::math::pow<2>(trajectory[i - 1].velocity.magnitude()) +
          2 * robot_max_acceleration * delta_pos);

      // trajectory[i].v =
      // units::velocity::feet_per_second_t(Find_Vsub2(units::unit_cast<double>(trajectory[i].v),
      // units::unit_cast<double>(robot_max_acceleration),
      // units::unit_cast<double>(delta_pos)));
    }
  }

  // If any point has 0 speed, just set it to the previous speed.
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    if (trajectory[i].velocity.magnitude() == 0_fps) {
      trajectory[i].velocity.magnitude() =
          i == 0 ? trajectory[1].velocity.magnitude()
                 : trajectory[i - 1].velocity.magnitude();
    }
  }

  return trajectory;
}

}  // namespace frc846