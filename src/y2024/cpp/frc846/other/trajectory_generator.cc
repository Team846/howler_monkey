#include "frc846/other/trajectory_generator.h"

#include <algorithm>

#include "frc846/util/math.h"
#include "frc846/loggable.h"

namespace frc846 {


std::vector<util::Position> InterpolatePoints(util::Vector2D<units::foot_t> start,
                                        util::Vector2D<units::foot_t> end,
                                        units::degree_t start_bearing,
                                        units::degree_t end_bearing,
                                        units::foot_t cut) {
  auto distance = (end - start).Magnitude();
  int n = std::max(units::math::ceil(distance / cut).to<int>(), 1);

  std::vector<util::Position> points(n);
  for (int i = 0; i < n; ++i) {
    double weight = (double)(i) / n;
    double bearingWeightMultiplier = 1.15;
    points[i] = {{
                     start.x * (1 - weight) + end.x * weight,
                     start.y * (1 - weight) + end.y * weight,
                 },
                 start_bearing * (1 - (weight * bearingWeightMultiplier)) 
                  + end_bearing * (weight * bearingWeightMultiplier)};

    if (start_bearing <= end_bearing) {
      points[i].bearing = units::math::min(end_bearing, points[i].bearing);
    } else {
      points[i].bearing = units::math::max(end_bearing, points[i].bearing);
    }
  }

  return points;
}

Trajectory GenerateTrajectory(
    std::vector<InputWaypoint> input_points,
    units::feet_per_second_t robot_max_v,
    units::feet_per_second_squared_t robot_max_acceleration,
    units::feet_per_second_squared_t robot_max_deceleration,
    units::inch_t cut) {
  Loggable loggable{"trajectory_generator"};

  if (input_points.size() < 2) {
    loggable.Error("Not enough input points! {} points given",
                input_points.size());
    return {};
  }

  // TODO check that first v > 0

  Trajectory trajectory;

  for (unsigned int i = input_points.size() - 1; i > 0; --i) {
    auto interpolated_points = InterpolatePoints(
        input_points[i].pos.point, input_points[i - 1].pos.point,
        input_points[i].pos.bearing, input_points[i - 1].pos.bearing, cut);
    interpolated_points.erase(interpolated_points.begin());

    trajectory.push_back({
        input_points[i].pos,
        input_points[i].v_max.value_or(robot_max_v),
    });

    for (auto point : interpolated_points) {
      trajectory.push_back({
          point,
          robot_max_v,
      });
    }
  }
  trajectory.push_back({
      input_points[0].pos,
      input_points[0].v_max.value_or(robot_max_v),
  });

  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    auto delta_pos =
        (trajectory[i].pos.point - trajectory[i - 1].pos.point).Magnitude();

    // v₂² = v₁² + 2aΔx
    // 2aΔx = v₂² - v₁²
    // a = (v₂² - v₁²) / (2Δx)
    auto deceleration = (units::math::pow<2>(trajectory[i].v) -
                         units::math::pow<2>(trajectory[i - 1].v)) /
                        (2 * delta_pos);
    if (deceleration > robot_max_deceleration) {
      // v₂² = v₁² + 2aΔx
      // v₂² = sqrt(v₁² + 2aΔx)

      // this code was replaced with calculateW because it makes the incorrect assumption that the values are positive
      trajectory[i].v =
          units::math::sqrt(units::math::pow<2>(trajectory[i - 1].v) +
                            2 * robot_max_deceleration * delta_pos);

    }
  }

  std::reverse(trajectory.begin(), trajectory.end());

  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    auto delta_pos =
        (trajectory[i].pos.point - trajectory[i - 1].pos.point).Magnitude();

    // v₂² = v₁² + 2aΔx
    // 2aΔx = v₂² - v₁²
    // a = (v₂² - v₁²) / (2Δx)
    auto acceleration = (units::math::pow<2>(trajectory[i].v) -
                         units::math::pow<2>(trajectory[i - 1].v)) /
                        (2 * delta_pos);
    if (acceleration > robot_max_acceleration) {
      // v₂² = v₁² + 2aΔx
      // v₂² = sqrt(v₁² + 2aΔx)
      trajectory[i].v =
          units::math::sqrt(units::math::pow<2>(trajectory[i - 1].v) +
                            2 * robot_max_acceleration * delta_pos);

      //trajectory[i].v = units::velocity::feet_per_second_t(Find_Vsub2(units::unit_cast<double>(trajectory[i].v), units::unit_cast<double>(robot_max_acceleration), units::unit_cast<double>(delta_pos)));
    }
  }

  // If any point has 0 speed, just set it to the previous speed.
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    if (trajectory[i].v == 0_fps) {
      trajectory[i].v = i == 0 ? trajectory[1].v : trajectory[i - 1].v;
    }
  }

  return trajectory;
}

}  // namespace frc846