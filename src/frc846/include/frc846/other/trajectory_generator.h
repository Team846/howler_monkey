#pragma once

#include <optional>
#include <vector>

#include "frc846/math/fieldpoints.h"

namespace frc846 {

struct Waypoint {
  frc846::math::FieldPoint pos;
};

using Trajectory = std::vector<Waypoint>;

std::vector<frc846::math::FieldPoint> InterpolatePoints(
    frc846::math::VectorND<units::foot_t, 2> start,
    frc846::math::VectorND<units::foot_t, 2> end, units::degree_t start_bearing,
    units::degree_t end_bearing, units::foot_t cut);

Trajectory GenerateTrajectory(
    std::vector<Waypoint> input_points, units::feet_per_second_t robot_max_v,
    units::feet_per_second_squared_t robot_max_acceleration,
    units::feet_per_second_squared_t robot_max_deceleration,
    units::inch_t cut = 2_in);

}  // namespace frc846
