#ifndef FRC846_TRAJECTORY_GENERATOR_H_
#define FRC846_TRAJECTORY_GENERATOR_H_

#include <optional>
#include <vector>

#include "frc846/util/math.h"

namespace frc846 {

struct InputWaypoint {
  frc846::util::Position pos;
  std::optional<units::feet_per_second_t> v_max;
};

struct Waypoint {
  frc846::util::Position pos;
  units::feet_per_second_t v;
};

using Trajectory = std::vector<Waypoint>;

std::vector<util::Position> InterpolatePoints(util::Vector2D<units::foot_t> start,
                                        util::Vector2D<units::foot_t> end,
                                        units::degree_t start_bearing,
                                        units::degree_t end_bearing,
                                        units::foot_t cut);

Trajectory GenerateTrajectory(
    std::vector<InputWaypoint> input_points,
    units::feet_per_second_t robot_max_v,
    units::feet_per_second_squared_t robot_max_acceleration,
    units::feet_per_second_squared_t robot_max_deceleration,
    units::inch_t cut = 6_in);

}  // namespace frc846

#endif  // FRC846_TRAJECTORY_GENERATOR_H_