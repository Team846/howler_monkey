#include "frc846/other/swerve_odometry.h"

#include <cmath>
#include <cstdio>

#include "frc846/math/vectors.h"

namespace frc846 {

SwerveOdometry::SwerveOdometry(
    frc846::math::VectorND<units::foot_t, 2> initial_position)
    : position_{initial_position[0], initial_position[1]} {}

void SwerveOdometry::Update(
    std::array<frc846::math::VectorND<units::foot_t, 2>, kModuleCount>
        wheel_vecs,
    units::radian_t bearing) {
  // change in distance from the last odometry update
  for (int i = 0; i < kModuleCount; i++) {
    units::foot_t wheel_dist = wheel_vecs[i].magnitude();
    units::foot_t delta_distance = wheel_dist - prev_wheel_distances_[i];
    units::foot_t dx =
        delta_distance * units::math::sin(wheel_vecs[i].angle(true) + bearing);
    units::foot_t dy =
        delta_distance * units::math::cos(wheel_vecs[i].angle(true) + bearing);

    prev_wheel_distances_[i] = wheel_dist;

    wheel_vecs[i][0] = dx;
    wheel_vecs[i][1] = dy;
  }

  frc846::math::VectorND<units::foot_t, 2> relative_displacement{0_ft, 0_ft};

  for (int i = 0; i < kModuleCount; i++) {
    relative_displacement += wheel_vecs[i] / kModuleCount;
  }

  position_ += relative_displacement.rotate(bearing, true);
}

void SwerveOdometry::SetPoint(frc846::math::VectorND<units::foot_t, 2> point) {
  position_[0] = point[0];
  position_[1] = point[1];
}

void SwerveOdometry::Zero() { SetPoint({0_ft, 0_ft}); }

}  // namespace frc846