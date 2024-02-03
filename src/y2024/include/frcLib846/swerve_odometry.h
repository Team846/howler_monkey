#ifndef frcLib846_SWERVE_ODOMETRY_H_
#define frcLib846_SWERVE_ODOMETRY_H_

#include <array>

#include "frcLib846/math.h"
#include "units/base.h"

namespace frcLib846 {

class SwerveOdometry {
 private:
  static constexpr int kModuleCount = 4;

 public:
  SwerveOdometry(Position initial_pose = {{0_ft, 0_ft}, 0_deg});

  Position pose() const { return pose_; }

  void Update(std::array<Vector2D<units::foot_t>, kModuleCount> wheel_vecs,
              units::radian_t omega);

  void SetPoint(Vector2D<units::foot_t> point);

  void Zero();

 private:
  Position pose_;
  std::array<units::foot_t, kModuleCount> prev_wheel_distances_ = {};
};

}  // namespace frcLib846

#endif  // frcLib846_SWERVE_ODOMETRY_H_