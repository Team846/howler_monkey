#pragma once

#include <array>

#include "frc846/util/math.h"
#include "units/base.h"

namespace frc846 {

class SwerveOdometry {
 private:
  static constexpr int kModuleCount = 4;

 public:
  SwerveOdometry(util::Position initial_pose = {{0_ft, 0_ft}, 0_deg});

  util::Position pose() const { return pose_; }

  void Update(
      std::array<util::Vector2D<units::foot_t>, kModuleCount> wheel_vecs,
      units::radian_t omega);

  void SetPoint(util::Vector2D<units::foot_t> point);

  void Zero();

 private:
  util::Position pose_;
  std::array<units::foot_t, kModuleCount> prev_wheel_distances_ = {};
};

}  // namespace frc846
