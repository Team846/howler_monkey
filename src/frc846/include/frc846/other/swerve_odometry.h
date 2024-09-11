#pragma once

#include <array>

#include "frc846/math/collection.h"
#include "frc846/math/vectors.h"
#include "units/base.h"

namespace frc846 {

class SwerveOdometry {
 private:
  static constexpr int kModuleCount = 4;

 public:
  SwerveOdometry(frc846::math::VectorND<units::foot_t, 2> initial_position);

  frc846::math::VectorND<units::foot_t, 2>& position() { return position_; }

  void Update(std::array<frc846::math::VectorND<units::foot_t, 2>, kModuleCount>
                  wheel_vecs,
              units::radian_t omega);

  void SetPoint(frc846::math::VectorND<units::foot_t, 2> point);

  void Zero();

 private:
  frc846::math::VectorND<units::foot_t, 2> position_;
  std::array<units::foot_t, kModuleCount> prev_wheel_distances_ = {};
};

}  // namespace frc846
