#pragma once

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>

#include <cmath>

#include "frc846/math/vectors.h"
#include "frc846/ntinf/pref.h"

namespace frc846::math {

struct FieldPoint {
  VectorND<units::foot_t, 2> point;
  units::degree_t bearing;

  VectorND<units::feet_per_second_t, 2> velocity;

  // Returns a FieldPoint that is 'mirrored' across the field
  FieldPoint mirror(bool shouldMirror = true) const {
    if (shouldMirror) {
      return FieldPoint{{field_size_x - point[0], field_size_y - point[1]},
                        180_deg + bearing,
                        {-velocity[0], -velocity[1]}};
    }
    return FieldPoint{
        {point[0], point[1]}, bearing, {velocity[0], velocity[1]}};
  }

  // Returns a FieldPoint that is 'mirrored' across the centerline of the field
  FieldPoint mirrorOnlyY(bool shouldMirror = true) const {
    if (shouldMirror) {
      return FieldPoint{{point[0], field_size_y - point[1]},
                        180_deg - bearing,
                        {velocity[0], -velocity[1]}};
    }
    return mirror(false);
  }

 private:
  static constexpr units::inch_t field_size_x = 651.25_in;
  static constexpr units::inch_t field_size_y = 315.5_in;
};

class FieldPointPreference {
 public:
  FieldPointPreference(std::string name, FieldPoint backup)
      : x{table_name_, name + "_x", backup.point[0]},
        y{table_name_, name + "_y", backup.point[1]},
        bearing{table_name_, name + "_deg", backup.bearing},
        v_x{table_name_, name + "_v_x", backup.velocity[0]},
        v_y{table_name_, name + "_v_y", backup.velocity[1]} {}

  FieldPoint get() {
    return {
        {x.value(), y.value()}, bearing.value(), {v_x.value(), v_y.value()}};
  }

 private:
  frc846::ntinf::Pref<units::inch_t> x;
  frc846::ntinf::Pref<units::inch_t> y;
  frc846::ntinf::Pref<units::degree_t> bearing;
  frc846::ntinf::Pref<units::feet_per_second_t> v_x;
  frc846::ntinf::Pref<units::feet_per_second_t> v_y;

  const std::string table_name_ = "Preferences/field_points";
};

}  // namespace frc846::math