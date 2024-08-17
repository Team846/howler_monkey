#pragma once

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>

#include <cmath>

#include "pref.h"

namespace frc846::util {

// Find the circumference of a circle given radius.
constexpr units::inch_t Circumference(units::meter_t radius) {
  return 2 * units::constants::pi * radius;
}

double HorizontalDeadband(double input, double x_intercept, double max,
                          double exponent = 1, double sensitivity = 1);

double VerticalDeadband(double input, double y_intercept, double max,
                        double exponent = 1, double sensitivity = 1);

// Returns the smallest difference between an element of C(θ₁) and an element of
// C(θ₂).
units::degree_t CoterminalDifference(units::degree_t angle,
                                     units::degree_t other_angle);

// Returns the smallest sum between two angles.
units::degree_t CoterminalSum(units::degree_t angle,
                              units::degree_t other_angle);

// A 2D vector.
//
// TODO only calculate magnitude and bearing once
template <class T>
struct Vector2D {
  static_assert(units::traits::is_unit_t<T>(), "must be a unit");

  T x;
  T y;

  // Magnitude of the vector.
  T Magnitude() const { return units::math::sqrt(x * x + y * y); }

  // Bearing of the vector.
  //
  // x and y are intentionally swapped in atan2.
  units::radian_t Bearing() const { return units::math::atan2(x, y); }

  // Returns a new vector rotate CLOCKWISE by theta.
  Vector2D<T> Rotate(units::degree_t theta) const {
    return {
        x * units::math::cos(theta) + y * units::math::sin(theta),
        x * -units::math::sin(theta) + y * units::math::cos(theta),
    };
  }

  Vector2D<T> Extrapolate(Vector2D<T> from, units::inch_t by) const {
    auto bearing = units::math::atan2(x - from.x, y - from.y);
    return {x + by * units::math::sin(bearing),
            y + by * units::math::cos(bearing)};
  }

  Vector2D<T> operator+(const Vector2D& other) const {
    return {x + other.x, y + other.y};
  }

  Vector2D<T> operator-(const Vector2D& other) const {
    return {x - other.x, y - other.y};
  }

  Vector2D<T> operator*(double scalar) const {
    return {x * scalar, y * scalar};
  }

  Vector2D<T> operator/(double scalar) const {
    return {x / scalar, y / scalar};
  }

  bool operator==(const Vector2D& other) const {
    return x == other.x && y == other.y;
  }

  // VectorPolar toVectorPolar() { return {Magnitude(), Bearing()}; }
};

// template <class T>
// struct VectorPolar {
//   static_assert(units::traits::is_unit_t<T>(), "must be a unit");

//   T magnitude;            // Length of vector
//   units::degree_t theta;  // Angle from vertical

//   Vector2D<T> toVector2D() {
//     return {
//       magnitude *units::math::sin(theta), magnitude *units::math::cos(theta);
//     }
//   }

//   VectorPolar operator+(const VectorPolar& other) const {
//     return (toVector2D() + other.toVector2D()).toVectorPolar();
//   }
// };

struct Position {
  Vector2D<units::foot_t> point;
  units::degree_t bearing;
};

class FieldPoint {
 public:
  static Position to_position(FieldPoint point) {
    return Position{{point.x.value(), point.y.value()}, point.bearing.value()};
  }

  static Position flipOnlyY(FieldPoint point, bool should_flip) {
    if (should_flip) {
      return {{point.x.value(), field_size_y - point.y.value()},
              180_deg - point.bearing.value()};
    }
    return {{point.x.value(), point.y.value()}, point.bearing.value()};
  }

  static Position flipOnlyY(Position point, bool should_flip) {
    if (should_flip) {
      return {{point.point.x, field_size_y - point.point.y},
              180_deg - point.bearing};
    }
    return point;
  }

  static Position flip(FieldPoint point, bool should_flip, bool onlyY = false) {
    if (onlyY) return flipOnlyY(point, should_flip);

    if (should_flip) {
      return Position{{-point.x.value(), field_size_y - point.y.value()},
                      180_deg + point.bearing.value()};
    } else {
      return Position{{point.x.value(), point.y.value()},
                      point.bearing.value()};
    }
  }

  static Position flip(Position point, bool should_flip, bool onlyY = false) {
    if (onlyY) return flipOnlyY(point, should_flip);
    if (should_flip) {
      return Position{
          {field_size_x - point.point.x, field_size_y - point.point.y},
          180_deg + point.bearing};
    } else {
      return Position{{point.point.x, point.point.y}, point.bearing};
    }
  }

  Position flip(bool should_flip, bool onlyY = false) {
    return flip(*this, should_flip, onlyY);
  }

  frc846::base::Loggable& point_names_ =
      *(new frc846::base::Loggable("Preferences/field_points"));

  FieldPoint(std::string name)
      : x{point_names_, name + "_x", 0.0_in},
        y{point_names_, name + "_y", 0.0_in},
        bearing{point_names_, name + "_deg", 0.0_deg},
        name_(name) {}

  FieldPoint(std::string name, units::inch_t x, units::inch_t y,
             units::degree_t deg)
      : x{point_names_, name + "_x", x},
        y{point_names_, name + "_y", y},
        bearing{point_names_, name + "_deg", deg},
        name_(name) {}

  Pref<units::inch_t> x;
  Pref<units::inch_t> y;
  Pref<units::degree_t> bearing;
  std::string name_;

 private:
  static constexpr units::inch_t field_size_x = 651.25_in;
  static constexpr units::inch_t field_size_y = 315.5_in;
};

}  // namespace frc846::util
