#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <utility>

#include "frc846/loggable.h"
#include "frc846/util/math.h"

struct ShootingAngles {
  units::degree_t launch_angle;
  units::degree_t turning_offset_angle;
};

class ShootingCalculator {
 private:
  static double pow(double base, int exponent) {
    for (int i = 0; i < exponent; i++) {
      base *= base;
    }
    return base;
  }

  static constexpr double pi = 3.14159265;

  static double radians(double degs) { return degs * pi / 180; }

  static double degs(double radians) { return radians * 180 / pi; }

  static constexpr double g = 32.0;

  static constexpr double h_speaker = 78.0 / 12.0;

  double lastAngle = 0.0;

 public:
  ShootingAngles calculateLaunchAngles(double v, double d, double r_v,
                                       double r_o, double h_shooter,
                                       double max_iterations = 7) {
    // if (lastAngle == 0.0) lastAngle = radians(45.0);
    try {
      double t = 0.0;
      for (int i = 0; i < max_iterations; i++) {
        t = d / (r_v + v * std::cos(std::asin(r_o / v) * std::cos(lastAngle)));
        lastAngle =
            std::asin((h_speaker - h_shooter + 1 / 2 * g * t * t) / (v * t));
      }

      if (v * std::sin(lastAngle) / g >= t) {
        return {units::radian_t(lastAngle),
                units::radian_t(std::asin(r_o / v))};
      }

    } catch (std::exception& exc) {
      (void)exc;
    }

    std::cout << "[Shooting calculator] out of range";

    lastAngle = radians(0.0);
    return {0.0_deg, 0.0_deg};
  }
};

struct RawPositions {
  double pivot_angle;
  double wrist_angle;
  double extension;
};

struct CoordinatePositions {
  double shooting_angle;
  double forward_axis;
  double upward_axis;
};

static constexpr double radians(double degs) {
  return degs * 3.141592658979 / 180;
}

static constexpr double degs(double radians) {
  return radians * 180 / 3.141592658979;
}

class InverseKinematics {
 private:
  static constexpr double pivotToWrist = 20.5;
  static constexpr double pivotToWristOffset = -4.25;

  static constexpr double wristToFlywheels = 6.2;

  static constexpr double wristToIntake = 14.5;
  static constexpr double wristToIntakeOtherAngle = radians(36.5);

  static constexpr double pivotToGround = 16.25;

  static constexpr double pivotToCenter = 9.25;

  static constexpr double robotWidth = 28;

  static double pow(double base, int exponent) {
    for (int i = 1; i < exponent; i++) {
      base *= base;
    }
    return base;
  }

 public:
  static bool withinBounds(CoordinatePositions pos) {
    return (pos.forward_axis < robotWidth / 2.0 + 12 &&
            pos.forward_axis > -robotWidth / 2.0 - 12 && pos.upward_axis > 0 &&
            pos.upward_axis < 48);
  }

  static double sumOutOfBounds(CoordinatePositions pos) {
    double sumOut = 0.0;
    if (pos.forward_axis >= robotWidth / 2.0) {
      sumOut += pos.forward_axis - robotWidth / 2.0;
    }
    if (pos.upward_axis >= 48) {
      sumOut += pos.upward_axis - 48;
    }
    return sumOut;
  }

  static RawPositions toRaw(CoordinatePositions pos, int point = 0) {
    RawPositions raw{};

    if (point == 0) {
      double pivotForwardComponent =
          (pos.forward_axis - (wristToIntake)*std::cos(pos.shooting_angle)) +
          pivotToCenter;

      double pivotUpwardComponent =
          (pos.upward_axis - (wristToIntake)*std::sin(pos.shooting_angle)) -
          pivotToGround;

      raw.extension =
          std::sqrt(pow(pivotForwardComponent, 2) +
                    pow(pivotUpwardComponent, 2) - pow(pivotToWristOffset, 2)) -
          pivotToWrist;

      double pivotTotalAngle =
          (std::atan2(pivotUpwardComponent, pivotForwardComponent));

      double pivotAddedAngle =
          (std::atan2(pivotToWristOffset, pivotToWrist + raw.extension));

      raw.pivot_angle = (pivotTotalAngle - pivotAddedAngle + radians(17.0));

      raw.wrist_angle = -(((pos.shooting_angle - raw.pivot_angle +
                            radians(17.0) - radians(47))) -
                          wristToIntakeOtherAngle);

    } else if (point == 1) {
      double pivotForwardComponent =
          (pos.forward_axis - (wristToFlywheels)*std::cos(pos.shooting_angle) +
           pivotToCenter);

      double pivotUpwardComponent =
          (pos.upward_axis - (wristToFlywheels)*std::sin(pos.shooting_angle) -
           pivotToGround);

      raw.extension =
          std::sqrt(pow(pivotForwardComponent, 2) +
                    pow(pivotUpwardComponent, 2) - pow(pivotToWristOffset, 2)) -
          pivotToWrist;

      double pivotTotalAngle =
          (std::atan2(pivotUpwardComponent, pivotForwardComponent));

      double pivotAddedAngle =
          (std::atan2(pivotToWristOffset, pivotToWrist + raw.extension));

      raw.pivot_angle = (pivotTotalAngle - pivotAddedAngle + radians(17.0));

      raw.wrist_angle = -(((pos.shooting_angle - raw.pivot_angle +
                            radians(17.0) - radians(47))));
    }

    return raw;
  }

  static CoordinatePositions toCoordinate(RawPositions pos, int point = 0) {
    CoordinatePositions coordinate{};

    if (point == 0) {
      pos.pivot_angle -= radians(17.0);
      pos.wrist_angle -= radians(49.0);

      double truePivotAngle =
          (pos.pivot_angle) +
          (std::atan2(pivotToWristOffset, pos.extension + pivotToWrist));

      double pivotDistanceHypotenuse = (std::sqrt(
          pow(pivotToWristOffset, 2) + pow(pivotToWrist + pos.extension, 2)));

      coordinate.upward_axis =
          (pivotToGround + pivotDistanceHypotenuse * std::sin(truePivotAngle)) +
          wristToIntake * std::sin(pos.pivot_angle - pos.wrist_angle +
                                   wristToIntakeOtherAngle);

      coordinate.forward_axis =
          pivotDistanceHypotenuse * std::cos(truePivotAngle) - pivotToCenter +
          wristToIntake * std::cos(pos.pivot_angle - pos.wrist_angle +
                                   wristToIntakeOtherAngle);

      coordinate.shooting_angle =
          ((pos.pivot_angle) - pos.wrist_angle + wristToIntakeOtherAngle);
    } else if (point == 1) {
      pos.pivot_angle -= radians(17.0);
      pos.wrist_angle -= radians(49.0);

      double truePivotAngle =
          (pos.pivot_angle) +
          (std::atan2(pivotToWristOffset, pos.extension + pivotToWrist));

      double pivotDistanceHypotenuse = (std::sqrt(
          pow(pivotToWristOffset, 2) + pow(pivotToWrist + pos.extension, 2)));

      coordinate.upward_axis =
          (pivotToGround + pivotDistanceHypotenuse * std::sin(truePivotAngle)) +
          wristToFlywheels * std::sin(pos.pivot_angle - pos.wrist_angle);

      coordinate.forward_axis =
          pivotDistanceHypotenuse * std::cos(truePivotAngle) - pivotToCenter +
          wristToFlywheels * std::cos(pos.pivot_angle - pos.wrist_angle);

      coordinate.shooting_angle = ((pos.pivot_angle) - pos.wrist_angle);
    }

    return coordinate;
  }

  static CoordinatePositions degree_toCoordinate(RawPositions pos,
                                                 int point = 0) {
    return toCoordinate(
        {radians(pos.pivot_angle), radians(pos.wrist_angle), pos.extension},
        point);
  }
};

class TrapCalculator {
 public:
  static std::vector<
      std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>>
  interpolateTrapPoints(
      frc846::util::Vector2D<units::inch_t> starting_coordinate,
      frc846::util::Vector2D<units::inch_t> ending_coordinate,
      units::degree_t starting_angle, units::degree_t ending_angle,
      int steps = 140) {
    std::vector<
        std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>>
        toReturn{};
    for (int i = 0; i < steps; i++) {
      auto x_coord =
          starting_coordinate.x +
          (i * 1.0 / steps) * (ending_coordinate.x - starting_coordinate.x);
      auto y_coord =
          starting_coordinate.y +
          (i * 1.0 / steps) * (ending_coordinate.y - starting_coordinate.y);
      auto angle =
          starting_angle + (i * 1.0 / steps) * (ending_angle - starting_angle);
      toReturn.push_back({{x_coord, y_coord}, angle});

      // std::cout << angle.to<double>() << std::endl;
    }
    return toReturn;
  }

  static RawPositions getRawsAtPoint(
      int counter,
      std::vector<
          std::pair<frc846::util::Vector2D<units::inch_t>, units::degree_t>>
          trapPoints) {
    auto toReturn = InverseKinematics::toRaw(
        CoordinatePositions{radians(trapPoints.at(counter).second.to<double>()),
                            trapPoints.at(counter).first.x.to<double>(),
                            trapPoints.at(counter).first.y.to<double>()});
    toReturn.wrist_angle = radians(trapPoints.at(counter).second.to<double>());
    return toReturn;
  }
};

#endif