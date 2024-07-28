#pragma once

#include <utility>

#include "frc846/base/loggable.h"
#include "frc846/util/math.h"

using namespace frc846::util;

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
                                       double angle_setpoint = 55.0,
                                       double max_iterations = 7) {
    if (d <= 4.0) {
      return {1_deg * angle_setpoint, 0.0_deg};
    }

    try {
      double t = 0.0;
      for (int i = 0; i < max_iterations; i++) {
        t = d / (r_v + v * std::cos(std::asin(r_o / v)) * std::cos(lastAngle));
        lastAngle = std::asin((h_speaker - h_shooter + 1.0 / 2.0 * g * t * t) /
                              (v * t));
      }

      if (v * std::sin(lastAngle) / g >= t) {
        return {units::radian_t(lastAngle),
                units::radian_t(std::asin(r_o / v))};
      }
    } catch (std::exception& exc) {
      (void)exc;
    }

    // std::cout << "[Shooting calculator] out of range";

    lastAngle = radians(0.0);
    return {0.0_deg, 0.0_deg};
  }
};

struct RawPositions {
  units::degree_t pivot_angle;
  units::inch_t extension;
  units::degree_t wrist_angle;
};

struct CoordinatePositions {
  units::degree_t shooting_angle;
  units::inch_t forward_axis;
  units::inch_t upward_axis;
};

static constexpr double radians(double degs) {
  return degs * 3.141592658979 / 180;
}

static constexpr double degs(double radians) {
  return radians * 180 / 3.141592658979;
}

// FIX adding/removing home_offsets
class ArmKinematics {
 private:
  static constexpr units::inch_t pivotToWrist = 20.5_in;
  static constexpr units::inch_t pivotToWristOffset = -4.25_in;

  static constexpr units::inch_t wristToShooter = 10_in;
  static constexpr units::inch_t shooterToIntake = 12.0_in;

  static constexpr double wristToIntakeOtherAngle = radians(36.5);

  static constexpr units::inch_t pivotToGround = 16.25_in;

  static constexpr units::inch_t pivotToCenter = -9.25_in;

  static constexpr units::inch_t robotWidth = 28_in;

  static double pow(double base, int exponent) {
    for (int i = 1; i < exponent; i++) {
      base *= base;
    }
    return base;
  }

 public:
  /* default position calculated is for the center of the shooter*/
  static CoordinatePositions calculateCoordinatePosition(RawPositions pos_raw,
                                                         bool intakePoint);

  static CoordinatePositions calculateRawPosition(RawPositions pos_raw,
                                                  bool intakePoint);

  static bool withinBounds(CoordinatePositions pos) {
    return (pos.forward_axis < robotWidth / 2.0 + 12.0_in &&
            pos.upward_axis < 47.5_in);
  }

  static double sumOutOfBounds(CoordinatePositions pos) {
    units::inch_t sumOut = 0.0_in;
    if (pos.forward_axis >= (robotWidth / 2.0 + 12.0_in)) {
      sumOut += pos.forward_axis - (robotWidth / 2.0 + 12.0_in);
    }
    if (pos.upward_axis >= 48.0_in) {
      sumOut += pos.upward_axis - 48.0_in;
    }
    return sumOut.to<double>();
  }

  /*static RawPositions toRaw(CoordinatePositions pos, int point = 0) {
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
  }*/
};
