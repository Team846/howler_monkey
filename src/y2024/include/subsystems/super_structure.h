#ifndef Y2024_SUBSYSTEMS_SUPER_STRUCTURE_H_
#define Y2024_SUBSYSTEMS_SUPER_STRUCTURE_H_

#include "frc846/util/share_tables.h"
#include "frc846/subsystem.h"
#include "frc846/util/pref.h"
#include "units/length.h"
#include "units/time.h"
#include "units/angle.h"


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
    return (pos.forward_axis < robotWidth / 2.0 + 10.5 && pos.forward_axis > -robotWidth / 2.0 - 10.5
      &&  pos.upward_axis > -10.0 && pos.upward_axis < 45);
  }

  static RawPositions toRaw(CoordinatePositions pos) {
    // if (!withinBounds(pos)) {
    //   double angle = atan2(pos.upward_axis, pos.forward_axis);

    //   double maxMagnitude = std::abs(std::min(robotWidth / 2.0 + 11.5, -robotWidth / 2.0 - 11.5) / cos(angle));
      

    //   double magnitude = std::min(std::max(0.0, std::min(std::max(0.0, std::hypot(pos.forward_axis, pos.upward_axis)), maxMagnitude)), 47.5);

    //   pos.forward_axis = magnitude * cos(angle);
    //   pos.upward_axis = magnitude * sin(angle);
    // }

    RawPositions raw{};

    // double pivotForwardComponent = (pos.forward_axis - (wristToIntake)*std::cos(pos.shooting_angle)) 
    //   + pivotToCenter;

    // double pivotUpwardComponent = (pos.upward_axis - (wristToIntake)*std::sin(pos.shooting_angle)) - pivotToGround;

    double pivotForwardComponent = (pos.forward_axis + pivotToCenter);

    double pivotUpwardComponent = (pos.upward_axis - pivotToGround);
    
    raw.extension = std::sqrt(pow(pivotForwardComponent, 2) + pow(pivotUpwardComponent, 2) - pow(pivotToWristOffset, 2)) 
        - pivotToWrist;

    // double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
    //   + pow(pivotToWrist + raw.extension, 2)));


    double pivotTotalAngle = (std::atan2(pivotUpwardComponent, pivotForwardComponent));

    double pivotAddedAngle = (std::atan2(pivotToWristOffset, pivotToWrist + raw.extension));

    raw.pivot_angle =  (pivotTotalAngle - pivotAddedAngle + radians(17.0));

    raw.wrist_angle = -(((pos.shooting_angle - raw.pivot_angle + radians(17.0) - radians(47))) - wristToIntakeOtherAngle);

    return raw;
  }

  static CoordinatePositions toCoordinate(RawPositions pos) {
    CoordinatePositions coordinate{};

    pos.pivot_angle -= radians(17.0);
    pos.wrist_angle -= radians(49.0);
  
    double truePivotAngle = (pos.pivot_angle) + (std::atan2(pivotToWristOffset,
          pos.extension+pivotToWrist));


    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + pos.extension, 2)));

    coordinate.upward_axis = (pivotToGround + 
       pivotDistanceHypotenuse * std::sin(truePivotAngle)) + 
        wristToIntake * std::sin(pos.pivot_angle - pos.wrist_angle + wristToIntakeOtherAngle);

    coordinate.forward_axis = pivotDistanceHypotenuse * std::cos(truePivotAngle) -
      pivotToCenter + wristToIntake * std::cos(pos.pivot_angle - pos.wrist_angle + wristToIntakeOtherAngle);

    coordinate.shooting_angle = ((pos.pivot_angle) - pos.wrist_angle + wristToIntakeOtherAngle); 

    return coordinate;
  }

  static CoordinatePositions degree_toCoordinate(RawPositions pos) {
    return toCoordinate({radians(pos.pivot_angle), radians(pos.wrist_angle), pos.extension});
  }

    static CoordinatePositions toCoordinatePoint2(RawPositions pos) {
    CoordinatePositions coordinate{};

    pos.pivot_angle -= radians(17.0);
    pos.wrist_angle -= radians(49.0);
  
    double truePivotAngle = (pos.pivot_angle) + (std::atan2(pivotToWristOffset,
          pos.extension+pivotToWrist));


    double pivotDistanceHypotenuse = (std::sqrt(pow(pivotToWristOffset, 2) 
      + pow(pivotToWrist + pos.extension, 2)));

    coordinate.upward_axis = (pivotToGround + 
       pivotDistanceHypotenuse * std::sin(truePivotAngle)) + 
        wristToFlywheels * std::sin(pos.pivot_angle - pos.wrist_angle);

    coordinate.forward_axis = pivotDistanceHypotenuse * std::cos(truePivotAngle) -
      pivotToCenter + wristToFlywheels * std::cos(pos.pivot_angle - pos.wrist_angle);

    coordinate.shooting_angle = ((pos.pivot_angle) - pos.wrist_angle); 

    return coordinate;
  }

  static CoordinatePositions degree_toCoordinatePoint2(RawPositions pos) {
    return toCoordinatePoint2({radians(pos.pivot_angle), radians(pos.wrist_angle), pos.extension});
  }
};


struct SuperStructureReadings {};
struct SuperStructureTarget {};

class SuperStructureSubsystem : public frc846::Subsystem<SuperStructureReadings, SuperStructureTarget> {
 public:
  SuperStructureSubsystem(bool init);

  SuperStructureTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroSubsystem() {
    hasZeroed = true;
  }

  bool GetHasZeroed() { return hasZeroed; }

  frc846::Loggable auto_named_{*this, "auto"};
  frc846::Pref<units::second_t> post_shoot_wait_{auto_named_, "post_shoot_wait", 1_s};
  frc846::Pref<units::second_t> pre_shoot_wait_{auto_named_, "pre_shoot_wait", 1_s};

  frc846::Loggable shooting_named_{*this, "shooting"};
  frc846::Pref<units::inch_t> teleop_shooter_height_{shooting_named_, "teleop_shooter_height", 39_in};
  frc846::Pref<units::inch_t> teleop_shooter_x_{shooting_named_, "teleop_shooter_x", 4_in};
  frc846::Pref<units::foot_t> shooter_range_{shooting_named_, "shooter_range", 11_ft};
  frc846::Pref<double> shooter_speed_tolerance_{shooting_named_, "shooter_speed_tolerance", 0.2};
  frc846::Pref<double> shoot_angle_calc_tolerance_{shooting_named_, "shoot_angle_calc_tolerance", 0.06};
  frc846::Pref<double> shoot_angle_calc_max_iterations_{shooting_named_, "shoot_angle_calc_max_iterations", 5000};
  frc846::Pref<units::degree_t> shoot_angle_calc_intial_guess_{shooting_named_, "shoot_angle_calc_intial_guess", 1.01_deg};
  frc846::Pref<double> shoot_drive_angle_calc_tolerance_{shooting_named_, "shoot_drive_angle_calc_tolerance", 0.06};
  frc846::Pref<double> shoot_drive_angle_calc_max_iterations_{shooting_named_, "shoot_drive_angle_calc_max_iterations", 5000};
  frc846::Pref<units::degree_t> shoot_drive_angle_calc_intial_guess_{shooting_named_, "shoot_drive_angle_calc_intial_guess", 1.01_deg};

  frc846::Pref<units::inch_t> trap_start_x{shooting_named_, "trap_start_x", 17_in};
  frc846::Pref<units::inch_t> trap_start_y{shooting_named_, "trap_start_y", 25_in};
  frc846::Pref<units::inch_t> trap_end_x{shooting_named_, "trap_fin_x", 25_in};
  frc846::Pref<units::inch_t> trap_end_y{shooting_named_, "trap_fin_y", 47_in};
  frc846::Pref<units::degree_t> trap_start_angle{shooting_named_, "trap_start_a", 60_deg};
  frc846::Pref<units::degree_t> trap_end_angle{shooting_named_, "trap_end_a", -10_deg};


 private:
  
  bool hasZeroed = false;

  frc846::Pref<bool> is_red_side_{*this, "is_red_side", true};

  SuperStructureReadings GetNewReadings() override;

  void DirectWrite(SuperStructureTarget target) override;
};

#endif