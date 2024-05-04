#ifndef y2024_SUBSYSTEMS_SWERVE_MODULE_H_
#define y2024_SUBSYSTEMS_SWERVE_MODULE_H_

#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <string>

#include "frc846/control/control.h"
#include "frc846/control/controlgains.h"
#include "frc846/ctre_namespace.h"
#include "frc846/subsystem.h"
#include "frc846/util/conversions.h"
#include "frc846/util/grapher.h"
#include "frc846/util/math.h"
#include "frc846/util/pref.h"

FRC846_CTRE_NAMESPACE()

// Open loop vs closed loop control
enum DrivetrainControl { kOpenLoop, kClosedLoop };

struct SwerveModuleReadings {
  units::feet_per_second_t speed;
  units::degree_t direction;
  units::foot_t distance;
};

struct SwerveModuleTarget {
  units::feet_per_second_t speed;
  units::degree_t direction;
  DrivetrainControl control;
};

class SwerveModuleSubsystem
    : public frc846::Subsystem<SwerveModuleReadings, SwerveModuleTarget> {
 public:
  SwerveModuleSubsystem(
      const frc846::Loggable& drivetrain, bool init, std::string location,
      units::degree_t fallback_cancoder_offset,
      frc846::control::ControlGainsHelper* drive_esc_gains_helper,
      frc846::control::ControlGainsHelper* steer_esc_gains_helper,
      units::foot_t drive_conversion, units::degree_t steer_conversion,
      int drive_esc_id, int steer_esc_id, int cancoder_id,
      frc846::Pref<units::ampere_t>& current_limit,
      frc846::Pref<units::ampere_t>& motor_stall_current,
      frc846::Pref<double>& braking_constant,
      frc846::Pref<units::feet_per_second_t>& max_speed);

  // Calculate the normalized target angle for the module to minimize rotations.
  // Returns the normalized direction and whether or not the drive motor should
  // be reversed.
  static std::pair<units::degree_t, bool> NormalizedDirection(
      units::degree_t current, units::degree_t target);

  // Zero the module positions with the CANcoder.
  void ZeroWithCANcoder();

  void ZeroCancoder();

  SwerveModuleTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // CANcoder magnet offset.
  frc846::Pref<units::degree_t> cancoder_offset_;

  // Magic value to offset steer position by (factors in CANcoder reading,
  // CANcoder offset, and Talon relative encoder start reading).
  units::degree_t zero_offset_;

  units::degree_t last_direction_;

  units::feet_per_second_t current_speed_;

  frc846::Loggable target_loggable_{*this, "target"};
  frc846::Grapher<units::feet_per_second_t> target_speed_graph_{
      target_loggable_, "speed"};
  frc846::Grapher<units::degree_t> target_direction_graph_{target_loggable_,
                                                           "direction"};
  frc846::Grapher<units::degree_t> cancoder_graph_{*this, "cancoder"};

  frc846::Grapher<units::degree_t> direction_graph_{*this, "direction"};
  frc846::Grapher<units::ampere_t> current_graph_{*this, "current"};

  frc846::Grapher<double> swerve_target_graph_{*this, "swerve_target_graph_"};

  frc846::Grapher<double> swerve_speed_graph_{*this, "swerve_speed_graph_"};

  frc846::Grapher<double> upper_dc_current_limiting_{
      *this, "upper_dc_current_limiting"};

  frc846::Grapher<double> lower_dc_current_limiting_{
      *this, "lower_dc_current_limiting"};

  // frc846::control::SparkRevController<units::foot_t> drive_esc_helper_;
  // frc846::control::SparkRevController<units::degree_t> steer_esc_helper_;

  frc846::control::TalonFXController<units::foot_t> drive_esc_helper_;
  frc846::control::TalonFXController<units::degree_t> steer_esc_helper_;

  ctre::CANcoder cancoder_;

  frc846::Pref<units::ampere_t>& current_limit_;
  frc846::Pref<units::ampere_t>& motor_stall_current_;

  frc846::Pref<double>& braking_constant_;

  frc846::Pref<units::feet_per_second_t>& max_speed_;

  SwerveModuleReadings GetNewReadings() override;

  void DirectWrite(SwerveModuleTarget target) override;
};

#endif  // y2024_SUBSYSTEMS_SWERVE_MODULE_H_