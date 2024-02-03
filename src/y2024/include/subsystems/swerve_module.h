#ifndef y2024_SUBSYSTEMS_SWERVE_MODULE_H_
#define y2024_SUBSYSTEMS_SWERVE_MODULE_H_

#include <ctre/phoenix/sensors/CANCoder.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <string>

#include "frc846/conversions.h"
#include "frc846/ctre_namespace.h"
#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/motor/config.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"

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
      frc846::motor::SparkMAXConfigHelper* drive_esc_config_helper,
      frc846::motor::GainsHelper* drive_esc_gains_helper,
      frc846::motor::SparkMAXConfigHelper* steer_esc_config_helper,
      frc846::motor::GainsHelper* steer_esc_gains_helper,
      frc846::Converter<units::foot_t>& drive_converter,
      frc846::Converter<units::degree_t>& steer_converter, int drive_esc_id,
      int steer_esc_id, int cancoder_id);

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

  //todo, get these from drivetrain or make them prefs
  constexpr const static units::feet_per_second_t kMaxSpeed = 14.2_fps;

  constexpr const static units::feet_per_second_t kControlChangeThresh = 0.3_fps;

  constexpr const static double kSpeedAdjustingFactor = 0.970;

  // CANcoder magnet offset.
  frc846::Pref<units::degree_t> cancoder_offset_;

  // Magic value to offset steer position by (factors in CANcoder reading,
  // CANcoder offset, and Talon relative encoder start reading).
  units::degree_t zero_offset_;

  units::degree_t last_direction_;

  units::feet_per_second_t current_speed_;

  frc846::Loggable target_loggable_{*this, "target"};
  frc846::Grapher<units::feet_per_second_t> target_speed_graph_{target_loggable_,
                                                                "speed"};
  frc846::Grapher<units::degree_t> target_direction_graph_{target_loggable_,
                                                           "direction"};
  frc846::Grapher<units::degree_t> cancoder_graph_{*this, "cancoder"};

  frc846::Grapher<units::degree_t> direction_graph_{*this, "direction"};
  frc846::Grapher<units::ampere_t> current_graph_{*this,
                                                                "current"};

  frc846::Grapher<double> swerve_target_graph_{*this,
                                                                "swerve_target_graph_"};

  frc846::Grapher<double> swerve_speed_graph_{*this,
                                                                "swerve_speed_graph_"};

  frc846::Converter<units::foot_t>& drive_converter_;
  frc846::Converter<units::degree_t>& steer_converter_;

  rev::CANSparkMax drive_esc_;
  rev::CANSparkMax steer_esc_;

  frc846::motor::SparkMAXHelper drive_esc_helper_;
  frc846::motor::SparkMAXHelper steer_esc_helper_;


  ctre::CANCoder cancoder_;

  SwerveModuleReadings GetNewReadings() override;

  void DirectWrite(SwerveModuleTarget target) override;
};

#endif  // y2024_SUBSYSTEMS_SWERVE_MODULE_H_