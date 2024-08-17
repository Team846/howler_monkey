#pragma once

#include <AHRS.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>
#include <variant>

#include "frc/filter/SlewRateLimiter.h"
#include "frc846/other/swerve_odometry.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/util/conversions.h"
#include "frc846/util/grapher.h"
#include "frc846/util/math.h"
#include "frc846/util/pref.h"
#include "frc846/wpilib/time.h"
#include "ports.h"
#include "subsystems/hardware/swerve_module.h"

struct DrivetrainReadings {
  bool is_gyro_connected;
  frc846::util::Position pose;
  units::degrees_per_second_t angular_velocity;
  units::degree_t tilt;
  frc846::util::Vector2D<units::feet_per_second_t> velocity;
};

// Robot vs field oriented translation control.
enum DrivetrainTranslationReference { kRobot, kField };

// Position control of drivetrain steering.
using DrivetrainRotationPosition = units::degree_t;

// Velocity control of drivetrain steering.
using DrivetrainRotationVelocity = units::degrees_per_second_t;

// Drivetrain rotation target.
using DrivetrainRotation =
    std::variant<DrivetrainRotationPosition, DrivetrainRotationVelocity>;

struct DrivetrainTarget {
  units::feet_per_second_t v_x;
  units::feet_per_second_t v_y;
  DrivetrainTranslationReference translation_reference;

  DrivetrainRotation rotation;
  DrivetrainControl control;
};

class DrivetrainSubsystem
    : public frc846::robot::GenericSubsystem<DrivetrainReadings,
                                             DrivetrainTarget> {
 public:
  DrivetrainSubsystem(bool initialize = true);

  void Setup() override {
    for (auto swerve_module : modules_all_) {
      swerve_module->Setup();
    }
  }

  // Number of swerve modules (avoid hardcoding 4 in loops and such).
  static constexpr int kModuleCount = 4;

  // Zero the modules with their CANCoders.
  void ZeroModules();

  void ZeroCancoders();

  // Zero bearing with the gyro.
  void ZeroBearing();

  // Zero bearing and reset odometry to zero.
  void ZeroOdometry();

  // Set odometry point.
  void SetPoint(frc846::util::Vector2D<units::foot_t> point);

  // Set bearing.
  void SetBearing(units::degree_t bearing);

  // Set Map
  void SetMap();

  // Max drivetrain speed (NEO SDS Mk4i L1 -> 12 theoretical).
  frc846::Pref<units::feet_per_second_t> max_speed_{*this, "max_speed",
                                                    14.2_fps};

  frc846::Pref<units::feet_per_second_t> close_drive_amp_max_speed_{
      *this, "close_drive_amp_max_speed", 3_fps};

  frc846::Pref<units::ampere_t> current_limit_{*this, "current_limit", 90_A};
  frc846::Pref<units::ampere_t> motor_stall_current_{
      *this, "motor_stall_current", 366_A};

  frc846::Pref<double> braking_constant_{*this, "braking_constant", 0.15};

  frc846::Pref<units::feet_per_second_t> vx_ramp_rate_limit{
      *this, "ramp_rate_vx", 30_fps};
  frc846::Pref<units::feet_per_second_t> vy_ramp_rate_limit{
      *this, "ramp_rate_vy", 100_fps};

  frc::SlewRateLimiter<units::fps> vx_ramp_rate_{vx_ramp_rate_limit.value() /
                                                 1_s};
  frc::SlewRateLimiter<units::fps> vy_ramp_rate_{vy_ramp_rate_limit.value() /
                                                 1_s};

  // Closed loop tuned for this
  frc846::Pref<units::feet_per_second_t> auto_max_speed_{
      *this, "auto_max_speed", 11.2_fps};

  frc846::Pref<double> driver_speed_multiplier_{*this,
                                                "driver_speed_multiplier", 1.0};

  frc846::Pref<double> slow_mode_percent_{*this, "slow_mode_percent", 0.04};
  frc846::Pref<double> slow_omega_percent_{*this, "slow_omega_percent", 0.12};
  frc846::Pref<double> pov_control_speed_{*this, "pov_control_speed_", 1.0};
  frc846::Pref<double> max_horizontal_strafe_{*this, "pov_control_speed_",
                                              10.0};

  frc846::Pref<units::feet_per_second_t> velocity_error{*this, "velocity_error",
                                                        0_fps};

  // Max turning speed.
  units::degrees_per_second_t max_omega() const {
    return max_speed_.value() / module_radius_ * 1_rad *
           percent_max_omega_.value();
  }

  // Max drivetrain acceleration for trajectory generation.
  frc846::Pref<units::feet_per_second_squared_t> max_acceleration_{
      *this, "max_acceleration", 8_fps_sq};

  // Max drivetrain deceleration for trajectory generation.
  frc846::Pref<units::feet_per_second_squared_t> max_deceleration_{
      *this, "max_deceleration", 10_fps_sq};

  // Lookahead distance during trajectory following.
  frc846::Pref<units::inch_t> extrapolation_distance_{
      *this, "extrapolation_distance", 8_in};

  frc846::Pref<units::degrees_per_second_t> angular_velocity_threshold_{
      *this, "angular_velocity_threshold", 1_deg_per_s};

  frc846::base::Loggable align_gains_loggable_{*this, "align_gains"};
  frc846::Pref<double> align_gains_p_{align_gains_loggable_, "p", 3.5};

  // Auto align tolerance
  frc846::Pref<units::inch_t> align_tolerance_{align_gains_loggable_,
                                               "align_tolerance", 0.3_in};

  // Convert a translation vector and the drivetrain angular velocity to the
  // individual module outputs.
  static std::array<frc846::util::Vector2D<units::feet_per_second_t>,
                    kModuleCount>
  SwerveControl(frc846::util::Vector2D<units::feet_per_second_t> translation,
                units::degrees_per_second_t rotation_speed, units::inch_t width,
                units::inch_t height, units::inch_t radius,
                units::feet_per_second_t max_speed);

  DrivetrainTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  int lastRelocalize = 0;

  // Drivetrain dimensions.
  frc846::Pref<units::inch_t> width_{*this, "width", 21.75_in};
  frc846::Pref<units::inch_t> height_{*this, "height", 26.75_in};

  // How much to scale the max turning speed by.
  frc846::Pref<double> percent_max_omega_{*this, "percent_max_omega", 0.45};

  // Distance from center of robot to module.
  units::inch_t module_radius_ =
      units::math::sqrt(units::math::pow<2>(width_.value() / 2) +
                        units::math::pow<2>(height_.value() / 2));

  // Wheel radius for odometry. 4" wheels.
  frc846::Pref<units::inch_t> wheel_radius_{*this, "wheel_radius", 1.93_in};

  // Rotation position gains.
  frc846::base::Loggable bearing_gains_loggable_{*this, "bearing_gains"};
  frc846::Pref<double> bearing_gains_p_{bearing_gains_loggable_, "p", 8.3};
  frc846::Pref<double> bearing_gains_d_{bearing_gains_loggable_, "d", -4.7};

  // Pose graphers.
  frc846::base::Loggable pose_loggable_{*this, "pose"};
  frc846::Grapher<units::foot_t> pose_x_graph_{pose_loggable_, "x"};
  frc846::Grapher<units::foot_t> pose_y_graph_{pose_loggable_, "y"};
  frc846::Grapher<units::degree_t> pose_bearing_graph{pose_loggable_,
                                                      "bearing"};

  // Velocity graphers.
  frc846::base::Loggable velocity_loggable_{*this, "velocity"};
  frc846::Grapher<units::feet_per_second_t> v_x_graph_{velocity_loggable_,
                                                       "v_x"};
  frc846::Grapher<units::feet_per_second_t> v_y_graph_{velocity_loggable_,
                                                       "v_y"};

  // Target graphers.
  frc846::base::Loggable target_loggable_{*this, "target"};
  frc846::Grapher<units::feet_per_second_t> target_v_x_graph_{target_loggable_,
                                                              "v_x"};
  frc846::Grapher<units::feet_per_second_t> target_v_y_graph_{target_loggable_,
                                                              "v_y"};
  frc846::Grapher<std::string> target_translation_reference_graph_{
      target_loggable_,
      "translation_reference",
  };
  frc846::Grapher<units::degree_t> target_rotation_position_graph_{
      target_loggable_, "rotation_position"};
  frc846::Grapher<units::degrees_per_second_t> target_rotation_velocity_graph_{
      target_loggable_, "rotation_velocity"};
  //   frc846::Grapher<units::degree_t> bearing_error{target_loggable_,
  //   "bearing_error"};

  frc846::SwerveOdometry odometry_;
  units::angle::degree_t bearing_offset_;

  frc846::base::Loggable drive_esc_loggable_{*this, "drive_esc"};
  frc846::base::Loggable steer_esc_loggable_{*this, "steer_esc"};
  frc::Field2d m_field;

  frc846::control::ConfigHelper drive_config_helper_{
      drive_esc_loggable_,
      {false,
       (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
           frc846::util::Circumference(wheel_radius_.value()).to<double>() /
           12.0,
       frc846::control::MotorIdleMode::kDefaultBrake,
       {80_A}},
      {0.0002, 0.0, 0.0001769},
  };

  frc846::control::ConfigHelper steer_config_helper_{
      steer_esc_loggable_,
      {false,
       (7.0 / 150.0) * 360.0,
       frc846::control::MotorIdleMode::kDefaultCoast,
       {40_A}},
      {0.12, 0.0},
  };

  frc846::base::Loggable current_braking_loggable{*this,
                                                  "smart_current_braking"};

  frc846::motion::CurrentControl current_braking{
      current_braking_loggable,
      {170_A, frc846::control::DefaultSpecifications::stall_current_kraken,
       0.45}};

  SwerveModuleSubsystem module_fl_{
      *this,
      is_initialized(),
      "FL",
      3.23_deg,
      &drive_config_helper_,
      &steer_config_helper_,
      ports::drivetrain_::kFLDrive_CANID,
      ports::drivetrain_::kFLSteer_CANID,
      ports::drivetrain_::kFLCANCoder_CANID,
      max_speed_,
      current_braking,
  };

  SwerveModuleSubsystem module_fr_{
      *this,
      is_initialized(),
      "FR",
      140.05_deg,
      &drive_config_helper_,
      &steer_config_helper_,
      ports::drivetrain_::kFRDrive_CANID,
      ports::drivetrain_::kFRSteer_CANID,
      ports::drivetrain_::kFRCANCoder_CANID,
      max_speed_,
      current_braking,
  };

  SwerveModuleSubsystem module_bl_{
      *this,
      is_initialized(),
      "BL",
      297.5_deg,
      &drive_config_helper_,
      &steer_config_helper_,
      ports::drivetrain_::kBLDrive_CANID,
      ports::drivetrain_::kBLSteer_CANID,
      ports::drivetrain_::kBLCANCoder_CANID,
      max_speed_,
      current_braking,
  };

  SwerveModuleSubsystem module_br_{
      *this,
      is_initialized(),
      "BR",
      157.89_deg,
      &drive_config_helper_,
      &steer_config_helper_,
      ports::drivetrain_::kBRDrive_CANID,
      ports::drivetrain_::kBRSteer_CANID,
      ports::drivetrain_::kBRCANCoder_CANID,
      max_speed_,
      current_braking,
  };

  SwerveModuleSubsystem* modules_all_[kModuleCount]{&module_fl_, &module_fr_,
                                                    &module_bl_, &module_br_};

  AHRS gyro_{frc::SerialPort::kMXP};

  DrivetrainReadings ReadFromHardware() override;

  void WriteToHardware(DrivetrainTarget target) override;
};
