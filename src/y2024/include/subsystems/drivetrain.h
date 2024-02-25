#ifndef y2024_SUBSYSTEMS_DRIVETRAIN_H_
#define y2024_SUBSYSTEMS_DRIVETRAIN_H_

#include <AHRS.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/angle.h>

#include <array>
#include <variant>

#include "frc/filter/SlewRateLimiter.h"
#include "frc846/util/conversions.h"
#include "frc846/util/grapher.h"
#include "frc846/util/math.h"
#include "frc846/control/controlgains.h"
#include "frc846/util/pref.h"
#include "frc846/subsystem.h"
#include "frc846/other/swerve_odometry.h"
#include "frc846/wpilib/time.h"
#include "ports.h"
#include "subsystems/swerve_module.h"

struct DrivetrainReadings {
  bool is_gyro_connected;
  frc846::util::Position pose;
  units::degrees_per_second_t angular_velocity;
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
    : public frc846::Subsystem<DrivetrainReadings, DrivetrainTarget> {
 public:
  DrivetrainSubsystem(bool initialize = true);

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

  // Max drivetrain speed (NEO SDS Mk4i L1 -> 12 theoretical).
  frc846::Pref<units::feet_per_second_t> max_speed_{*this, "max_speed",
                                                    14.2_fps};

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

  frc846::Pref<double> slow_mode_percent_{*this, "slow_mode_percent", 0.04};
  frc846::Pref<double> slow_omega_percent_{*this, "slow_omega_percent", 0.12};
  frc846::Pref<double> pov_control_speed_{*this, "pov_control_speed_", 1.0};
  frc846::Pref<double> max_horizontal_strafe_{*this, "pov_control_speed_", 10.0};

  frc846::Pref<units::feet_per_second_t> velocity_error{*this, "velocity_error", 0_fps};

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

  frc846::Loggable align_gains_loggable_{*this, "align_gains"};
  frc846::Pref<double> align_gains_p_{align_gains_loggable_, "p", 3.5};

  // Auto align tolerance
  frc846::Pref<units::inch_t> align_tolerance_{align_gains_loggable_,
                                               "align_tolerance", 0.3_in};


    //April Tag 
    units::second_t aprilTagFrameTime;
    frc846::util::Position poseAtFrameCapture;
    bool updatedTagPos=false;
    frc846::Loggable april_tags_named_{*this, "april_tags"};
    frc846::Pref<double> confidence_factor_{april_tags_named_, "april_confidence_factor",1.0};
    frc846::Pref<double> velocity_factor_{april_tags_named_, "april_velocity_factor", 1.0};
    frc846::Pref<double> distance_factor_{april_tags_named_, "april_distance_factor", 1.0};
    frc846::Pref<double> angle_offset_factor_{april_tags_named_, "april_angle_factor", 1.0};

    bool aprilFrameRequested=false;
    long aprilFrameRequest=0;
    frc846::util::Position poseAtLastRequest;

    nt::NetworkTableInstance nt_table =
        nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> aprilTag_table =
        nt::NetworkTableInstance::GetDefault().GetTable("AprilTags");
    frc846::Pref<bool> april_tags_enabled_{april_tags_named_, "init_april_tags", true};


  // Convert a translation vector and the drivetrain angular velocity to the
  // individual module outputs.
  static std::array<frc846::util::Vector2D<units::feet_per_second_t>, kModuleCount>
  SwerveControl(frc846::util::Vector2D<units::feet_per_second_t> translation,
                units::degrees_per_second_t rotation_speed, units::inch_t width,
                units::inch_t height, units::inch_t radius,
                units::feet_per_second_t max_speed);

  DrivetrainTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
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
  frc846::Loggable bearing_gains_loggable_{*this, "bearing_gains"};
  frc846::Pref<double> bearing_gains_p_{bearing_gains_loggable_, "p", 8.3};
  frc846::Pref<double> bearing_gains_d_{bearing_gains_loggable_, "d", -4.7};

  // Pose graphers.
  frc846::Loggable pose_loggable_{*this, "pose"};
  frc846::Grapher<units::foot_t> pose_x_graph_{pose_loggable_, "x"};
  frc846::Grapher<units::foot_t> pose_y_graph_{pose_loggable_, "y"};
  frc846::Grapher<units::degree_t> pose_bearing_graph{pose_loggable_, "bearing"};

  // Velocity graphers.
  frc846::Loggable velocity_loggable_{*this, "velocity"};
  frc846::Grapher<units::feet_per_second_t> v_x_graph_{velocity_loggable_, "v_x"};
  frc846::Grapher<units::feet_per_second_t> v_y_graph_{velocity_loggable_, "v_y"};

  // Target graphers.
  frc846::Loggable target_loggable_{*this, "target"};
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

  frc846::SwerveOdometry odometry_;
  units::angle::degree_t bearing_offset_;

  frc846::Loggable drive_esc_loggable_{*this, "drive_esc"};
  frc846::Loggable steer_esc_loggable_{*this, "steer_esc"};
  frc::Field2d m_field;

  frc846::control::ControlGainsHelper* drive_esc_gains_helper_ =
      new frc846::control::ControlGainsHelper{
          drive_esc_loggable_,
          {
              0.0002,    /* p */
              0,         /* i */
              0,         /* d */
              0.0001769, /* f */
              0,         /* max_integral_accumulator */
          },
          80_A, // NEO smart current limit
          1.0,  // peak output
      };

  frc846::control::ControlGainsHelper* steer_esc_gains_helper_ =
      new frc846::control::ControlGainsHelper{
          steer_esc_loggable_,
          {
              0.12,      /* p */
              0,         /* i */
              0,         /* d */
              0,         /* f */
              0,         /* max_integral_accumulator */
          },
          40_A, // NEO smart current limit
          1.0,  // peak output
      };

  units::foot_t drive_conversion_ = 
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
          frc846::util::Circumference(wheel_radius_.value());
  
  units::degree_t steer_conversion_ = (7.0 / 150.0) * 1_tr;

  SwerveModuleSubsystem module_fl_{
      *this,
      is_initialized(),
      "FL",
      3.23_deg,
      drive_esc_gains_helper_,
      steer_esc_gains_helper_,
      drive_conversion_,
      steer_conversion_,
      ports::drivetrain_::kFLDrive_CANID,
      ports::drivetrain_::kFLSteer_CANID,
      ports::drivetrain_::kFLCANCoder_CANID,
  };

  SwerveModuleSubsystem module_fr_{
      *this,
      is_initialized(),
      "FR",
      140.05_deg,
      drive_esc_gains_helper_,
      steer_esc_gains_helper_,
      drive_conversion_,
      steer_conversion_,
      ports::drivetrain_::kFRDrive_CANID,
      ports::drivetrain_::kFRSteer_CANID,
      ports::drivetrain_::kFRCANCoder_CANID,
  };

  SwerveModuleSubsystem module_bl_{
      *this,
      is_initialized(),
      "BL",
      297.5_deg,
      drive_esc_gains_helper_,
      steer_esc_gains_helper_,
      drive_conversion_,
      steer_conversion_,
      ports::drivetrain_::kBLDrive_CANID,
      ports::drivetrain_::kBLSteer_CANID,
      ports::drivetrain_::kBLCANCoder_CANID,
  };

  SwerveModuleSubsystem module_br_{
      *this,
      is_initialized(),
      "BR",
      157.89_deg,
      drive_esc_gains_helper_,
      steer_esc_gains_helper_,
      drive_conversion_,
      steer_conversion_,
      ports::drivetrain_::kBRDrive_CANID,
      ports::drivetrain_::kBRSteer_CANID,
      ports::drivetrain_::kBRCANCoder_CANID,
  };

  SwerveModuleSubsystem* modules_all_[kModuleCount]{&module_fl_, &module_fr_,
                                                    &module_bl_, &module_br_};

  AHRS gyro_{frc::SPI::Port::kMXP};

  DrivetrainReadings GetNewReadings() override;

  void DirectWrite(DrivetrainTarget target) override;
};

#endif  // y2024_SUBSYSTEMS_DRIVETRAIN_H_