#include "commands/drive_command.h"

#include <utility>

#include "frc846/util/math.h"
#include "subsystems/field.h"
#include "subsystems/drivetrain.h"
#include "subsystems/swerve_module.h"
#include "frc846/loggable.h"

DriveCommand::DriveCommand(RobotContainer& container)
    : driver_(container.driver_),
      drivetrain_(container.drivetrain_) {
  AddRequirements({&drivetrain_});
  SetName("drive_command");
}

void DriveCommand::Execute() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // Left bumper   | robot centric translation
  // Right bumper  | precision drive

  bool is_robot_centric = false;
  bool is_slow_drive = driver_.readings().right_bumper;
  bool prep_align_speaker = driver_.readings().right_trigger;


  // -----TRANSLATION CONTROL-----

  double translate_x = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_x, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
  double translate_y = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_y, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
      
  if (prep_align_speaker) {
    translate_x = units::math::min(0.75, units::math::max(translate_x, -0.75));
    translate_y = units::math::min(0.75, units::math::max(translate_y, -0.75));
  }

  drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value();
  drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value();

  // Slow down translation if slow mode is active
  if (is_slow_drive) {
    translate_x = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_x*abs(driver_.readings().left_stick_x), driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
    translate_y = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_y*abs(driver_.readings().left_stick_y), driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
    drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value() * drivetrain_.slow_mode_percent_.value();
    drivetrain_target.v_y = (prep_align_speaker ? 0.0 : translate_y) * drivetrain_.max_speed_.value() * drivetrain_.slow_mode_percent_.value();
  }

  // Robot vs field oriented translation
  drivetrain_target.translation_reference =
      (is_robot_centric) ? DrivetrainTranslationReference::kRobot
                       : DrivetrainTranslationReference::kField;

  drivetrain_target.control = kOpenLoop;

  // -----STEER CONTROL-----

  double steer_x = frc846::util::HorizontalDeadband(
      driver_.readings().right_stick_x, driver_.steer_deadband_.value(), 1,
      driver_.steer_exponent_.value(), 1);

  if (steer_x != 0) {
    // Manual steer
    auto target = steer_x * drivetrain_.max_omega();

    // Slow down steering if slow mode is active
    if (is_slow_drive) {
      target *= drivetrain_.slow_omega_percent_.value();
    }

    drivetrain_target.rotation = DrivetrainRotationVelocity(target);
  } else {
    // Hold position
    drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  }

  if (prep_align_speaker) {
    auto current_x = drivetrain_.readings().pose.point.x;
    auto current_y = drivetrain_.readings().pose.point.y;
    auto target_x = field::points::kSpeakerTeleop().x;
    auto target_y = field::points::kSpeakerTeleop().y;

    auto dist_x = target_x - current_x;
    auto dist_y = target_y - current_y;

    if (units::math::abs(dist_x) > 0.5_ft || units::math::abs(dist_y) > 0.5_ft) {
      auto target_angle = units::math::atan2(dist_x, dist_y);

      double SPEAKER_HEIGHT = 3.7;
      double LAUNCH_VELOCITY = 27.0;
      double GRAVITY = 32.0;

      // Re-angle to correct sideways motion:

      // h = 1/2 * g * t^2
      // t = sqrt(2h/g)
      // s = (r_v_orth)*t
      // vtan(theta) = s
      // theta = atan(r_v_orth*t/v)

      auto robot_velocity = drivetrain_.readings().velocity;
      auto point_target = (field::points::kSpeakerTeleop() - drivetrain_.readings().pose.point);

      point_target = point_target.Rotate(90_deg);

      double robot_velocity_orth_component = 
        (robot_velocity.x.to<double>() * point_target.x.to<double>() + 
          robot_velocity.y.to<double>() * point_target.y.to<double>())/point_target.Magnitude().to<double>();

      units::degree_t theta_adjust =  units::radian_t(std::atan2(robot_velocity_orth_component * 
        std::sqrt(2 * SPEAKER_HEIGHT / GRAVITY), LAUNCH_VELOCITY));

      
      drivetrain_target.rotation = DrivetrainRotationPosition(target_angle + 180_deg - theta_adjust);
    }
  }


  drivetrain_.SetTarget(drivetrain_target);
}

bool DriveCommand::IsFinished() { return false; }