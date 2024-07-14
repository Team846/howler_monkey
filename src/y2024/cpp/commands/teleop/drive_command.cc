#include "commands/teleop/drive_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/swerve_module.h"

DriveCommand::DriveCommand(RobotContainer& container)
    : driver_(container.driver_),
      drivetrain_(container.drivetrain_),
      super_(container.super_structure_),
      vision_(container.vision_),
      shooter_(container.shooter_) {
  AddRequirements({&drivetrain_});
  SetName("drive_command");
}

void DriveCommand::Execute() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // Left bumper   | robot centric translation
  // Right bumper  | precision drive

  bool is_robot_centric = false;
  // bool is_slow_drive = driver_.readings().right_bumper;
  bool prep_align_speaker = driver_.readings().y_button;
  bool amping = driver_.readings().left_bumper;
  bool sourcing = driver_.readings().x_button;

  bool flipping_controls =
      !frc846::util::ShareTables::GetBoolean("is_red_side");

  // -----TRANSLATION CONTROL-----

  double translate_x = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_x, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
  double translate_y = frc846::util::HorizontalDeadband(
      driver_.readings().left_stick_y, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);

  if (flipping_controls) {
    translate_x = -translate_x;
    translate_y = -translate_y;
  }

  if (prep_align_speaker) {
    translate_x = units::math::min(0.75, units::math::max(translate_x, -0.75));
    translate_y = units::math::min(0.75, units::math::max(translate_y, -0.75));
  }

  drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value() *
                          drivetrain_.driver_speed_multiplier_.value();
  drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value() *
                          drivetrain_.driver_speed_multiplier_.value();

  // Slow down translation if slow mode is active
  // if (is_slow_drive) {
  //   translate_x = frc846::util::HorizontalDeadband(
  //       driver_.readings().left_stick_x *
  //           std::abs(driver_.readings().left_stick_x),
  //       driver_.translation_deadband_.value(), 1,
  //       driver_.translation_exponent_.value(), 1);
  //   translate_y = frc846::util::HorizontalDeadband(
  //       driver_.readings().left_stick_y *
  //           std::abs(driver_.readings().left_stick_y),
  //       driver_.translation_deadband_.value(), 1,
  //       driver_.translation_exponent_.value(), 1);
  //   drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value() *
  //                           drivetrain_.slow_mode_percent_.value();
  //   drivetrain_target.v_y = (prep_align_speaker ? 0.0 : translate_y) *
  //                           drivetrain_.max_speed_.value() *
  //                           drivetrain_.slow_mode_percent_.value();
  // }

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
    // if (is_slow_drive) {
    //   target *= drivetrain_.slow_omega_percent_.value();
    // }

    drivetrain_target.rotation = DrivetrainRotationVelocity(target);
  } else {
    // Hold position
    drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  }

  if (prep_align_speaker) {
    VisionReadings vision_readings = vision_.readings();
    double shooting_dist = vision_readings.est_dist_from_speaker.to<double>();

    shooting_dist =
        shooting_dist + super_.teleop_shooter_x_.value().to<double>() / 12.0;

    units::degree_t theta_adjust =
        shooting_calculator
            .calculateLaunchAngles(
                shooter_.shooting_exit_velocity_.value(), shooting_dist,
                vision_readings.velocity_in_component,
                vision_readings.velocity_orth_component,
                super_.teleop_shooter_height_.value().to<double>())
            .turning_offset_angle;

    if (units::math::abs(vision_readings.est_dist_from_speaker_x) > 0.5_ft ||
        units::math::abs(vision_readings.est_dist_from_speaker_y) > 0.5_ft) {
      auto target_angle =
          units::math::atan2(vision_readings.est_dist_from_speaker_x,
                             vision_readings.est_dist_from_speaker_y);

      drivetrain_target.rotation =
          DrivetrainRotationPosition(target_angle + theta_adjust);
    }
  } else if (amping) {
    driver_adjust_ += driver_.readings().right_stick_x / 5.0;
    driver_adjust_ = std::min(std::max(driver_adjust_, -10.0), 10.0);

    // if (!flipping_controls) {
    drivetrain_target.rotation =
        DrivetrainRotationPosition(90_deg + units::degree_t(driver_adjust_));
    // } else {
    //   drivetrain_target.rotation =
    //       DrivetrainRotationPosition(-90_deg +
    //       units::degree_t(driver_adjust_));
    // }
  } else if (sourcing) {
    driver_adjust_ += driver_.readings().right_stick_x / 5.0;
    driver_adjust_ = std::min(std::max(driver_adjust_, -10.0), 10.0);

    if (!flipping_controls) {
      drivetrain_target.rotation = DrivetrainRotationPosition(
          -51.4_deg + units::degree_t(driver_adjust_));
    } else {
      drivetrain_target.rotation = DrivetrainRotationPosition(
          51.4_deg + units::degree_t(driver_adjust_));
    }
  } else {
    driver_adjust_ = 0.0;
  }

  drivetrain_.SetTarget(drivetrain_target);
}

bool DriveCommand::IsFinished() { return false; }