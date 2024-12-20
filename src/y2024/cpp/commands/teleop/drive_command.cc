#include "commands/teleop/drive_command.h"

#include <utility>

#include "frc846/base/loggable.h"
#include "frc846/math/collection.h"
#include "frc846/util/share_tables.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/swerve_module.h"

DriveCommand::DriveCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, DriveCommand>{
          container, "drive_command"} {
  AddRequirements({&container_.drivetrain_});

  ShootingCalculatorConstants sc_constants{78.0_in};
  shooting_calculator.setConstants(sc_constants);
}

void DriveCommand::OnInit() { driver_adjust_ = 0.0; }

void DriveCommand::Periodic() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // TODO: Add button mappings

  bool is_robot_centric = false;
  bool prep_align_speaker =
      container_.control_input_.GetReadings().running_super_shoot;
  bool amping = container_.control_input_.GetReadings().running_amp;
  bool intaking = container_.control_input_.GetReadings().running_intake;
  bool sourcing = container_.control_input_.GetReadings().running_source;

  bool flipping_controls =
      !frc846::util::ShareTables::GetBoolean("is_red_side");

  // -----TRANSLATION CONTROL-----

  double translate_x = frc846::math::HorizontalDeadband(
      container_.control_input_.GetReadings().translate_x,
      container_.control_input_.driver_.translation_deadband_.value(), 1,
      container_.control_input_.driver_.translation_exponent_.value(), 1);
  double translate_y = frc846::math::HorizontalDeadband(
      container_.control_input_.GetReadings().translate_y,
      container_.control_input_.driver_.translation_deadband_.value(), 1,
      container_.control_input_.driver_.translation_exponent_.value(), 1);

  if (flipping_controls) {
    translate_x = -translate_x;
    translate_y = -translate_y;
  }

  if (prep_align_speaker) {
    translate_x = units::math::min(0.75, units::math::max(translate_x, -0.75));
    translate_y = units::math::min(0.75, units::math::max(translate_y, -0.75));
  }

  drivetrain_target.v_x =
      translate_x * container_.drivetrain_.max_speed_.value() *
      container_.drivetrain_.driver_speed_multiplier_.value();
  drivetrain_target.v_y =
      translate_y * container_.drivetrain_.max_speed_.value() *
      container_.drivetrain_.driver_speed_multiplier_.value();

  // Robot vs field oriented translation
  drivetrain_target.translation_reference =
      (is_robot_centric) ? DrivetrainTranslationReference::kRobot
                         : DrivetrainTranslationReference::kField;

  drivetrain_target.control = kOpenLoop;

  // -----STEER CONTROL-----

  double steer_x = frc846::math::HorizontalDeadband(
      container_.control_input_.GetReadings().rotation,
      container_.control_input_.driver_.steer_deadband_.value(), 1,
      container_.control_input_.driver_.steer_exponent_.value(), 1);

  // Manual steer
  auto target_steer = steer_x * container_.drivetrain_.max_omega();

  //   if (intaking &&
  //       container_.drivetrain_.GetReadings().pose.velocity.magnitude() >
  //           1.0_fps) {
  //     auto current_velocity_bearing =
  //         container_.drivetrain_.GetReadings().pose.velocity.angle(true);
  //     auto target_velocity_bearing = 1_rad * std::atan2(translate_x,
  //     translate_y);

  //     auto avg_bearing =
  //         (target_velocity_bearing + current_velocity_bearing) / 2.0;

  //     auto steer_error = frc846::math::CoterminalDifference(
  //         target_velocity_bearing, avg_bearing);

  //     target_steer +=
  //         container_.drivetrain_.intake_align_gain_.value() * steer_error /
  //         1_s;
  //   }

  drivetrain_target.rotation = DrivetrainRotationVelocity(target_steer);

  if (prep_align_speaker) {
    VisionReadings vision_readings = container_.vision_.GetReadings();

    ShootingCalculatorInput sc_input = {
        vision_readings.est_dist_from_speaker +
            container_.super_structure_.teleop_shooter_x_.value(),
        vision_readings.velocity_in_component * 1_fps,
        vision_readings.velocity_orth_component * 1_fps,
        container_.super_structure_.teleop_shooter_height_.value(),
        0_deg,
        container_.shooter_.shooting_exit_velocity_.value() * 1_fps};

    units::degree_t theta_adjust =
        shooting_calculator.calculate(sc_input).twist_angle;

    if (units::math::abs(vision_readings.est_dist_from_speaker_x) > 0.5_ft ||
        units::math::abs(vision_readings.est_dist_from_speaker_y) > 0.5_ft) {
      auto target_angle =
          units::math::atan2(vision_readings.est_dist_from_speaker_x,
                             vision_readings.est_dist_from_speaker_y);

      drivetrain_target.rotation =
          DrivetrainRotationPosition(target_angle + theta_adjust);
    }
  } else if (amping) {
    driver_adjust_ += container_.control_input_.GetReadings().rotation / 5.0;
    driver_adjust_ = std::min(std::max(driver_adjust_, -10.0), 10.0);

    drivetrain_target.rotation =
        DrivetrainRotationPosition(90_deg + units::degree_t(driver_adjust_));

  } else if (sourcing) {
    driver_adjust_ += container_.control_input_.GetReadings().rotation / 5.0;
    driver_adjust_ = std::min(std::max(driver_adjust_, -10.0), 10.0);

    if (!flipping_controls) {
      drivetrain_target.rotation = DrivetrainRotationPosition(
          -51.4_deg + units::degree_t(driver_adjust_));
    } else {
      drivetrain_target.rotation = DrivetrainRotationPosition(
          180_deg + 90_deg - 51.4_deg + units::degree_t(driver_adjust_));
    }
  } else {
    driver_adjust_ = 0.0;
  }

  container_.drivetrain_.SetTarget(drivetrain_target);
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }