#include "commands/teleop/drive_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/base/loggable.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/swerve_module.h"

DriveCommand::DriveCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, DriveCommand>{
          container, "drive_command"} {
  AddRequirements({&container_.drivetrain_});
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
  bool sourcing = container_.control_input_.GetReadings().running_source;

  bool targeting_note = container_.control_input_.GetReadings().targeting_note;

  bool flipping_controls =
      !frc846::util::ShareTables::GetBoolean("is_red_side");

  // -----TRANSLATION CONTROL-----

  double translate_x = frc846::util::HorizontalDeadband(
      container_.control_input_.GetReadings().translate_x,
      container_.control_input_.driver_.translation_deadband_.value(), 1,
      container_.control_input_.driver_.translation_exponent_.value(), 1);
  double translate_y = frc846::util::HorizontalDeadband(
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
  double steer_x = frc846::util::HorizontalDeadband(
      container_.control_input_.GetReadings().rotation,
      container_.control_input_.driver_.steer_deadband_.value(), 1,
      container_.control_input_.driver_.steer_exponent_.value(), 1);

  if (steer_x !=
      0) {  // always override any autonomous routines with manual control
    // Manual steer
    auto target = steer_x * container_.drivetrain_.max_omega();

    drivetrain_target.rotation = DrivetrainRotationVelocity(target);
  } else {
    // Hold position
    drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  }

  if (prep_align_speaker) {
    VisionReadings vision_readings = container_.vision_.GetReadings();
    double shooting_dist = vision_readings.est_dist_from_speaker.to<double>();

    shooting_dist =
        shooting_dist +
        container_.super_structure_.teleop_shooter_x_.value().to<double>() /
            12.0;

    units::degree_t theta_adjust =
        shooting_calculator
            .calculateLaunchAngles(
                container_.shooter_.shooting_exit_velocity_.value(),
                shooting_dist, vision_readings.velocity_in_component,
                vision_readings.velocity_orth_component,
                container_.super_structure_.teleop_shooter_height_.value()
                    .to<double>())
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
  } else if (targeting_note && container_.gpd_.GetReadings().note_detected) {
    // Turn towards the note
    frc846::util::Vector2D rel_note_pos =
        container_.gpd_.GetReadings().closest_note -
        container_.drivetrain_.GetReadings().pose.point;
    drivetrain_target.rotation = DrivetrainRotationPosition(
        rel_note_pos.Bearing());  // field centric, negative because switch
                                  // between ccw to cs
  } else {
    driver_adjust_ = 0.0;
  }
  container_.drivetrain_.SetTarget(drivetrain_target);
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }