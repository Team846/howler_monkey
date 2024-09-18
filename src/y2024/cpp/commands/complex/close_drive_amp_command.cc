#include "commands/complex/close_drive_amp_command.h"

#include "field.h"

CloseDriveAmpCommand::CloseDriveAmpCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, CloseDriveAmpCommand>{
          container, "close_drive_amp_command"} {
  AddRequirements({&container_.drivetrain_});
}

void CloseDriveAmpCommand::OnInit() {}

void CloseDriveAmpCommand::Periodic() {
  auto amp_point = field::points.kAmpNoFlip();

  DrivetrainTarget drivetrain_target;
  drivetrain_target.rotation = DrivetrainRotationPosition(amp_point.bearing);
  drivetrain_target.translation_reference = kField;
  drivetrain_target.control = kClosedLoop;

  auto amp_drive_dist =
      (field::points.kPreAmpNoFlip().point - amp_point.point).magnitude();

  auto max_speed = container_.drivetrain_.close_drive_amp_max_speed_.value();

  drivetrain_target.v_x = (amp_point.point[0] -
                           container_.drivetrain_.GetReadings().pose.point[0]) /
                          amp_drive_dist * max_speed;
  drivetrain_target.v_y = (amp_point.point[1] -
                           container_.drivetrain_.GetReadings().pose.point[1]) /
                          amp_drive_dist * max_speed;

  drivetrain_target.v_x = units::math::max(
      -max_speed, units::math::min(max_speed, drivetrain_target.v_x));
  drivetrain_target.v_y = units::math::max(
      -max_speed, units::math::min(max_speed, drivetrain_target.v_y));

  container_.drivetrain_.SetTarget(drivetrain_target);
}

void CloseDriveAmpCommand::OnEnd(bool interrupted) {}

bool CloseDriveAmpCommand::IsFinished() {
  return (field::points.kAmpNoFlip().point -
          container_.drivetrain_.GetReadings().pose.point)
             .magnitude() <= 2_in;
}