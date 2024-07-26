#include "commands/complex/close_drive_amp_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "field.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "frc846/wpilib/time.h"

CloseDriveAmpCommand::CloseDriveAmpCommand(RobotContainer& container)
    : frc846::Loggable{"close_drive_amp_command"},
      drivetrain_(container.drivetrain_) {
  AddRequirements({&drivetrain_});
  SetName("close_drive_amp_command");
}

void CloseDriveAmpCommand::Initialize() {
  Log("Close Drive Amp Command Initialize");
}

void CloseDriveAmpCommand::Execute() {
  auto amp_point = field::points::kAmpNoFlip();

  DrivetrainTarget drivetrain_target;
  drivetrain_target.rotation = DrivetrainRotationPosition(amp_point.bearing);
  drivetrain_target.translation_reference = kField;
  drivetrain_target.control = kClosedLoop;

  auto amp_drive_dist =
      (field::points::kPreAmpNoFlip().point - amp_point.point).Magnitude();

  auto max_speed = drivetrain_.close_drive_amp_max_speed_.value();

  drivetrain_target.v_x =
      (amp_point.point.x - drivetrain_.readings().pose.point.x) /
      amp_drive_dist * max_speed;
  drivetrain_target.v_y =
      (amp_point.point.y - drivetrain_.readings().pose.point.y) /
      amp_drive_dist * max_speed;

  drivetrain_target.v_x = units::math::max(
      -max_speed, units::math::min(max_speed, drivetrain_target.v_x));
  drivetrain_target.v_y = units::math::max(
      -max_speed, units::math::min(max_speed, drivetrain_target.v_y));

  drivetrain_.SetTarget(drivetrain_target);
}

void CloseDriveAmpCommand::End(bool interrupted) {
  Log("Close Drive Amp Command Finished");
}

bool CloseDriveAmpCommand::IsFinished() {
  return (field::points::kAmpNoFlip().point - drivetrain_.readings().pose.point)
             .Magnitude() <= 2_in;
}