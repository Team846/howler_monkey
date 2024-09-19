#include "frc846/swerve/follow_trajectory_command.h"

namespace frc846::swerve {

FollowTrajectoryCommand::FollowTrajectoryCommand(
    RobotContainer& container, std::vector<frc846::Waypoint> input_points)
    : frc846::robot::GenericCommand<
          RobotContainer, FollowTrajectoryCommand>{container,
                                                   "follow_trajectory_command"},
      input_points_(input_points) {
  AddRequirements({&container_.drivetrain_});

  wtcalculator.setConstants({container_.drivetrain_.max_speed_.value(),
                             container_.drivetrain_.max_acceleration_.value(),
                             2_in, 20_ms});
}

void FollowTrajectoryCommand::OnInit() {
  Log("Starting Trajectory");
  for (frc846::Waypoint i : input_points_) {
    Log("points x{} y{} bearing {}", i.point[0], i.point[1], i.bearing);
  }
  Log("initial pose x{}, y{}, bearing {}",
      container_.drivetrain_.GetReadings().pose.point[0],
      container_.drivetrain_.GetReadings().pose.point[1],
      container_.drivetrain_.GetReadings().pose.bearing);
  is_done_ = false;

  start_time_ = frc846::wpilib::CurrentFPGATime();

  input_points_.insert(input_points_.begin(), 1,
                       {container_.drivetrain_.GetReadings().pose});

  target_idx_ = 1;

  if (input_points_.size() < 2) {
    Error("trajectory size ({}) is less than 2 - ending!",
          input_points_.size());
    is_done_ = true;
  }
}

void FollowTrajectoryCommand::Periodic() {
  // Just in case trajectory size was < 2
  if (is_done_) {
    return;
  }

  frc846::swerve::WTCInput lpcinput{
      input_points_[target_idx_ - 1].point,
      input_points_[target_idx_].point,
      container_.drivetrain_.GetReadings().pose.point,
      container_.drivetrain_.GetReadings().pose.bearing,
      input_points_[target_idx_].bearing,
      container_.drivetrain_.GetReadings().pose.velocity.magnitude(),
      input_points_[target_idx_].velocity.magnitude()};

  frc846::swerve::WTCOutput lpcoutput = wtcalculator.calculate(lpcinput);

  if (lpcoutput.crossed_waypt) {
    target_idx_++;
    Log("Cross waypoint - now on {}/{}", target_idx_ + 1, input_points_.size());

    if (target_idx_ == input_points_.size()) {
      Log("Done!");
      is_done_ = true;
      return;
    }
  }

  DrivetrainTarget target;
  target.v_x = lpcoutput.target_vel[0];
  target.v_y = lpcoutput.target_vel[1];
  target.translation_reference = DrivetrainTranslationReference::kField;
  target.rotation = DrivetrainRotationPosition(lpcoutput.bearing);
  target.control = kClosedLoop;

  container_.drivetrain_.SetTarget(target);
}

void FollowTrajectoryCommand::OnEnd(bool interrupted) {
  (void)interrupted;
  container_.drivetrain_.SetTargetZero();
}

bool FollowTrajectoryCommand::IsFinished() { return is_done_; }

};  // namespace frc846::swerve