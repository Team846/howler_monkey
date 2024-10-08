#include "frc846/swerve/follow_trajectory_command.h"

namespace frc846::swerve {

FollowTrajectoryCommand::FollowTrajectoryCommand(
    RobotContainer& container, std::vector<frc846::Waypoint> input_points,
    int mirror)
    : frc846::robot::GenericCommand<
          RobotContainer, FollowTrajectoryCommand>{container,
                                                   "follow_trajectory_command"},
      input_points_(input_points),
      mirror_(mirror) {
  AddRequirements({&container_.drivetrain_});
}

void FollowTrajectoryCommand::OnInit() {
  wtcalculator.setConstants({container_.drivetrain_.auto_max_speed_.value(),
                             container_.drivetrain_.max_acceleration_.value(),
                             container_.drivetrain_.max_deceleration_.value(),
                             container_.drivetrain_.max_speed_.value(),
                             80_fps_sq, 80_fps_sq, 3_in, 20_ms});

  Log("Starting Trajectory");
  Log("Initial pose x{}, y{}, Bearing {}",
      container_.drivetrain_.GetReadings().pose.point[0],
      container_.drivetrain_.GetReadings().pose.point[1],
      container_.drivetrain_.GetReadings().pose.bearing);

  start_time_ = frc846::wpilib::CurrentFPGATime();

  auto vision_readings = container_.vision_.GetReadings();
  auto dtpose = container_.drivetrain_.GetReadings().pose;
  auto pose =
      (container_.vision_.using_vision_autos_.value()

           ? frc846::math::FieldPoint{{vision_readings.x_pos,
                                       vision_readings.y_pos},
                                      dtpose.bearing,
                                      {dtpose.velocity[0], dtpose.velocity[1]}}
           : dtpose);

  path_points_.clear();
  path_points_.push_back(pose);
  for (auto x : input_points_) {
    path_points_.push_back(
        mirror_ == 0 ? x : (mirror_ == 1 ? x.mirror() : x.mirrorOnlyY()));
  }
  for (frc846::Waypoint i : path_points_) {
    Log("Points x{} y{} Bearing {}", i.point[0], i.point[1], i.bearing);
  }

  is_done_ = false;

  target_idx_ = 1;

  if (path_points_.size() < 2) {
    Error("Trajectory size ({}) is less than 2 - ending!", path_points_.size());
    is_done_ = true;
  }
}

void FollowTrajectoryCommand::Periodic() {
  // Just in case trajectory size was < 2
  if (is_done_) {
    return;
  }

  auto vision_readings = container_.vision_.GetReadings();
  auto dtpose = container_.drivetrain_.GetReadings().pose;
  auto pose =
      (container_.vision_.using_vision_autos_.value()

           ? frc846::math::FieldPoint{{vision_readings.x_pos,
                                       vision_readings.y_pos},
                                      dtpose.bearing,
                                      {dtpose.velocity[0], dtpose.velocity[1]}}
           : dtpose);

  frc846::swerve::WTCInput lpcinput{
      path_points_[target_idx_ - 1].point,
      path_points_[target_idx_].point,
      pose.point,
      pose.bearing,
      path_points_[target_idx_].bearing,
      pose.velocity.magnitude(),
      path_points_[target_idx_].velocity.magnitude()};

  frc846::swerve::WTCOutput lpcoutput = wtcalculator.calculate(lpcinput);

  if (lpcoutput.crossed_waypt) {
    Log("Initial x{} y{}.", path_points_[target_idx_ - 1].point[0],
        path_points_[target_idx_ - 1].point[1]);
    Log("Target x{} y{}.", path_points_[target_idx_].point[0],
        path_points_[target_idx_].point[1]);
    target_idx_++;
    Log("Cross waypoint - now on {}/{}", target_idx_ + 1, path_points_.size());
    Log("Position x{} y{}", pose.point[0], pose.point[1]);

    if (target_idx_ == path_points_.size()) {
      Log("Done!");
      is_done_ = true;
      return;
    }
  }

  DrivetrainTarget target;
  target.v_x = lpcoutput.target_vel[0];
  target.v_y = lpcoutput.target_vel[1];
  target.translation_reference = DrivetrainTranslationReference::kField;
  target.rotation = DrivetrainRotationVelocity(lpcoutput.rotational_vel);
  target.control = kOpenLoop;

  container_.drivetrain_.SetTarget(target);
}

void FollowTrajectoryCommand::OnEnd(bool interrupted) {
  (void)interrupted;
  container_.drivetrain_.SetTargetZero();
}

bool FollowTrajectoryCommand::IsFinished() { return is_done_; }

};  // namespace frc846::swerve