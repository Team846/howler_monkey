#include "commands/follow_trajectory_command.h"

FollowTrajectoryCommand::FollowTrajectoryCommand(
    RobotContainer& container, std::vector<frc846::Waypoint> input_points)
    : frc846::robot::GenericCommand<
          RobotContainer, FollowTrajectoryCommand>{container,
                                                   "follow_trajectory_command"},
      input_points_(input_points) {
  AddRequirements({&container_.drivetrain_});
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
  target_idx_ = 1;
  is_done_ = false;

  start_time_ = frc846::wpilib::CurrentFPGATime();

  auto points = input_points_;
  points.insert(points.begin(), 1, {container_.drivetrain_.GetReadings().pose});

  trajectory_ = frc846::GenerateTrajectory(
      points, container_.drivetrain_.auto_max_speed_.value(),
      container_.drivetrain_.max_acceleration_.value(),
      container_.drivetrain_.max_deceleration_.value());

  if (trajectory_.size() < 4) {
    Error("trajectory size ({}) is less than 4 - ending!", trajectory_.size());
    is_done_ = true;
  } else {
    double extrapolation_distance =
        container_.drivetrain_.extrapolation_distance_.value() / 1_ft;
    current_extrapolated_point_ = trajectory_[1].point - trajectory_[0].point;
    current_extrapolated_point_ =
        current_extrapolated_point_.unit() * extrapolation_distance;
  }
}

void FollowTrajectoryCommand::Periodic() {
  // Just in case trajectory size was < 2
  if (is_done_) {
    return;
  }

  if (HasCrossedWaypoint(trajectory_[target_idx_], trajectory_[target_idx_ - 1],
                         container_.drivetrain_.GetReadings().pose.point,
                         current_extrapolated_point_)) {
    target_idx_++;
    Log("Cross waypoint - now on {}/{}", target_idx_ + 1, trajectory_.size());

    if (target_idx_ == trajectory_.size()) {
      Log("Done!");
      is_done_ = true;
      return;
    }

    double extrapolation_distance =
        container_.drivetrain_.extrapolation_distance_.value() / 1_ft;
    current_extrapolated_point_ =
        trajectory_[target_idx_].point - trajectory_[target_idx_ - 1].point;
    current_extrapolated_point_ =
        current_extrapolated_point_.unit() * extrapolation_distance;
  }

  auto delta_pos = current_extrapolated_point_ -
                   container_.drivetrain_.GetReadings().pose.point;
  auto direction = units::math::atan2(delta_pos[1], delta_pos[0]);

  DrivetrainTarget target;
  target.v_x = trajectory_[target_idx_].velocity.magnitude() *
               units::math::cos(direction);
  target.v_y = trajectory_[target_idx_].velocity.magnitude() *
               units::math::sin(direction);
  target.translation_reference = DrivetrainTranslationReference::kField;
  target.rotation =
      DrivetrainRotationPosition(trajectory_[target_idx_].bearing);
  target.control = kClosedLoop;

  container_.drivetrain_.SetTarget(target);
}

void FollowTrajectoryCommand::OnEnd(bool interrupted) {
  (void)interrupted;
  container_.drivetrain_.SetTargetZero();
}

bool FollowTrajectoryCommand::IsFinished() { return is_done_; }

// https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
bool FollowTrajectoryCommand::HasCrossedWaypoint(
    frc846::Waypoint current_waypoint, frc846::Waypoint prev_waypoint,
    frc846::math::VectorND<units::foot_t, 2> pos,
    frc846::math::VectorND<units::foot_t, 2> extrapolated_point) {
  // fmt::print(
  //     "\ncurrent_waypoint x {}, current_waypoint y {}, prev_waypoint x {} y "
  //     "{}, pos x{} y{}, extrap x{}, y{}\n",
  //     current_waypoint.point.x, current_waypoint.point.y,
  //     prev_waypoint.point.x, prev_waypoint.point.y, pos.x, pos.y,
  //     extrapolated_point.x, extrapolated_point.y);

  auto d = [](frc846::math::VectorND<units::foot_t, 2> target,
              frc846::math::VectorND<units::foot_t, 2> p1,
              frc846::math::VectorND<units::foot_t, 2> p2) {
    double x = ((target[0] - p1[0]) * (p2[1] - p1[1]) -
                (target[1] - p1[1]) * (p2[0] - p1[0]))
                   .to<double>();
    if (x > 0) {
      return 1;
    } else if (x < 0) {
      return -1;
    }
    return 0;
  };

  auto delta_y = current_waypoint.point[1] - prev_waypoint.point[1];
  auto delta_x = current_waypoint.point[0] - prev_waypoint.point[0];
  auto theta = units::math::atan(-delta_x / delta_y);
  double cos_theta = units::math::cos(theta);
  double sin_theta = units::math::sin(theta);

  auto p1 = current_waypoint.point - frc846::math::VectorND<units::foot_t, 2>{
                                         1_ft * cos_theta,
                                         1_ft * sin_theta,
                                     };
  auto p2 = current_waypoint.point + frc846::math::VectorND<units::foot_t, 2>{
                                         1_ft * cos_theta,
                                         1_ft * sin_theta,
                                     };

  return d(pos, p1, p2) == d(extrapolated_point, p1, p2);
}