#include "commands/speaker_trajectory_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/other/trajectory_generator.h"
#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"
#include "subsystems/hardware/swerve_module.h"

SpeakerTrajectoryCommand::SpeakerTrajectoryCommand(
    RobotContainer& container, std::vector<frc846::InputWaypoint> input_points)
    : frc846::Loggable{"speaker_trajectory_command"},
      drivetrain_(container.drivetrain_),
      vision_(container.vision_),
      input_points_(input_points) {
  AddRequirements({&drivetrain_});
  SetName("speaker_trajectory_command");
}

void SpeakerTrajectoryCommand::Initialize() {
  Log("Starting Trajectory");
  for (frc846::InputWaypoint i : input_points_) {
    Log("points x{} y{} bearing {}", i.pos.point.x, i.pos.point.y,
        i.pos.bearing);
  }
  Log("initial pose x{}, y{}, bearing {}", drivetrain_.readings().pose.point.x,
      drivetrain_.readings().pose.point.y, drivetrain_.readings().pose.bearing);
  target_idx_ = 1;
  is_done_ = false;

  start_time_ = frc846::wpilib::CurrentFPGATime();

  auto points = input_points_;
  points.insert(points.begin(), 1, {drivetrain_.readings().pose, 0_fps});

  trajectory_ =
      frc846::GenerateTrajectory(points, drivetrain_.auto_max_speed_.value(),
                                 drivetrain_.max_acceleration_.value(),
                                 drivetrain_.max_deceleration_.value());

  if (trajectory_.size() < 2) {
    Warn("trajectory size ({}) is less than 2 - ending!", trajectory_.size());
    is_done_ = true;
  } else {
    current_extrapolated_point_ = trajectory_[1].pos.point.Extrapolate(
        trajectory_[0].pos.point, drivetrain_.extrapolation_distance_.value());
  }
}

void SpeakerTrajectoryCommand::Execute() {
  VisionReadings vision_readings = vision_.readings();

  // Just in case trajectory size was < 2
  if (is_done_) {
    return;
  }

  if (HasCrossedWaypoint(trajectory_[target_idx_], trajectory_[target_idx_ - 1],
                         drivetrain_.readings().pose.point,
                         current_extrapolated_point_)) {
    target_idx_++;
    Log("Cross waypoint - now on {}/{}", target_idx_ + 1, trajectory_.size());

    if (target_idx_ == trajectory_.size()) {
      Log("Done!");
      is_done_ = true;
      return;
    }

    current_extrapolated_point_ =
        trajectory_[target_idx_].pos.point.Extrapolate(
            trajectory_[target_idx_ - 1].pos.point,
            drivetrain_.extrapolation_distance_.value());
  }

  auto delta_pos =
      current_extrapolated_point_ - drivetrain_.readings().pose.point;
  auto direction = units::math::atan2(delta_pos.y, delta_pos.x);

  DrivetrainTarget target;
  target.v_x = trajectory_[target_idx_].v * units::math::cos(direction);
  target.v_y = trajectory_[target_idx_].v * units::math::sin(direction);
  target.translation_reference = DrivetrainTranslationReference::kField;
  target.rotation = DrivetrainRotationPosition(
      units::math::atan2(vision_readings.est_dist_from_speaker_x,
                         vision_readings.est_dist_from_speaker_y));
  target.control = kClosedLoop;

  drivetrain_.SetTarget(target);
}

void SpeakerTrajectoryCommand::End(bool interrupted) {
  (void)interrupted;
  drivetrain_.SetTargetZero();
}

bool SpeakerTrajectoryCommand::IsFinished() {
  if (frc::RobotBase::IsSimulation() &&
      frc846::wpilib::CurrentFPGATime() - start_time_ > 3_s) {
    Log("Ending after 3s!");
    return true;
  }

  return is_done_;
}

// https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
bool SpeakerTrajectoryCommand::HasCrossedWaypoint(
    frc846::Waypoint current_waypoint, frc846::Waypoint prev_waypoint,
    frc846::util::Vector2D<units::foot_t> pos,
    frc846::util::Vector2D<units::foot_t> extrapolated_point) {
  // fmt::print(
  //     "\ncurrent_waypoint x {}, current_waypoint y {}, prev_waypoint x {} y "
  //     "{}, pos x{} y{}, extrap x{}, y{}\n",
  //     current_waypoint.pos.point.x, current_waypoint.pos.point.y,
  //     prev_waypoint.pos.point.x, prev_waypoint.pos.point.y, pos.x, pos.y,
  //     extrapolated_point.x, extrapolated_point.y);

  auto d = [](frc846::util::Vector2D<units::foot_t> target,
              frc846::util::Vector2D<units::foot_t> p1,
              frc846::util::Vector2D<units::foot_t> p2) {
    double x =
        ((target.x - p1.x) * (p2.y - p1.y) - (target.y - p1.y) * (p2.x - p1.x))
            .to<double>();
    if (x > 0) {
      return 1;
    } else if (x < 0) {
      return -1;
    }
    return 0;
  };

  auto delta_y = current_waypoint.pos.point.y - prev_waypoint.pos.point.y;
  auto delta_x = current_waypoint.pos.point.x - prev_waypoint.pos.point.x;
  auto theta = units::math::atan(-delta_x / delta_y);
  double cos_theta = units::math::cos(theta);
  double sin_theta = units::math::sin(theta);

  auto p1 = current_waypoint.pos.point - frc846::util::Vector2D<units::foot_t>{
                                             1_ft * cos_theta,
                                             1_ft * sin_theta,
                                         };
  auto p2 = current_waypoint.pos.point + frc846::util::Vector2D<units::foot_t>{
                                             1_ft * cos_theta,
                                             1_ft * sin_theta,
                                         };

  return d(pos, p1, p2) == d(extrapolated_point, p1, p2);
}