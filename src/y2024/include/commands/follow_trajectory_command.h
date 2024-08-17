#pragma once

#include "frc846/other/trajectory_generator.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class FollowTrajectoryCommand
    : public frc846::robot::GenericCommand<RobotContainer,
                                           FollowTrajectoryCommand> {
 public:
  FollowTrajectoryCommand(RobotContainer& container,
                          std::vector<frc846::InputWaypoint> input_points);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  std::vector<frc846::InputWaypoint> input_points_;
  frc846::Trajectory trajectory_;

  unsigned int target_idx_ = 1;
  bool is_done_ = false;
  frc846::util::Vector2D<units::foot_t> current_extrapolated_point_;

  units::second_t start_time_;

  static bool HasCrossedWaypoint(
      frc846::Waypoint current_waypoint, frc846::Waypoint prev_waypoint,
      frc846::util::Vector2D<units::foot_t> pos,
      frc846::util::Vector2D<units::foot_t> test_target);
};
