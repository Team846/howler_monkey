#ifndef y2024_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_
#define y2024_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frcLib846/math.h"
#include "frcLib846/trajectory_generator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"

class FollowTrajectoryCommand
    : public frc2::CommandHelper<frc2::Command, FollowTrajectoryCommand>,
      public frcLib846::Loggable {
 public:
  FollowTrajectoryCommand(RobotContainer& container,
                          std::vector<frcLib846::InputWaypoint> input_points);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;

  std::vector<frcLib846::InputWaypoint> input_points_;
  frcLib846::Trajectory trajectory_;

  unsigned int target_idx_ = 1;
  bool is_done_ = false;
  frcLib846::Vector2D<units::foot_t> current_extrapolated_point_;

  units::second_t start_time_;

  static bool HasCrossedWaypoint(frcLib846::Waypoint current_waypoint,
                                 frcLib846::Waypoint prev_waypoint,
                                 frcLib846::Vector2D<units::foot_t> pos,
                                 frcLib846::Vector2D<units::foot_t> test_target);
};

#endif  // y2024_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_