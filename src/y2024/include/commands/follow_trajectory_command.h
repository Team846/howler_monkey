#ifndef y2024_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_
#define y2024_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "frc846/other/trajectory_generator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"

class FollowTrajectoryCommand
    : public frc2::CommandHelper<frc2::Command, FollowTrajectoryCommand>,
      public frc846::Loggable {
 public:
  FollowTrajectoryCommand(RobotContainer& container,
                          std::vector<frc846::InputWaypoint> input_points);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;
  
  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;

  std::vector<frc846::InputWaypoint> input_points_;
  frc846::Trajectory trajectory_;

  unsigned int target_idx_ = 1;
  bool is_done_ = false;
  frc846::util::Vector2D<units::foot_t> current_extrapolated_point_;

  units::second_t start_time_;

  static bool HasCrossedWaypoint(frc846::Waypoint current_waypoint,
                                 frc846::Waypoint prev_waypoint,
                                 frc846::util::Vector2D<units::foot_t> pos,
                                 frc846::util::Vector2D<units::foot_t> test_target);
};

#endif  // y2024_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_