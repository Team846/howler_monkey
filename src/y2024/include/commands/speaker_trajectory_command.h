#ifndef y2024_COMMANDS_SPEAKER_TRAJECTORY_COMMAND_H_
#define y2024_COMMANDS_SPEAKER_TRAJECTORY_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/other/trajectory_generator.h"
#include "frc846/util/math.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/robot_container.h"

class SpeakerTrajectoryCommand
    : public frc2::CommandHelper<frc2::Command, SpeakerTrajectoryCommand>,
      public frc846::Loggable {
 public:
  SpeakerTrajectoryCommand(RobotContainer& container,
                           std::vector<frc846::InputWaypoint> input_points);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;
  VisionSubsystem& vision_;

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

#endif  // y2024_COMMANDS_SPEAKER_TRAJECTORY_COMMAND_H_