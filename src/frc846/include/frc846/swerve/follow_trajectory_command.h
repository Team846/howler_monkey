#pragma once

#include "frc846/robot/GenericCommand.h"
#include "frc846/swerve/waypt_traversal_calculator.h"
#include "subsystems/robot_container.h"

namespace frc846::swerve {

class FollowTrajectoryCommand
    : public frc846::robot::GenericCommand<RobotContainer,
                                           FollowTrajectoryCommand> {
 public:
  FollowTrajectoryCommand(RobotContainer& container,
                          std::vector<frc846::Waypoint> input_points);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  std::vector<frc846::Waypoint> input_points_;
  unsigned int target_idx_;

  bool is_done_ = false;

  units::second_t start_time_;

  frc846::swerve::WayptTraversalCalculator wtcalculator{};
};

};  // namespace frc846::swerve