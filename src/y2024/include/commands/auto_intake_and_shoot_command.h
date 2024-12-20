#pragma once

#include "frc846/robot/GenericCommand.h"
#include "frc846/swerve/follow_trajectory_command.h"
#include "subsystems/robot_container.h"

class AutoIntakeAndShootCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
                                                AutoIntakeAndShootCommand,
                                                frc2::SequentialCommandGroup> {
 public:
  AutoIntakeAndShootCommand(RobotContainer& container,
                            std::vector<frc846::Waypoint> intake_path,
                            std::vector<frc846::Waypoint> shoot_path);
};
