#pragma once

#include "commands/follow_trajectory_command.h"
#include "frc846/other/trajectory_generator.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class AutoIntakeAndShootCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
                                                AutoIntakeAndShootCommand,
                                                frc2::SequentialCommandGroup> {
 public:
  AutoIntakeAndShootCommand(RobotContainer& container,
                            frc846::InputWaypoint intake_point,
                            frc846::InputWaypoint mid_point,
                            frc846::InputWaypoint shoot_point);
};
