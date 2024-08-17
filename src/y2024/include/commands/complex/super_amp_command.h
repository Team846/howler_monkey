#pragma once

#include <frc2/command/ParallelDeadlineGroup.h>

#include "commands/follow_trajectory_command.h"
#include "frc846/other/trajectory_generator.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class SuperAmpCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer, SuperAmpCommand,
                                                frc2::SequentialCommandGroup> {
 public:
  SuperAmpCommand(RobotContainer& container, bool is_red_side);
};
