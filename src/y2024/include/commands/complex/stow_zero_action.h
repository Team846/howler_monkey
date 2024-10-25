#pragma once

#include <frc2/command/ParallelDeadlineGroup.h>

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class StowZeroActionCommand
    : public frc846::robot::GenericCommandGroup<
          RobotContainer, StowZeroActionCommand, frc2::SequentialCommandGroup> {
 public:
  StowZeroActionCommand(RobotContainer& container);
};
