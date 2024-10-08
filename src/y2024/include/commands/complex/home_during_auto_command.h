#pragma once

#include <frc2/command/ParallelDeadlineGroup.h>

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class HomeDuringAutoCommand
    : public frc846::robot::GenericCommandGroup<
          RobotContainer, HomeDuringAutoCommand, frc2::SequentialCommandGroup> {
 public:
  HomeDuringAutoCommand(RobotContainer& container);
};
