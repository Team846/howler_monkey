#pragma once

#include "commands/follow_trajectory_command.h"
#include "frc846/other/trajectory_generator.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class FivePieceAuto
    : public frc846::robot::GenericCommandGroup<RobotContainer, FivePieceAuto,
                                                frc2::SequentialCommandGroup> {
 public:
  FivePieceAuto(RobotContainer& container, bool should_flip);
};
