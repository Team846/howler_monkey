#pragma once

#include "frc846/robot/GenericCommand.h"
#include "frc846/swerve/follow_trajectory_command.h"
#include "subsystems/robot_container.h"

class DriveAuto
    : public frc846::robot::GenericCommandGroup<RobotContainer, DriveAuto,
                                                frc2::SequentialCommandGroup> {
 public:
  DriveAuto(RobotContainer& container);
};
