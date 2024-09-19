#pragma once

#include "frc846/robot/GenericCommand.h"
#include "frc846/swerve/follow_trajectory_command.h"
#include "subsystems/robot_container.h"

class OnePieceAuto
    : public frc846::robot::GenericCommandGroup<RobotContainer, OnePieceAuto,
                                                frc2::SequentialCommandGroup> {
 public:
  OnePieceAuto(RobotContainer& container, units::degree_t start_angle,
               std::string num);
};
