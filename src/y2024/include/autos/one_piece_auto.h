#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/other/trajectory_generator.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class OnePieceAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, OnePieceAuto> {
 public:
  OnePieceAuto(RobotContainer& container, units::degree_t start_angle, std::string num);
};
