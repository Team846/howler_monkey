#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/other/trajectory_generator.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class FivePieceAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, FivePieceAuto> {
 public:
  FivePieceAuto(RobotContainer& container, bool should_flip_);

  bool should_flip_;
};