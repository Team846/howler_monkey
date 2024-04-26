#ifndef FOUR_PIECE_AUTO_H_
#define FOUR_PIECE_AUTO_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/other/trajectory_generator.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class FourPieceAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, FourPieceAuto> {
 public:
  FourPieceAuto(RobotContainer& container, bool should_flip_);

  bool should_flip_;
};

#endif  // y2024_COMMANDS_FOUR_PIECE_AUTO_H_