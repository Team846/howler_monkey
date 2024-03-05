#ifndef ONE_PIECE_AUTO_H_
#define ONE_PIECE_AUTO_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/util/math.h"
#include "frc846/other/trajectory_generator.h"
#include "subsystems/robot_container.h"

class OnePieceAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 OnePieceAuto> {
 public:
  OnePieceAuto(RobotContainer& container, bool should_flip_);

  units::foot_t start_distance;

  bool should_flip_;
};

#endif  // y2024_COMMANDS_ONE_PIECE_AUTO_H_