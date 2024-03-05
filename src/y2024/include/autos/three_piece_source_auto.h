#ifndef THREE_PIECE_SOURCE_AUTO_H_
#define THREE_PIECE_SOURCE_AUTO_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/util/math.h"
#include "frc846/other/trajectory_generator.h"
#include "subsystems/robot_container.h"

class ThreePieceSourceAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ThreePieceSourceAuto> {
 public:
  ThreePieceSourceAuto(RobotContainer& container, bool should_flip_);

  units::foot_t first_distance;

  bool should_flip_;
};

#endif  // y2024_COMMANDS_THREE_PIECE_AUTO_H_