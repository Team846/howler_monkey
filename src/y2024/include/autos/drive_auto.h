#ifndef y2024_COMMANDS_DRIVE_AUTO_H_
#define y2024_COMMANDS_DRIVE_AUTO_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frcLib846/math.h"
#include "frcLib846/trajectory_generator.h"
#include "subsystems/robot_container.h"

class DriveAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 DriveAuto> {
 public:
  DriveAuto(RobotContainer& container, bool should_flip_);

  bool should_flip_;
};

#endif  // y2024_COMMANDS_DRIVE_AUTO_H_