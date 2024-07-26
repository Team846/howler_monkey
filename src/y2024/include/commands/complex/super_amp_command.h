#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/other/trajectory_generator.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

frc2::CommandPtr SuperAmpCommand(RobotContainer& container, bool is_red_side);
