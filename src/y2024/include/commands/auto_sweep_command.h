#ifndef Y2024_COMMANDS_AUTO_SWEEP_COMMAND_H_
#define Y2024_COMMANDS_AUTO_SWEEP_COMMAND_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/other/trajectory_generator.h"
#include "subsystems/robot_container.h"
#include "frc846/util/math.h"

frc2::SequentialCommandGroup AutoSweepCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point, bool flip, double shoot_time);

//TODO add bool flip command to make it easier

#endif  // Y2024_COMMANDS_AUTO_SWEEP_H_
