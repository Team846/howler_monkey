#ifndef Y2024_COMMANDS_AUTO_INTAKE_SHOOT_STILL_COMMAND_H_
#define Y2024_COMMANDS_AUTO_INTAKE_SHOOT_STILL_COMMAND_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/other/trajectory_generator.h"
#include "subsystems/robot_container.h"
#include "frc846/util/math.h"
#include "subsystems/drivetrain.h"

frc2::SequentialCommandGroup AutoIntakeShootStillCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point, bool flip);

#endif  // Y2024_COMMANDS_AUTO_INTAKE_SHOOT_STILL_H_
