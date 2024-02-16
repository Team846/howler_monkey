#ifndef Y2024_COMMANDS_AUTO_INTAKE_AND_SHOOT_COMMAND_H_
#define Y2024_COMMANDS_AUTO_INTAKE_AND_SHOOT_COMMAND_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/other/trajectory_generator.h"
#include "subsystems/robot_container.h"

frc2::SequentialCommandGroup AutoIntakeAndShootCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point);

//TODO add bool flip command to make it easier

#endif  // Y2024_COMMANDS_AUTO_INTAKE_AND_SHOOT_H_
