#include "commands/complex/home_during_auto_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/basic/wrist_zero_command.h"
#include "commands/teleop/stow_command.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"

HomeDuringAutoCommand::HomeDuringAutoCommand(RobotContainer& container)
    : frc846::robot::GenericCommandGroup<RobotContainer, HomeDuringAutoCommand,
                                         frc2::SequentialCommandGroup>{
          container, "home_during_auto_command",
          frc2::SequentialCommandGroup{
              frc2::ParallelDeadlineGroup{
                  frc2::WaitUntilCommand{[&] {
                    return container.pivot_.WithinTolerance(
                        container.super_structure_.getStowSetpoint().pivot);
                  }},
                  StowCommand{container}},
              WristZeroCommand{container},
          }} {}