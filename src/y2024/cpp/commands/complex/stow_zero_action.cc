#include "commands/complex/stow_zero_action.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/basic/wrist_zero_command.h"
#include "commands/teleop/stow_command.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"

StowZeroActionCommand::StowZeroActionCommand(RobotContainer& container)
    : frc846::robot::GenericCommandGroup<RobotContainer, StowZeroActionCommand,
                                         frc2::SequentialCommandGroup>{
          container, "stow_zero_action_command",
          frc2::SequentialCommandGroup{
              frc2::ParallelDeadlineGroup{
                  frc2::WaitCommand{1.65_s},
                  StowCommand{container}},
              WristZeroCommand{container},
              StowCommand{container} /* Otherwise, this command will be
                                                 rescheduled -> infinite loop */
          }} {}