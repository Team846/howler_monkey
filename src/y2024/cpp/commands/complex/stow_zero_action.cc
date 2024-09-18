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
          container, "super_amp_command",
          frc2::SequentialCommandGroup{
              frc2::ParallelDeadlineGroup{
                  frc2::WaitUntilCommand{[&] {
                    return container_.pivot_.WithinTolerance(
                        container_.super_structure_.getStowSetpoint().pivot);
                  }},
                  StowCommand{container}},
              WristZeroCommand{container},
              StowCommand{container} /* Otherwise, this command will be
                                                 rescheduled -> infinite loop */
          }} {}