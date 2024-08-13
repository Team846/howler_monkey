#include "commands/complex/super_amp_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/basic/amp_command.h"
#include "commands/basic/await_piece_state_command.h"
#include "commands/basic/eject_command.h"
#include "commands/basic/pull_command.h"
#include "commands/complex/close_drive_amp_command.h"
#include "commands/follow_trajectory_command.h"
#include "field.h"
#include "frc2/command/WaitCommand.h"

frc2::CommandPtr SuperAmpCommand(RobotContainer& container, bool is_red_side) {
  return //frc2::ParallelDeadlineGroup{
      //frc2::SequentialCommandGroup{
          FollowTrajectoryCommand{container,
                                  {{{field::points::kPreAmpNoFlip()}, 0_fps}}}.ToPtr(); //,
    //       CloseDriveAmpCommand{container},
    //       frc2::ParallelDeadlineGroup{
    //           frc2::SequentialCommandGroup{
    //               AwaitPieceStateCommand{container, false},
    //               frc2::WaitCommand{0.5_s},
    //           },
    //           EjectCommand{container}},
    //       FollowTrajectoryCommand{container,
    //                               {{{field::points::kPreAmpNoFlip()}, 0_fps}}}},
    //   AmpCommand{container},
    //   frc2::ParallelDeadlineGroup{frc2::WaitCommand{1_s},
    //                               PullCommand{container}}}
    //   .ToPtr();
}