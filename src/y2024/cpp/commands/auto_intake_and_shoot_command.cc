#include "commands/auto_intake_and_shoot_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc2/command/WaitCommand.h"
#include "commands/follow_trajectory_command.h"
#include "commands/deploy_intake_command.h"
#include "commands/prepare_shoot_command.h"
#include "commands/stow_command.h"
#include "subsystems/field.h"


frc2::SequentialCommandGroup AutoIntakeAndShootCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point) {
    
    //Subtract a bit from speaker values to make shooter overshoot by a bit
    // units::inch_t distance {(sqrt(pow(shoot_point.pos.point.x.value() - 
    //                             (field::points::kSpeaker().x.value() - 5), 2.0f) + 
    //                             pow(shoot_point.pos.point.y.value() - 
    //                             (field::points::kSpeaker().y.value() - 5), 2.0f)))};
        
  return frc2::SequentialCommandGroup{
        //Grab and Intake 
        
        // deployIntakeCommand(container, true), 
        DeployIntakeCommand{container},
        FollowTrajectoryCommand{ container, {intake_point}},
        frc2::WaitCommand(0.1_s),

        StowCommand{container},
        //TODO add intake->shoot integration. don't need to stow first
        frc2::ParallelDeadlineGroup{
            PrepareShootCommand{container},
            FollowTrajectoryCommand{ container, {shoot_point}}
        },
        
        frc2::WaitCommand(0.1_s),
        // ShootCommand(container, true), 
        frc2::WaitCommand(0.1_s),
        StowCommand{container}

        //TODO add option to stow after or intake after? for convenience
  };
}