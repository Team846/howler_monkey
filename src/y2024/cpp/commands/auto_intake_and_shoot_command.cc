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
#include "commands/speaker_align_command.h"
#include "commands/shoot_command.h"
#include "subsystems/field.h"


frc2::SequentialCommandGroup AutoIntakeAndShootCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point, bool flip) {
    
  units::foot_t distance = (field::points::kSpeaker(flip) - shoot_point.pos.point).Magnitude();
        
  return frc2::SequentialCommandGroup{
        DeployIntakeCommand{container},
        FollowTrajectoryCommand{ container, {intake_point}},
        frc2::WaitCommand(0.3_s),
        PrepareShootCommand{container, distance},
        FollowTrajectoryCommand{ container, {shoot_point}},
        SpeakerAlignCommand{container, shoot_point.pos.point},
        frc2::WaitCommand(0.3_s),
        ShootCommand{container},
        frc2::WaitCommand(0.5_s),
        StowCommand{container}
  };
}