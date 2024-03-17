#include "commands/auto_intake_shoot_still_command.h"
#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

#include "frc2/command/WaitCommand.h"
#include "commands/follow_trajectory_command.h"
#include "commands/deploy_intake_command.h"
#include "commands/prepare_short_shoot_command.h"
#include "commands/prepare_far_shoot_command.h"
#include "commands/stow_command.h"
#include "commands/speaker_align_command.h"
#include "commands/shoot_command.h"
#include "commands/speaker_align_command.h"
#include "subsystems/field.h"


frc2::SequentialCommandGroup AutoIntakeShootStillCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point, bool flip) {
    
  units::foot_t distance = (field::points::kSpeaker(flip) - shoot_point.pos.point).Magnitude();
    
    DrivetrainTarget rotateTarget;
    rotateTarget.rotation = shoot_point.pos.bearing;
    rotateTarget.v_x = 0_fps;
    rotateTarget.v_y = 0_fps;
    rotateTarget.translation_reference = kField;
    rotateTarget.control = kClosedLoop;

  return frc2::SequentialCommandGroup{
        DeployIntakeCommand{container},
        FollowTrajectoryCommand{ container, {intake_point}},
        PrepareFarShootCommand{container, distance.to<double>()},
        SpeakerAlignCommand{container, shoot_point},
        frc2::WaitCommand(container.super_structure_.pre_shoot_wait_.value()),
        ShootCommand{container},
        frc2::WaitCommand(container.super_structure_.post_shoot_wait_.value()),
  };
}