#include "commands/auto_intake_and_shoot_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/basic/deploy_intake_command.h"
#include "commands/basic/prepare_auto_shoot_command.h"
#include "commands/basic/shoot_command.h"
#include "commands/basic/stow_command.h"
#include "commands/follow_trajectory_command.h"
#include "commands/speaker_trajectory_command.h"
#include "field.h"
#include "frc2/command/WaitCommand.h"

frc2::SequentialCommandGroup AutoIntakeAndShootCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point) {
  return frc2::SequentialCommandGroup{
      DeployIntakeCommand{container},
      FollowTrajectoryCommand{container, {intake_point}},
      frc2::ParallelDeadlineGroup{
          frc2::SequentialCommandGroup{
              SpeakerTrajectoryCommand{container, {shoot_point}},
              frc2::WaitCommand(
                  container.super_structure_.pre_shoot_wait_.value())},
          PrepareAutoShootCommand{container}},
      ShootCommand{container},
      frc2::WaitCommand(container.super_structure_.post_shoot_wait_.value()),
      StowCommand{container}};
}