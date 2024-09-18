#include "commands/auto_intake_and_shoot_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/basic/auto_deploy_intake_command.h"
#include "commands/basic/auto_shoot_command.h"
#include "commands/basic/prepare_auto_shoot_command.h"
#include "commands/follow_trajectory_command.h"
#include "field.h"
#include "frc2/command/WaitCommand.h"

AutoIntakeAndShootCommand::AutoIntakeAndShootCommand(
    RobotContainer& container, std::vector<frc846::Waypoint> intake_path,
    std::vector<frc846::Waypoint> shoot_path)
    : frc846::robot::GenericCommandGroup<RobotContainer,
                                         AutoIntakeAndShootCommand,
                                         frc2::SequentialCommandGroup>{
          container, "auto_intake_shoot_command",
          frc2::SequentialCommandGroup{

              AutoDeployIntakeCommand{container},
              FollowTrajectoryCommand{container, intake_path},
              frc2::ParallelDeadlineGroup{
                  frc2::SequentialCommandGroup{
                      FollowTrajectoryCommand{container, shoot_path},
                      frc2::WaitCommand(
                          container.super_structure_.pre_shoot_wait_.value())},
                  PrepareAutoShootCommand{container}},
              AutoShootCommand{container},
              frc2::WaitCommand(
                  container.super_structure_.post_shoot_wait_.value())

          }} {}