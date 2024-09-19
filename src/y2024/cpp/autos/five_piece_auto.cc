#include "autos/five_piece_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "commands/auto_intake_and_shoot_command.h"
#include "commands/basic/auto_shoot_command.h"
#include "commands/basic/prepare_auto_shoot_command.h"
#include "commands/basic/spin_up_command.h"
#include "commands/basic/wrist_zero_command.h"
#include "commands/teleop/stow_command.h"
#include "field.h"
#include "frc846/swerve/follow_trajectory_command.h"

FivePieceAuto::FivePieceAuto(RobotContainer& container, bool should_flip)
    : frc846::robot::GenericCommandGroup<RobotContainer, FivePieceAuto,
                                         frc2::SequentialCommandGroup>{
          container, should_flip ? "5p_auto_blue" : "5p_auto_red",
          frc2::SequentialCommandGroup{

              frc2::InstantCommand{[&, flip = should_flip] {
                auto pose_ = field::points.kFivePieceOrigin(flip);
                container.drivetrain_.SetPoint(pose_.point);
                container.drivetrain_.SetBearing(flip ? 180_deg : 0_deg);
              }},
              frc2::ParallelDeadlineGroup{
                  frc2::ParallelDeadlineGroup{
                      frc2::WaitUntilCommand{[&] {
                        return container_.pivot_.WithinTolerance(
                            container_.super_structure_.getStowSetpoint()
                                .pivot); /* homing wrist at start of auto */
                      }},
                      StowCommand{container}},
                  SpinUpCommand{
                      container}}, /* start to spin up shooter to save time */
              WristZeroCommand{container}, /* homing the wrist */
              PrepareAutoShootCommand{container}, AutoShootCommand{container},
              frc2::WaitCommand{
                  container.super_structure_.post_shoot_wait_.value()},

              AutoIntakeAndShootCommand(
                  container, field::points.intake_one_path(should_flip),
                  {field::points.kFivePieceOrigin(should_flip)}),

              AutoIntakeAndShootCommand(
                  container, field::points.intake_two_path(should_flip),
                  {field::points.kFivePieceOrigin(should_flip)}),

              AutoIntakeAndShootCommand(
                  container, field::points.intake_three_path(should_flip),
                  {field::points.kFivePieceOrigin(should_flip)}),

              frc846::swerve::FollowTrajectoryCommand(
                  container, {field::points.kFivePieceFinish(should_flip)})}} {}
