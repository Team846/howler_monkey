#include "autos/four_piece_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <memory>

#include "commands/auto_intake_and_shoot_command.h"
#include "commands/follow_trajectory_command.h"
#include "commands/prepare_shoot_command.h"
#include "commands/shoot_command.h"
#include "field.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/util/math.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"
#include "subsystems/swerve_module.h"

FourPieceAuto::FourPieceAuto(RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("FourPieceAutoCommand" +
          std::string(should_flip_ ? " red" : " blue"));

  AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kFourPieceOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
      }},
      frc2::SequentialCommandGroup{frc2::WaitCommand{1.5_s},
                                   PrepareShootCommand{container}},
      ShootCommand{container},
      frc2::WaitCommand{container.super_structure_.post_shoot_wait_.value()},

      AutoIntakeAndShootCommand(
          container, {field::points::kFourPieceIntakeOne(should_flip_), 0_fps},
          {field::points::kFourPieceShootOne(should_flip_), 0_fps}),

      AutoIntakeAndShootCommand(
          container, {field::points::kFourPieceIntakeTwo(should_flip_), 0_fps},
          {field::points::kFourPieceShootTwo(should_flip_), 0_fps}),

      AutoIntakeAndShootCommand(
          container,
          {field::points::kFourPieceIntakeThree(should_flip_), 0_fps},
          {field::points::kFourPieceShootThree(should_flip_), 0_fps}),

      FollowTrajectoryCommand{
          container,
          {{field::points::kFourPieceFinalPosition(should_flip_), 0_fps}}});
}
