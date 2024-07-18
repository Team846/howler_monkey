#include "autos/five_piece_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <memory>

#include "commands/auto_intake_and_shoot_command.h"
#include "commands/basic/auto_shoot_command.h"
#include "commands/basic/prepare_auto_shoot_command.h"
#include "commands/follow_trajectory_command.h"
#include "field.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/swerve_module.h"
#include "subsystems/robot_container.h"

FivePieceAuto::FivePieceAuto(RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("FivePieceAutoCommand" +
          std::string(should_flip_ ? " blue" : " red"));

  AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kFivePieceOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(should_flip_ ? 180_deg : 0_deg);
      }},
      PrepareAutoShootCommand{container}, AutoShootCommand{container},
      frc2::WaitCommand{container.super_structure_.post_shoot_wait_.value()},

      AutoIntakeAndShootCommand(
          container, {field::points::kFivePieceIntakeOne(should_flip_), 0_fps},
          {field::points::kFivePieceMidOne(should_flip_), 15_fps},
          {field::points::kFivePieceShootOne(should_flip_), 0_fps}),

      AutoIntakeAndShootCommand(
          container, {field::points::kFivePieceIntakeTwo(should_flip_), 0_fps},
          {field::points::kFivePieceMidTwo(should_flip_), 15_fps},
          {field::points::kFivePieceShootTwo(should_flip_), 0_fps}),

      AutoIntakeAndShootCommand(
          container,
          {field::points::kFivePieceIntakeThree(should_flip_), 0_fps},
          {field::points::kFivePieceMidThree(should_flip_), 15_fps},
          {field::points::kFivePieceShootThree(should_flip_), 0_fps}),

      FollowTrajectoryCommand(
          container,
          {{field::points::kFivePieceIntakeOne(should_flip_), 0_fps}}));
}
