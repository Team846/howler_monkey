#include "autos/five_piece_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <memory>


#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/util/math.h"

#include "commands/prepare_short_shoot_command.h"
#include "commands/prepare_far_shoot_command.h"
#include "commands/shoot_command.h"
#include "commands/follow_trajectory_command.h"
#include "commands/shoot_command.h"
#include "commands/stow_command.h"
#include "commands/speaker_align_command.h"
#include "commands/auto_intake_and_shoot_command.h"
#include "commands/auto_sweep_command.h"
#include "commands/auto_intake_shoot_still_command.h"
#include "commands/deploy_intake_command.h"
#include "commands/spin_up_command.h"

#include "subsystems/field.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"
#include "subsystems/swerve_module.h"

FivePieceAuto::FivePieceAuto(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip){
    
    SetName("FivePieceAutoCommand" +
      std::string(should_flip_ ? " red" : " blue"));

    AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kFivePieceOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
        first_distance = (field::points::kSpeaker(flip) - pose_.point).Magnitude();
      }},
      PrepareShortShootCommand{ container,  (field::points::kSpeaker(should_flip_) - field::points::kFivePieceOrigin(should_flip_).point).Magnitude().to<double>()}, //first_distance.to<double>() },
      // SpeakerAlignCommand{ container, 
      //     field::points::kFPOrigin(should_flip_).point},
      frc2::WaitCommand{1_s}, //1.5 FIX TODO make wait time preference
      ShootCommand{ container },
      frc2::WaitCommand{container.super_structure_.post_shoot_wait_.value()},
      
      AutoIntakeShootStillCommand( container, {field::points::kFivePieceIntakeOne(should_flip_), 0_fps}, 
                                  {field::points::kFivePieceShootOne(should_flip_), 0_fps}, should_flip_),
      
      DeployIntakeCommand {container}, 
      FollowTrajectoryCommand {container, {{field::points::kFivePieceIntakeCenter(should_flip_), 13_fps}}}, 
      StowCommand {container}, 
      SpinUpCommand {container},

      frc2::ParallelDeadlineGroup {
        FollowTrajectoryCommand{ container, {{field::points::kFivePieceCenterIntermediate(should_flip_), 0_fps}, 
                                            {field::points::kFivePieceShootCenter(should_flip_), 0_fps}}},

        PrepareShortShootCommand{container, 0}
      },

      ShootCommand{container},
      frc2::WaitCommand(container.super_structure_.post_shoot_wait_.value()),

      AutoIntakeShootStillCommand( container, {field::points::kFivePieceIntakeTwo(should_flip_), 0_fps}, 
                                  {field::points::kFivePieceShootTwo(should_flip_), 0_fps}, should_flip_),

      DeployIntakeCommand {container},
      FollowTrajectoryCommand {container, {{field::points::kFivePieceThreeIntermediate(should_flip_), 0_fps}, 
                                          {field::points::kFivePieceIntakeThree(should_flip_), 0_fps}}},

      // AutoIntakeShootStillCommand( container, {field::points::kFivePieceIntakeThree(should_flip_), 0_fps}, 
      //                             {field::points::kFivePieceShootThree(should_flip_), 0_fps}, should_flip_),

      PrepareFarShootCommand{container, 0},
      SpeakerAlignCommand{container, {field::points::kFivePieceShootThree(should_flip_), 0_fps}},
      ShootCommand{container},
      frc2::WaitCommand(container.super_structure_.post_shoot_wait_.value()),
    
      StowCommand (container )
     );
}
