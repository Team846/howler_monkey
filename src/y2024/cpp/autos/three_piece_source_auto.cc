#include "autos/three_piece_source_auto.h"

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
#include "commands/deploy_intake_command.h"
#include "commands/shoot_command.h"
#include "commands/stow_command.h"
#include "commands/speaker_align_command.h"
#include "commands/auto_intake_and_shoot_command.h"
#include "subsystems/field.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"
#include "subsystems/swerve_module.h"

ThreePieceSourceAuto::ThreePieceSourceAuto(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip){
    
    SetName("FivePieceAutoCommand" +
      std::string(should_flip_ ? " red" : " blue"));

    AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kSourceSideOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
        first_distance = (field::points::kSpeaker(flip) - pose_.point).Magnitude();
      }},

      PrepareShortShootCommand{ container, 0.0}, //first_distance.to<double>() },
      // SpeakerAlignCommand{ container, 
      //     field::points::kSSOrigin(should_flip_).point},
      frc2::WaitCommand{1_s},
      ShootCommand{ container },

      DeployIntakeCommand{container},
      FollowTrajectoryCommand{ container, {{field::points::kSourceSideIntermediateOne(should_flip_), 0_fps}, 
                                            {field::points::kSourceSideIntakeOne(should_flip_), 0_fps}}},
      PrepareFarShootCommand {container, 0},

      FollowTrajectoryCommand{container, {{field::points::kSourceSideIntermediateOne(should_flip_), 0_fps},
                                            {field::points::kSourceSideShootOne(should_flip_), 0_fps}}},
      
      ShootCommand{ container }, 

      DeployIntakeCommand{container},
      FollowTrajectoryCommand {container, {{field::points::kSourceSideIntermediateTwo(should_flip_), 0_fps},
                                            {field::points::kSourceSideIntakeTwo(should_flip_), 0_fps}}},
      
      PrepareFarShootCommand {container, 0},
      FollowTrajectoryCommand {container, {{field::points::kSourceSideIntermediateTwo(should_flip_), 0_fps},
                                            {field::points::kSourceSideShootTwo(
                                              
                                              
                                              should_flip_), 0_fps}}},
      
      ShootCommand{ container }, 
      StowCommand { container }, 
      
      FollowTrajectoryCommand{ container, {{field::points::kSourceSideIntermediateTwo(should_flip_), 4_fps}}}
     );
}
