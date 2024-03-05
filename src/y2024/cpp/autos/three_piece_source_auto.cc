#include "autos/three_piece_source_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <memory>

#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/util/math.h"
#include "commands/prepare_shoot_command.h"
#include "commands/shoot_command.h"
#include "commands/follow_trajectory_command.h"
#include "commands/shoot_command.h"
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
        auto pose_ = field::points::kSSOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
        first_distance = (field::points::kSpeaker(flip) - pose_.point).Magnitude();
      }},
      PrepareShootCommand{ container, 0.0}, //first_distance.to<double>() },
      SpeakerAlignCommand{ container, 
          field::points::kSSOrigin(should_flip_).point},
      ShootCommand{ container },
    
      
      AutoIntakeAndShootCommand( container, {field::points::kSSIntakeOne(should_flip_), 0_fps}, 
                                  {field::points::kSSShootOne(should_flip_), 0_fps}, should_flip_),
      
      AutoIntakeAndShootCommand( container, {field::points::kSSIntakeTwo(should_flip_), 0_fps}, 
                                  {field::points::kSSShootTwo(should_flip_), 0_fps}, should_flip_),


      FollowTrajectoryCommand{ container, {{field::points::kSSFinalPosition(should_flip_), 0_fps}}}
     );
}
