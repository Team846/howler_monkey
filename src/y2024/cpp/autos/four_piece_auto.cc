#include "autos/four_piece_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <memory>


#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/util/math.h"
#include "commands/shoot_command.h"
#include "commands/follow_trajectory_command.h"
#include "commands/shoot_command.h"
#include "commands/speaker_align_command.h"
#include "commands/auto_intake_and_shoot_command.h"
#include "commands/positioning/auto_shoot_command.h"
#include "subsystems/setpoints.h"
#include "subsystems/field.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"
#include "subsystems/swerve_module.h"

FourPieceAuto::FourPieceAuto(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip){
    
    SetName("FourPieceAutoCommand" +
      std::string(should_flip_ ? " red" : " blue"));

    AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kFourPieceOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
        first_distance = (field::points::kSpeaker(flip) - pose_.point).Magnitude();
      }},
      AutoShootCommand{container, setpoints::kAutoShortShoot(0), setpoints::kAutoShortShoot(2), 0}, //PrepareShort
      // SpeakerAlignCommand{ container, 
      //     field::points::kFPOrigin(should_flip_).point},
      frc2::WaitCommand{1.5_s},
      ShootCommand{ container },
      frc2::WaitCommand{container.super_structure_.post_shoot_wait_.value()},
    
      
      AutoIntakeAndShootCommand( container, {field::points::kFourPieceIntakeOne(should_flip_), 0_fps}, 
                                  {field::points::kFourPieceShootOne(should_flip_), 0_fps}, should_flip_),
      
      AutoIntakeAndShootCommand( container, {field::points::kFourPieceIntakeTwo(should_flip_), 0_fps}, 
                                  {field::points::kFourPieceShootTwo(should_flip_), 0_fps}, should_flip_),
      
      AutoIntakeAndShootCommand( container, {field::points::kFourPieceIntakeThree(should_flip_), 0_fps}, 
                                  {field::points::kFourPieceShootThree(should_flip_), 0_fps}, should_flip_),

      FollowTrajectoryCommand{ container, {{field::points::kFourPieceFinalPosition(should_flip_), 0_fps}}}
     );
}
