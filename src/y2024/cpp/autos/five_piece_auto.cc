// #include "autos/five_piece_auto.h"

// #include <frc2/command/Commands.h>
// #include <frc2/command/ParallelDeadlineGroup.h>
// #include <frc2/command/ParallelRaceGroup.h>
// #include <frc2/command/SequentialCommandGroup.h>
// #include <memory>


// #include "frc2/command/WaitCommand.h"
// #include "frc2/command/WaitUntilCommand.h"
// #include "frc846/util/math.h"
// #include "commands/prepare_shoot_command.h"
// #include "commands/shoot_command.h"
// #include "commands/follow_trajectory_command.h"
// #include "commands/shoot_command.h"
// #include "commands/speaker_align_command.h"
// #include "commands/auto_intake_and_shoot_command.h"
// #include "subsystems/field.h"
// #include "subsystems/drivetrain.h"
// #include "subsystems/robot_container.h"
// #include "subsystems/swerve_module.h"

// FivePieceAuto::FivePieceAuto(
//     RobotContainer& container, bool should_flip)
//     : should_flip_(should_flip){
    
//     SetName("FivePieceAutoCommand" +
//       std::string(should_flip_ ? " red" : " blue"));

//     AddCommands(
//       frc2::InstantCommand{[&, flip = should_flip_] {
//         auto pose_ = field::points::kFPOrigin(flip);
//         container.drivetrain_.SetPoint(pose_.point);
//         container.drivetrain_.SetBearing(pose_.bearing);
//         first_distance = (field::points::kSpeaker(flip) - pose_.point).Magnitude();
//       }},
//       PrepareShootCommand{ container, first_distance },
//       SpeakerAlignCommand{ container, 
//           field::points::kFPOrigin(should_flip_).point},
//       ShootCommand{ container },
    
      
//       AutoIntakeAndShootCommand( container, {field::points::kFPIntakeOne(should_flip_), 0_fps}, 
//                                   {field::points::kFPShootOne(should_flip_), 0_fps}, should_flip_),
      
//       AutoIntakeAndShootCommand( container, {field::points::kFPIntakeTwo(should_flip_), 0_fps}, 
//                                   {field::points::kFPShootTwo(should_flip_), 0_fps}, should_flip_),
      
//       AutoIntakeAndShootCommand( container, {field::points::kFPIntakeThree(should_flip_), 0_fps}, 
//                                   {field::points::kFPShootThree(should_flip_), 0_fps}, should_flip_),

//       AutoIntakeAndShootCommand( container, {field::points::kFPIntakeFour(should_flip_), 0_fps}, 
//                                   {field::points::kFPShootFour(should_flip_), 0_fps}, should_flip_),

//       FollowTrajectoryCommand{ container, {{field::points::kFPFinalPosition(should_flip_), 0_fps}}}
//      );
// }
