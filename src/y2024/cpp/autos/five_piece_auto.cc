// #include "autos/five_piece_auto.h"

// #include <frc2/command/Commands.h>
// #include <frc2/command/ParallelDeadlineGroup.h>
// #include <frc2/command/ParallelRaceGroup.h>
// #include <frc2/command/SequentialCommandGroup.h>
// #include <memory>


// #include "frc2/command/WaitCommand.h"
// #include "frc2/command/WaitUntilCommand.h"
// #include "frc846/math.h"
// #include "commands/follow_trajectory_command.h"
// // #include "y2024/commands/shoot_command.h"
// #include "commands/auto_intake_and_shoot.h"
// #include "subsystems/field.h"
// #include "subsystems/drivetrain.h"
// #include "subsystems/robot_container.h"
// #include "subsystems/swerve_module.h"

// frc2::CommandPtr FivePieceAuto(
//     RobotContainer& container, bool should_flip_){
//      return frc2::cmd::Sequence(
//       frc2::InstantCommand{[&, flip = should_flip_] {
//         auto pose_ = field::points::kFPOrigin(flip);
//         container.drivetrain_.SetPoint(pose_.point);
//         container.drivetrain_.SetBearing(pose_.bearing);
//       }}.ToPtr(),
//       //Turn and shoot preload
//         FollowTrajectoryCommand{ container, 
//             {{field::points::kFPPreloadShoot(should_flip_), 0_fps}}}.ToPtr(),
//         frc2::WaitCommand(0.1_s).ToPtr()
        

//         // AutoIntakeAndShootCommand( container, {field::points::kFPWing3Intake(should_flip_), 5_fps}, 
//         //                             {field::points::kFPWing3Shoot(should_flip_), 0_fps}),
        
//         // AutoIntakeAndShootCommand( container, {field::points::kFPWing2Intake(should_flip_), 5_fps}, 
//         //                             {field::points::kFPWing2Shoot(should_flip_), 0_fps}),
        
//         // AutoIntakeAndShootCommand( container, {field::points::kFPWing1Intake(should_flip_), 5_fps}, 
//         //                             {field::points::kFPWing1Shoot(should_flip_), 0_fps}),

//         // //Getting center 1 ring
//         // AutoIntakeAndShootCommand( container, {field::points::kFPCenter1Intake(should_flip_), 5_fps}, 
//         //                             {field::points::kFPCenter1Shoot(should_flip_), 0_fps})
//     );
// }
