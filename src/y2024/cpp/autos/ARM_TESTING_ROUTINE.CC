// #include "autos/ARM_TESTING_ROUTINE.h"

// #include <frc2/command/Commands.h>
// #include <frc2/command/ParallelDeadlineGroup.h>
// #include <frc2/command/ParallelRaceGroup.h>
// #include <frc2/command/SequentialCommandGroup.h>

// #include "frc2/command/WaitCommand.h"
// #include "frc2/command/WaitUntilCommand.h"
// #include "frc846/util/math.h"
// #include "commands/follow_trajectory_command.h"
// #include "subsystems/field.h"
// #include "subsystems/drivetrain.h"
// #include "subsystems/robot_container.h"
// #include "subsystems/swerve_module.h"

// TestingArmRoutine::TestingArmRoutine(
//     RobotContainer& container) : {
//   SetName("ARM TESTING ROUTINE");
//   AddCommands(

//       //run all commands

//       frc2::InstantCommand{[&, flip = should_flip_] { std::cout << "Stow Command" << std::endl; }},
//       StowCommand{container},
//       frc2::WaitCommand{1_s},

//       frc2::InstantCommand{[&, flip = should_flip_] { std::cout << "Deploy Intake Command" << std::endl; }},
//       DeployIntakeCommand{container},
//       frc2::WaitCommand{2_s},

//       frc2::InstantCommand{[&, flip = should_flip_] { std::cout << "Prepare Shoot Command" << std::endl; }},     
//       PrepareShootCommand{ container, 0.0},
//       frc2::WaitCommand{2_s},

//       frc2::InstantCommand{[&, flip = should_flip_] { std::cout << "Stow Command" << std::endl; }},
//       StowCommand{container},
//       frc2::WaitCommand{1_s}
      
//   );
// }