#include "autos/one_piece_auto.h"

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
#include "commands/wait_till_spin_up_command.h"

OnePieceAuto::OnePieceAuto(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip){
    
    SetName("OnePieceAutoCommand" +
      std::string(should_flip_ ? " red" : " blue"));

    AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kOPOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        // container.drivetrain_.SetBearing(pose_.bearing);
      }},
      frc2::InstantCommand{[&, flip = should_flip_] {
        DrivetrainTarget target{3_fps, 0_fps, kRobot, DrivetrainRotationVelocity(0), kOpenLoop};

        container.drivetrain_.SetTarget(target);
      }},
      frc2::InstantCommand{[&, flip = should_flip_] {
        container.drivetrain_.SetTarget(container.drivetrain_.ZeroTarget());
      }}
     
     );
}
