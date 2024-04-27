#include "autos/one_piece_auto.h"

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
        std::cout << container.drivetrain_.readings().pose.bearing.to<double>() << std::endl;
        std::cout << (field::points::one_piece_extra_distance_.value() * units::math::sin(container.drivetrain_.readings().pose.bearing)).to<double>() << std::endl;
        std::cout << (field::points::one_piece_extra_distance_.value() * units::math::cos(container.drivetrain_.readings().pose.bearing)).to<double>() << std::endl;
      }},
      AutoShootCommand{container, setpoints::kAutoShortShoot(0), setpoints::kAutoShortShoot(2), 0}, //PrepareShort
      frc2::WaitCommand{ 2.0_s },
      ShootCommand{ container },
      frc2::WaitCommand{ 0.5_s },

      FollowTrajectoryCommand{ container, {{{{field::points::kOPOrigin(should_flip_).point.x 
        + field::points::one_piece_extra_distance_.value() * units::math::sin(container.drivetrain_.readings().pose.bearing),
        field::points::kOPOrigin(should_flip_).point.y + field::points::one_piece_extra_distance_.value() * units::math::cos(container.drivetrain_.readings().pose.bearing)}, 
          field::points::kOPOrigin(should_flip_).bearing}, 0_fps}}}
     );
}
