#include "autos/drive_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/util/math.h"
#include "commands/follow_trajectory_command.h"
#include "field.h"
#include "subsystems/drivetrain.h"
#include "subsystems/robot_container.h"
#include "subsystems/swerve_module.h"

DriveAuto::DriveAuto(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("DriveAutoCommand" +
          std::string(should_flip_ ? " red" : " blue"));
  AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kTestingOrigin(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);

        std::cout << "start" << std::endl;
      }},

      FollowTrajectoryCommand{
          container,
          {
              {field::points::kTestingPoint(should_flip_), 0_fps},
          },
      },

      frc2::InstantCommand{[&, flip = should_flip_] {

        std::cout << "end" << std::endl;
      }}
  );
}