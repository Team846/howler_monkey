#include "autos/drive_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "field.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/swerve/follow_trajectory_command.h"

DriveAuto::DriveAuto(RobotContainer& container)
    : frc846::robot::GenericCommandGroup<RobotContainer, DriveAuto,
                                         frc2::SequentialCommandGroup>{
          container, "drive_auto",
          frc2::SequentialCommandGroup{

              frc2::InstantCommand{[&] {
                auto pose_ = field::points.kTestingOrigin();
                container.drivetrain_.SetPoint(pose_.point);
                container.drivetrain_.SetBearing(pose_.bearing);
              }},
              frc846::swerve::FollowTrajectoryCommand{
                  container, {field::points.kTestingPoint()}}

          }} {}