#include "commands/auto_sweep_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc2/command/WaitCommand.h"
#include "commands/follow_trajectory_command.h"
#include "commands/positioning/stow_command.h"
#include "commands/speaker_align_command.h"
#include "commands/shoot_command.h"
#include "commands/positioning/auto_shoot_command.h"
#include "commands/positioning/intake_command.h"
#include "subsystems/setpoints.h"
#include "subsystems/field.h"


frc2::SequentialCommandGroup AutoSweepCommand(
    RobotContainer& container, frc846::InputWaypoint intake_point,
    frc846::InputWaypoint shoot_point, bool flip, double shoot_time) {
    
  units::foot_t distance = (field::points::kSpeaker(flip) - shoot_point.pos.point).Magnitude();
        
  return frc2::SequentialCommandGroup{
        frc2::ParallelCommandGroup{
          IntakeCommand{container},
          frc2::InstantCommand([&] {
            container.shooter_.SetTarget({kShooterIdle});
            container.intake_.SetTarget({kIntake});
          })
        },
        FollowTrajectoryCommand{ container, {intake_point}},
        
        frc2::ParallelDeadlineGroup {
            FollowTrajectoryCommand{ container, {shoot_point}},
            frc2::SequentialCommandGroup{
                AutoShootCommand{container, setpoints::kAutoShortShoot(0), setpoints::kAutoShortShoot(2), 0}, //PrepareShort
                frc2::WaitCommand(container.super_structure_.pre_shoot_wait_.value()),
                frc2::WaitCommand(units::time::second_t{shoot_time}),
                ShootCommand{container},
                frc2::WaitCommand(container.super_structure_.post_shoot_wait_.value())
            }
        }
        // SpeakerAlignCommand{container, shoot_point.pos.point},
  };
}