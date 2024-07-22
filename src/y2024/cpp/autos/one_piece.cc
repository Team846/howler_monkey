#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <memory>

#include "autos/one_piece_auto.h"
#include "commands/auto_intake_and_shoot_command.h"
#include "commands/basic/auto_shoot_command.h"
#include "commands/basic/prepare_auto_shoot_command.h"
#include "commands/follow_trajectory_command.h"
#include "field.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/swerve_module.h"
#include "subsystems/robot_container.h"

OnePieceAuto::OnePieceAuto(RobotContainer& container,
                           units::degree_t start_angle, std::string num) {
  SetName("OnePieceAutoCommand " + num);

  AddCommands(
      frc2::InstantCommand{[&, s = start_angle] {
        container.drivetrain_.SetBearing(s);
        // std::cout << "Zeroing to " << should_flip ? 180 : 0 << std::endl;
      }},
      PrepareAutoShootCommand{container}, AutoShootCommand{container},
      frc2::WaitCommand{container.super_structure_.post_shoot_wait_.value()});
}
