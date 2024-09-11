#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "autos/one_piece_auto.h"
#include "commands/basic/auto_shoot_command.h"
#include "commands/basic/prepare_auto_shoot_command.h"
#include "frc2/command/WaitCommand.h"

OnePieceAuto::OnePieceAuto(RobotContainer& container,
                           units::degree_t start_angle, std::string num)
    : frc846::robot::GenericCommandGroup<RobotContainer, OnePieceAuto,
                                         frc2::SequentialCommandGroup>{
          container, "1p_auto_" + num,
          frc2::SequentialCommandGroup{

              // frc2::InstantCommand{[&, s = start_angle] {
              //   container.drivetrain_.SetBearing(s);
              //   Log("OP: Zeroing to {}", s.to<double>());
              // }},
              // PrepareAutoShootCommand{container},
              // AutoShootCommand{container},
              // frc2::WaitCommand{
              //     container.super_structure_.post_shoot_wait_.value()}

          }} {}
