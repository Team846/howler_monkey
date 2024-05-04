#ifndef y2024_COMMANDS_WRIST_COMMAND_H_
#define y2024_COMMANDS_WRIST_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "frc846/control/sequencer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/abstract/vision.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/pivot.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/wrist.h"
#include "subsystems/robot_container.h"

class WristCommand : public frc2::CommandHelper<frc2::Command, WristCommand> {
 public:
  WristCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  ControlInputSubsystem& control_input_;
  WristSubsystem& wrist_;
  PivotSubsystem& pivot_;
  DrivetrainSubsystem& drivetrain_;
  ShooterSubsystem& shooter_;
  SuperStructureSubsystem& super_;
  VisionSubsystem& vision_;

  ControlInputReadings prev_ci_readings_{};

  ShootingCalculator shooting_calculator{};

  frc846::control::Sequencer msequencer_{"wrist_command_sequencer"};

  units::degree_t ms_adj = 0.0_deg;
};

#endif  // y2024_COMMANDS_WRIST_COMMAND_H_