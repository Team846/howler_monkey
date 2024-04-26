#ifndef y2024_COMMANDS_WRIST_COMMAND_H_
#define y2024_COMMANDS_WRIST_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/control/motion.h"
#include "subsystems/control_input.h"
#include "subsystems/drivetrain.h"
#include "subsystems/pivot.h"
#include "subsystems/robot_container.h"
#include "subsystems/scorer.h"
#include "subsystems/super_structure.h"
#include "subsystems/vision.h"
#include "subsystems/wrist.h"

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
  ScorerSubsystem& scorer_;
  SuperStructureSubsystem& super_;
  VisionSubsystem& vision_;

  ControlInputReadings prev_ci_readings_{};

  ShootingCalculator shooting_calculator{};

  bool wristHasRun = false;

  units::degree_t ms_adj = 0.0_deg;
};

#endif  // y2024_COMMANDS_WRIST_COMMAND_H_