#ifndef y2024_COMMANDS_DEPLOY_INTAKE_COMMAND_H_
#define y2024_COMMANDS_DEPLOY_INTAKE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frcLib846/math.h"
#include "subsystems/scorer.h"
#include "subsystems/scoring_positioner.h"
#include "subsystems/robot_container.h"

class DeployIntakeCommand
    : public frc2::CommandHelper<frc2::Command, DeployIntakeCommand>,
      public frcLib846::Loggable {
 public:
  DeployIntakeCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ScorerSubsystem& scorer_;
  ScoringPositionerSubsystem& scoring_positioner_;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_DEPLOY_INTAKE_COMMAND_H_