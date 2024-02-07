#ifndef y2024_COMMANDS_DEPLOY_INTAKE_COMMAND_H_
#define y2024_COMMANDS_DEPLOY_INTAKE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/math.h"
#include "subsystems/shintake.h"
#include "subsystems/pivot.h"
#include "subsystems/telescope.h"
#include "subsystems/wrist.h"
#include "subsystems/robot_container.h"

class DeployIntakeCommand
    : public frc2::CommandHelper<frc2::Command, DeployIntakeCommand>,
      public frc846::Loggable {
 public:
  DeployIntakeCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShintakeSubsystem& shintake_;
  PivotSubsystem& pivot_;
  TelescopeSubsystem& telescope_;
  WristSubsystem& wrist_;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_DEPLOY_INTAKE_COMMAND_H_