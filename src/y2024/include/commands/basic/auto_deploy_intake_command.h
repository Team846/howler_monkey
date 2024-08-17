#pragma once

#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class AutoDeployIntakeCommand
    : public frc846::robot::GenericCommand<RobotContainer,
                                           AutoDeployIntakeCommand> {
 public:
  AutoDeployIntakeCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  bool is_done_ = false;
};
