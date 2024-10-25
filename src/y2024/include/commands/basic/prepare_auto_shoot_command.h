#pragma once

#include "calculators/shooting_calculator.h"
#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class PrepareAutoShootCommand
    : public frc846::robot::GenericCommand<RobotContainer,
                                           PrepareAutoShootCommand> {
 public:
  PrepareAutoShootCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};
