#pragma once

#include "constants.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
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

 private:
  ShootingCalculator shooting_calculator_;
};
