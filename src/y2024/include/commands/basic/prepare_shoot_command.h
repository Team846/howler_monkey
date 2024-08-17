#pragma once

#include "constants.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
#include "subsystems/robot_container.h"

class PrepareShootCommand
    : public frc846::robot::GenericCommand<RobotContainer,
                                           PrepareShootCommand> {
 public:
  PrepareShootCommand(RobotContainer& container, bool super_shot);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  bool super_shot_;

  ShootingCalculator shooting_calculator_;
};
