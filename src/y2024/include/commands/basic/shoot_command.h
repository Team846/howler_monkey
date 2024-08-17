#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class ShootCommand
    : public frc846::robot::GenericCommand<RobotContainer, ShootCommand> {
 public:
  ShootCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};
