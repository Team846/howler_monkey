#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class PassCommand
    : public frc846::robot::GenericCommand<RobotContainer, PassCommand> {
 public:
  PassCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};
