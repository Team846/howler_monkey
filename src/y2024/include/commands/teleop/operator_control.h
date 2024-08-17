#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class OperatorControlCommand
    : public frc846::robot::GenericCommand<RobotContainer,
                                           OperatorControlCommand> {
 public:
  OperatorControlCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};
