#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class TrapCommand
    : public frc846::robot::GenericCommand<RobotContainer, TrapCommand> {
 public:
  TrapCommand(RobotContainer& container, int stage);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  int stage_;
};
