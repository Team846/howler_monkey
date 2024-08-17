#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class BracerCommand
    : public frc846::robot::GenericCommand<RobotContainer, BracerCommand> {
 public:
  BracerCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  int counter = 120;

  ControlInputReadings prev_ci_readings_{};
};
