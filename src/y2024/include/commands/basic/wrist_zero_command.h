#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class WristZeroCommand
    : public frc846::robot::GenericCommand<RobotContainer, WristZeroCommand> {
 public:
  WristZeroCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

  InterruptionBehavior GetInterruptionBehavior() const override {
    return InterruptionBehavior::kCancelIncoming;
  }
};
