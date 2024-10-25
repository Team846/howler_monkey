#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class SpinUpCommand
    : public frc846::robot::GenericCommand<RobotContainer, SpinUpCommand> {
 public:
  SpinUpCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  bool has_spinned_up_;

  units::second_t start_time;
};
