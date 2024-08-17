#pragma once

#include "frc846/robot/GenericCommand.h"
#include "frc846/util/math.h"
#include "subsystems/abstract/super_structure.h"
#include "subsystems/robot_container.h"

class AmpCommand
    : public frc846::robot::GenericCommand<RobotContainer, AmpCommand> {
 public:
  AmpCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};
