#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class AwaitPieceStateCommand
    : public frc846::robot::GenericCommand<RobotContainer,
                                           AwaitPieceStateCommand> {
 public:
  AwaitPieceStateCommand(RobotContainer& container, bool target_has_piece);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  bool target_has_piece_;
};
