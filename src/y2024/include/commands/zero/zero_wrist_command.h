#ifndef y2024_COMMANDS_ZERO_WRIST_COMMAND_H_
#define y2024_COMMANDS_ZERO_WRIST_COMMAND_H_

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/wrist.h"
#include "subsystems/super_structure.h"
#include "subsystems/robot_container.h"
#include "frc846/util/share_tables.h"

class ZeroWristCommand
    : public frc2::CommandHelper<frc2::Command, ZeroWristCommand> {
 public:
  ZeroWristCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

  virtual InterruptionBehavior GetInterruptionBehavior() const override {
    return InterruptionBehavior::kCancelIncoming;
  }

 private:
  WristSubsystem& wrist_;
  SuperStructureSubsystem& super_;

  bool is_done_ = false;
  int loops = 1;
  double last_pos_ = 0;
};

#endif  // y2024_COMMANDS_DRIVE_COMMAND_H_