#ifndef y2024_COMMANDS_PREPARE_FAR_SHOOT_COMMAND_H_
#define y2024_COMMANDS_PREPARE_FAR_SHOOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/scorer.h"
#include "subsystems/pivot.h"
#include "subsystems/telescope.h"
#include "subsystems/wrist.h"
#include "subsystems/robot_container.h"

class PrepareFarShootCommand
    : public frc2::CommandHelper<frc2::Command, PrepareFarShootCommand>,
      public frc846::Loggable {
 public:
  PrepareFarShootCommand(RobotContainer& container, double shooting_distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ScorerSubsystem& scorer_;
  PivotSubsystem& pivot_;
  TelescopeSubsystem& telescope_;
  WristSubsystem& wrist_;

  double dist_;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_PREPARE_SHOOT_COMMAND_H_