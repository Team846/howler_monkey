#ifndef y2024_COMMANDS_AUTO_SHOOT_COMMAND_H_
#define y2024_COMMANDS_AUTO_SHOOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/pivot.h"
#include "subsystems/telescope.h"
#include "subsystems/wrist.h"
#include "subsystems/robot_container.h"

class AutoShootCommand
    : public frc2::CommandHelper<frc2::Command, AutoShootCommand>,
      public frc846::Loggable {
 public:
  AutoShootCommand(RobotContainer& container, double pivot_target, double wrist_target, double tele_target);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  double pivot_target_;
  double wrist_target_;
  double tele_target_;
  PivotSubsystem& pivot_;
  TelescopeSubsystem& telescope_;
  WristSubsystem& wrist_;

  double pmpiv_adj=0;
  double pmtele_adj=0;
  double pms_adj=0;

  double mpiv_adj=0;
  double mtele_adj=0;
  double ms_adj=0;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_AUTO_SHOOT_COMMAND_H_