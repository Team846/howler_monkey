#ifndef y2024_COMMANDS_TRAP_COMMAND_H_
#define y2024_COMMANDS_TRAP_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/util/math.h"
#include "subsystems/pivot.h"
#include "subsystems/telescope.h"
#include "subsystems/wrist.h"
#include "subsystems/operator.h"
#include "subsystems/robot_container.h"

class TrapCommand
    : public frc2::CommandHelper<frc2::Command, TrapCommand>,
      public frc846::Loggable {
 public:
  TrapCommand(RobotContainer& container);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  PivotSubsystem& pivot_;
  TelescopeSubsystem& telescope_;
  WristSubsystem& wrist_;

  OperatorSubsystem& operator_;
  SuperStructureSubsystem& super_;

  double trapCounter=0;
  double trapDivisor=1;

  double pmpiv_adj=0;
  double pmtele_adj=0;
  double pms_adj=0;

  double mpiv_adj=0;
  double mtele_adj=0;
  double ms_adj=0;

  bool is_done_ = false;
};

#endif  // y2024_COMMANDS_TRAP_COMMAND_H_