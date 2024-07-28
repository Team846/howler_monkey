#include "commands/basic/trap_command.h"

#include <frc/RobotBase.h>

#include <cmath>

#include "frc846/util/math.h"
#include "frc846/wpilib/time.h"

TrapCommand::TrapCommand(RobotContainer& container, int stage)
    : frc846::base::Loggable{"trap_command"},
      intake_(container.intake_),
      shooter_(container.shooter_),
      super_(container.super_structure_),
      stage_{stage} {
  AddRequirements({&intake_, &shooter_, &container.pivot_,
                   &container.telescope_, &container.wrist_});
  SetName("trap_command");
}

void TrapCommand::Initialize() { Log("Trap Command Initialize"); }

void TrapCommand::Execute() {
  if (stage_ == 1) {
    intake_.SetTarget(intake_.ZeroTarget());
    shooter_.SetTarget(shooter_.ZeroTarget());

    super_.SetTargetSetpoint(super_.getPreClimbSetpoint());
  }
  if (stage_ == 2) {
    intake_.SetTarget(intake_.ZeroTarget());
    shooter_.SetTarget(shooter_.ZeroTarget());

    auto pullClimbTarget = super_.getPreClimbSetpoint();
    pullClimbTarget.pivot = super_.pivot_pull_target_.value();
    super_.SetTargetSetpoint(pullClimbTarget);
  }
  if (stage_ == 3) {
    intake_.SetTarget(intake_.ZeroTarget());
    shooter_.SetTarget(shooter_.ZeroTarget());

    super_.SetTargetSetpoint(super_.getPreScoreSetpoint());
  }
  if (stage_ == 4) {
    intake_.SetTarget(intake_.ZeroTarget());
    shooter_.SetTarget(shooter_.ZeroTarget());

    super_.SetTargetSetpoint(super_.getTrapScoreSetpoint());
  }
  if (stage_ == 5) {
    shooter_.SetTarget(shooter_.ZeroTarget());

    intake_.SetTarget({IntakeState::kRelease});
  }
  if (stage_ == 6) {
    intake_.SetTarget(intake_.ZeroTarget());
    shooter_.SetTarget(shooter_.ZeroTarget());

    super_.SetTargetSetpoint(super_.getPostScoreSetpoint());
  }
}

void TrapCommand::End(bool interrupted) { Log("Trap Command Finished"); }

bool TrapCommand::IsFinished() { return false; }