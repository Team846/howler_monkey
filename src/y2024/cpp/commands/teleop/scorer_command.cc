#include "commands/teleop/scorer_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/swerve_module.h"

ScorerCommand::ScorerCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      scorer_(container.scorer_),
      super_(container.super_structure_) {
  AddRequirements({&scorer_});
  SetName("scorer_command");
}

void ScorerCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  ScorerTarget target = scorer_.ZeroTarget();

  if (ci_readings_.shooting) {
    target.target_state = ScorerState::kShoot;
    target.opr = false;
  } else if (ci_readings_.running_intake || ci_readings_.running_source) {
    target.target_state = ScorerState::kIntake;
    target.opr = false;
  } else if (ci_readings_.running_pass) {
    target.target_state = ScorerState::kPass;
    target.opr = false;
  } else if (ci_readings_.manual_feed) {
    target.target_state = ScorerState::kIntake;
    target.opr = true;
  } else if (ci_readings_.manual_spin_up) {
    target.target_state = ScorerState::kSpinUp;
    target.opr = false;
  } else if (ci_readings_.eject) {
    target.target_state = ScorerState::kRelease;
    target.opr = false;
  } else {
    target.target_state = ScorerState::kIdle;
    target.opr = false;
  }

  prev_ci_readings_ = ci_readings_;

  scorer_.SetTarget(target);
}

bool ScorerCommand::IsFinished() { return false; }