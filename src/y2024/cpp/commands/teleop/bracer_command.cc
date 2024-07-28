#include "commands/teleop/bracer_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/base/loggable.h"
#include "frc846/util/math.h"

BracerCommand::BracerCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      bracer_(container.bracer_),
      super_(container.super_structure_) {
  AddRequirements({&bracer_});
  SetName("bracer_command");
}

void BracerCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  BracerTarget target = bracer_.ZeroTarget();

  if (ci_readings_.stageOfTrap != 0) {
    target.state = BracerState::kExtend;
  } else {
    target.state = BracerState::kRetract;
  }

  ci_readings_.stageOfTrap = std::max(ci_readings_.stageOfTrap, 0);

  if ((ci_readings_.stageOfTrap == 0 && prev_ci_readings_.stageOfTrap != 0) ||
      (ci_readings_.stageOfTrap != 0 && prev_ci_readings_.stageOfTrap == 0)) {
    counter = 120;
  }

  if (counter >= 1)
    counter -= 1;
  else
    target.state = BracerState::kStow;

  prev_ci_readings_ = ci_readings_;
  bracer_.SetTarget(target);
}

bool BracerCommand::IsFinished() { return false; }