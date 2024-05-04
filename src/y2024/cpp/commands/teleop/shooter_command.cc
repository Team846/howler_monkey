#include "commands/teleop/shooter_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/swerve_module.h"

ShooterCommand::ShooterCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      shooter_(container.shooter_),
      super_(container.super_structure_) {
  AddRequirements({&shooter_});
  SetName("shooter_command");
}

void ShooterCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  ShooterTarget target = shooter_.ZeroTarget();

  if (ci_readings_.shooting) {
    target.target_state = ShooterState::kRun;
  } else if (ci_readings_.running_intake || ci_readings_.running_source) {
    target.target_state = ShooterState::kIdle;
  } else if (ci_readings_.running_pass) {
    target.target_state = ShooterState::kRun;
  } else if (ci_readings_.manual_feed) {
    target.target_state = ShooterState::kIdle;
  } else if (ci_readings_.manual_spin_up) {
    target.target_state = ShooterState::kRun;
  } else if (ci_readings_.eject) {
    target.target_state = ShooterState::kIdle;
  } else {
    target.target_state = ShooterState::kIdle;
  }

  prev_ci_readings_ = ci_readings_;

  shooter_.SetTarget(target);
}

bool ShooterCommand::IsFinished() { return false; }