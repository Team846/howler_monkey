#include "commands/teleop/intake_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/swerve_module.h"

IntakeCommand::IntakeCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      intake_(container.intake_),
      super_(container.super_structure_) {
  AddRequirements({&intake_});
  SetName("intake_command");
}

void IntakeCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  IntakeTarget target = intake_.ZeroTarget();

  if (ci_readings_.shooting) {
    target.target_state = IntakeState::kFeed;
  } else if (ci_readings_.running_intake || ci_readings_.running_source) {
    target.target_state = IntakeState::kIntake;
  } else if (ci_readings_.running_pass) {
    target.target_state = IntakeState::kIntake;
  } else if (ci_readings_.manual_feed) {
    target.target_state = IntakeState::kPull;
  } else if (ci_readings_.manual_spin_up) {
    target.target_state = IntakeState::kHold;
  } else if (ci_readings_.eject) {
    target.target_state = IntakeState::kRelease;
  } else {
    target.target_state = IntakeState::kHold;
  }

  prev_ci_readings_ = ci_readings_;

  intake_.SetTarget(target);
}

bool IntakeCommand::IsFinished() { return false; }