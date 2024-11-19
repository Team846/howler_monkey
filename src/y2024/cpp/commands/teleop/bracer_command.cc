#include "commands/teleop/bracer_command.h"

BracerCommand::BracerCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, BracerCommand>{
          container, "bracer_command"} {
  AddRequirements({&container_.bracer_});
}

void BracerCommand::OnInit() {}

void BracerCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  BracerTarget target = {kStow};

  // if (ci_readings_.stageOfTrap != 0) {
  //   target.state = BracerState::kExtend;
  // } else {
  //   target.state = BracerState::kRetract;
  // }

  ci_readings_.stageOfTrap = std::max(ci_readings_.stageOfTrap, 0);

  if ((ci_readings_.stageOfTrap == 0 && prev_ci_readings_.stageOfTrap != 0) ||
      (ci_readings_.stageOfTrap != 0 && prev_ci_readings_.stageOfTrap == 0)) {
    counter = 120;
  }

  if (counter >= 1)
    counter -= 1;
  else
    target.state = BracerState::kStow;

  if (container_.control_input_.operator_.GetReadings().right_trigger) {
    target.state = BracerState::kRetract;
  } else if (container_.control_input_.operator_.GetReadings().right_bumper) {
    target.state = BracerState::kExtend;
  }

  prev_ci_readings_ = ci_readings_;
  container_.bracer_.SetTarget(target);
}

void BracerCommand::OnEnd(bool interrupted) {}

bool BracerCommand::IsFinished() { return false; }