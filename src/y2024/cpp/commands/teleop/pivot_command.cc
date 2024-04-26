#include "commands/teleop/pivot_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/swerve_module.h"

PivotCommand::PivotCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      pivot_(container.pivot_),
      super_(container.super_structure_) {
  AddRequirements({&pivot_});
  SetName("pivot_command");
}

void PivotCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  PivotTarget pivot_target = pivot_.ZeroTarget();

  if (std::abs(ci_readings_.pivot_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    mpiv_adj += pivot_.max_adjustment_rate_.value() / 50.0_Hz *
                ci_readings_.pivot_manual_adjust;
  }

  if (ci_readings_.stageOfTrap != 0) {
    // TRAP LOGIC TODO

    if (prev_ci_readings_.stageOfTrap == 0) mpiv_adj = 0_deg;
    pivotHasRun = true;
  } else if (ci_readings_.running_prep_shoot ||
             ci_readings_.running_super_shoot) {
    pivot_target.pivot_output = super_.getShootSetpoint().pivot;

    if (!prev_ci_readings_.running_prep_shoot) mpiv_adj = 0_deg;
    pivotHasRun = true;
  } else if (ci_readings_.running_source) {
    pivot_target.pivot_output = super_.getSourceSetpoint().pivot;

    if (!prev_ci_readings_.running_source) mpiv_adj = 0_deg;
    pivotHasRun = true;
  } else if (ci_readings_.running_intake) {
    pivot_target.pivot_output = super_.getIntakeSetpoint().pivot;

    if (!prev_ci_readings_.running_intake) mpiv_adj = 0_deg;
    pivotHasRun = true;
  } else if (ci_readings_.running_amp) {
    pivot_target.pivot_output = super_.getAmpSetpoint().pivot;

    if (!prev_ci_readings_.running_amp) mpiv_adj = 0_deg;
    pivotHasRun = true;
  } else if (ci_readings_.running_pass) {
    pivot_target.pivot_output = super_.getIntakeSetpoint().pivot;

    if (!prev_ci_readings_.running_pass) mpiv_adj = 0_deg;
    pivotHasRun = true;
  } else {
    pivot_target.pivot_output = super_.getStowSetpoint().pivot;

    if (pivotHasRun) mpiv_adj = 0_deg;
    pivotHasRun = false;
  }

  if (auto *pivotTarget =
          std::get_if<units::degree_t>(&pivot_target.pivot_output)) {
    pivot_target.pivot_output = *pivotTarget + mpiv_adj;
  }

  prev_ci_readings_ = ci_readings_;
  pivot_.SetTarget(pivot_target);
}

bool PivotCommand::IsFinished() { return false; }