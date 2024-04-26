#include "commands/teleop/telescope_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/swerve_module.h"

TelescopeCommand::TelescopeCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      telescope_(container.telescope_),
      super_(container.super_structure_) {
  AddRequirements({&telescope_});
  SetName("telescope_command");
}

void TelescopeCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  TelescopeTarget telescope_target = telescope_.ZeroTarget();

  if (std::abs(ci_readings_.telescope_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    mtele_adj += telescope_.max_adjustment_rate_.value() / 50.0_Hz *
                 ci_readings_.telescope_manual_adjust;
  }

  if (ci_readings_.stageOfTrap != 0) {
    // TRAP LOGIC TODO

    if (prev_ci_readings_.stageOfTrap == 0) mtele_adj = 0_in;
    telescopeHasRun = true;
  } else if (ci_readings_.running_prep_shoot ||
             ci_readings_.running_super_shoot) {
    telescope_target.extension = super_.getShootSetpoint().telescope;

    if (!prev_ci_readings_.running_prep_shoot) mtele_adj = 0_in;
    telescopeHasRun = true;
  } else if (ci_readings_.running_source) {
    telescope_target.extension = super_.getSourceSetpoint().telescope;

    if (!prev_ci_readings_.running_source) mtele_adj = 0_in;
    telescopeHasRun = true;
  } else if (ci_readings_.running_intake) {
    telescope_target.extension = super_.getIntakeSetpoint().telescope;

    if (!prev_ci_readings_.running_intake) mtele_adj = 0_in;
    telescopeHasRun = true;
  } else if (ci_readings_.running_amp) {
    telescope_target.extension = super_.getAmpSetpoint().telescope;

    if (!prev_ci_readings_.running_amp) mtele_adj = 0_in;
    telescopeHasRun = true;
  } else if (ci_readings_.running_pass) {
    telescope_target.extension = super_.getIntakeSetpoint().telescope;

    if (!prev_ci_readings_.running_pass) mtele_adj = 0_in;
    telescopeHasRun = true;
  } else {
    telescope_target.extension = super_.getStowSetpoint().telescope;

    if (telescopeHasRun) mtele_adj = 0.0_in;
    telescopeHasRun = false;
  }

  if (auto *telescopeTarget =
          std::get_if<units::inch_t>(&telescope_target.extension)) {
    telescope_target.extension = *telescopeTarget + mtele_adj;
  }

  prev_ci_readings_ = ci_readings_;
  telescope_.SetTarget(telescope_target);
}

bool TelescopeCommand::IsFinished() { return false; }