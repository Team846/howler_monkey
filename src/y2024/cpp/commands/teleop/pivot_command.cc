#include "commands/teleop/pivot_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/swerve_module.h"

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

  units::degree_t adj = 0_deg;

  if (std::abs(ci_readings_.pivot_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    adj = pivot_.max_adjustment_rate_.value() / 50.0_Hz *
          ci_readings_.pivot_manual_adjust;
  }

  if (ci_readings_.stageOfTrap != 0) {
    // if trap stage == 1, pivot -> preclimb pos
    // if trap stage == 2, pivot duty cycle down until both hooks engaged
    // if trap stage == 3, trap sequence
    // TRAP LOGIC TODO

  } else if (ci_readings_.running_prep_shoot ||
             ci_readings_.running_super_shoot) {
    msequencer_.execute("prep_shoot",
                        {{[&]() -> void {
                            pivot_target.pivot_output =
                                super_.getShootSetpoint().pivot;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mpiv_adj = 0_deg; });
  } else if (ci_readings_.running_source) {
    msequencer_.execute("source",
                        {{[&]() -> void {
                            pivot_target.pivot_output =
                                super_.getSourceSetpoint().pivot;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mpiv_adj = 0_deg; });
  } else if (ci_readings_.running_intake) {
    msequencer_.execute("intake",
                        {{[&]() -> void {
                            pivot_target.pivot_output =
                                super_.getIntakeSetpoint().pivot;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mpiv_adj = 0_deg; });
  } else if (ci_readings_.running_amp) {
    msequencer_.execute("amp",
                        {{[&]() -> void {
                            pivot_target.pivot_output =
                                super_.getAmpSetpoint().pivot;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mpiv_adj = 0_deg; });
  } else if (ci_readings_.running_pass) {
    msequencer_.execute("pass",
                        {{[&]() -> void {
                            pivot_target.pivot_output =
                                super_.getIntakeSetpoint().pivot;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mpiv_adj = 0_deg; });
  } else {
    msequencer_.execute("stow",
                        {{[&]() -> void {
                            pivot_target.pivot_output =
                                super_.getStowSetpoint().pivot;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mpiv_adj = 0_deg; });
  }

  if (auto *pivotTarget =
          std::get_if<units::degree_t>(&pivot_target.pivot_output)) {
    auto nextSumOutOfBounds = InverseKinematics::sumOutOfBounds(
        InverseKinematics::degree_toCoordinate(
            {(*pivotTarget + mpiv_adj + adj).to<double>(),
             frc846::util::ShareTables::GetDouble("wrist_position"),
             frc846::util::ShareTables::GetDouble("telescope_extension")}));
    auto currentSumOutOfBounds = InverseKinematics::sumOutOfBounds(
        InverseKinematics::degree_toCoordinate(
            {(*pivotTarget + mpiv_adj).to<double>(),
             frc846::util::ShareTables::GetDouble("wrist_position"),
             frc846::util::ShareTables::GetDouble("telescope_extension")}));
    if (nextSumOutOfBounds < 0.05 ||
        nextSumOutOfBounds <= currentSumOutOfBounds) {
      mpiv_adj += adj;
    }
    pivot_target.pivot_output = *pivotTarget + mpiv_adj;
  }

  prev_ci_readings_ = ci_readings_;
  pivot_.SetTarget(pivot_target);
}

bool PivotCommand::IsFinished() { return false; }