#include "commands/teleop/telescope_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/swerve_module.h"

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

  units::inch_t adj = 0_in;

  if (std::abs(ci_readings_.telescope_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    adj = telescope_.max_adjustment_rate_.value() / 50.0_Hz *
          ci_readings_.telescope_manual_adjust;
  }

  if (ci_readings_.stageOfTrap != 0) {
    // TRAP LOGIC TODO

  } else if (ci_readings_.running_prep_shoot ||
             ci_readings_.running_super_shoot) {
    msequencer_.execute("prep_shoot",
                        {{[&]() -> void {
                            telescope_target.extension =
                                super_.getShootSetpoint().telescope;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mtele_adj = 0_in; });
  } else if (ci_readings_.running_source) {
    msequencer_.execute("source",
                        {{[&]() -> void {
                            telescope_target.extension =
                                super_.getSourceSetpoint().telescope;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mtele_adj = 0_in; });
  } else if (ci_readings_.running_intake) {
    msequencer_.execute("intake",
                        {{[&]() -> void {
                            telescope_target.extension =
                                super_.getIntakeSetpoint().telescope;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mtele_adj = 0_in; });
  } else if (ci_readings_.running_amp) {
    msequencer_.execute("amp",
                        {{[&]() -> void {
                            telescope_target.extension =
                                super_.getAmpSetpoint().telescope;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mtele_adj = 0_in; });
  } else if (ci_readings_.running_pass) {
    msequencer_.execute("pass",
                        {{[&]() -> void {
                            telescope_target.extension =
                                super_.getIntakeSetpoint().telescope;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mtele_adj = 0_in; });
  } else {
    msequencer_.execute("stow",
                        {{[&]() -> void {
                            telescope_target.extension =
                                super_.getStowSetpoint().telescope;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { mtele_adj = 0_in; });
  }

  if (auto *telescopeTarget =
          std::get_if<units::inch_t>(&telescope_target.extension)) {
    auto nextSumOutOfBounds = InverseKinematics::sumOutOfBounds(
        InverseKinematics::degree_toCoordinate(
            {frc846::util::ShareTables::GetDouble("pivot_position"),
             frc846::util::ShareTables::GetDouble("wrist_position"),
             (*telescopeTarget + mtele_adj + adj).to<double>()}));
    auto currentSumOutOfBounds = InverseKinematics::sumOutOfBounds(
        InverseKinematics::degree_toCoordinate(
            {frc846::util::ShareTables::GetDouble("pivot_position"),
             frc846::util::ShareTables::GetDouble("wrist_position"),
             (*telescopeTarget + mtele_adj).to<double>()}));
    if (nextSumOutOfBounds < 0.05 ||
        nextSumOutOfBounds <= currentSumOutOfBounds) {
      mtele_adj += adj;
    }
    telescope_target.extension = *telescopeTarget + mtele_adj;
  }

  prev_ci_readings_ = ci_readings_;
  telescope_.SetTarget(telescope_target);
}

bool TelescopeCommand::IsFinished() { return false; }