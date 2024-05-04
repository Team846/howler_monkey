#include "commands/teleop/wrist_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/hardware/swerve_module.h"

WristCommand::WristCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      wrist_(container.wrist_),
      pivot_(container.pivot_),
      drivetrain_(container.drivetrain_),
      shooter_(container.shooter_),
      super_(container.super_structure_),
      vision_(container.vision_) {
  AddRequirements({&wrist_});
  SetName("wrist_command");
}

void WristCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  WristTarget wrist_target = wrist_.ZeroTarget();

  units::degree_t adj = 0_deg;

  if (std::abs(ci_readings_.wrist_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    adj = wrist_.max_adjustment_rate_.value() / 50.0_Hz *
          ci_readings_.wrist_manual_adjust;
  }

  frc846::util::ShareTables::SetString("shooting_state", "kNone");

  if (ci_readings_.stageOfTrap != 0) {
    // TRAP LOGIC TODO FIX

  } else if (ci_readings_.running_prep_shoot) {
    units::degree_t pivotAngle = pivot_.readings().pivot_position;
    auto pivotTarget = pivot_.GetTarget();
    if (auto *pivotTargetAngle =
            std::get_if<units::angle::degree_t>(&pivotTarget.pivot_output)) {
      pivotAngle = *pivotTargetAngle;
    }

    msequencer_.execute("prep_shoot",
                        {{[&]() -> void {
                            wrist_target.wrist_output =
                                90_deg + pivotAngle -
                                pivot_.pivot_home_offset_.value() -
                                wrist_.wrist_home_offset_.value() +
                                super_.getShootSetpoint().wrist;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { ms_adj = 0_deg; });
  } else if (ci_readings_.running_super_shoot) {
    VisionReadings vision_readings = vision_.readings();
    double shooting_dist = vision_readings.est_dist_from_speaker.to<double>();

    shooting_dist =
        shooting_dist + super_.teleop_shooter_x_.value().to<double>() / 12.0;

    units::degree_t theta =
        shooting_calculator
            .calculateLaunchAngles(
                shooter_.shooting_exit_velocity_.value(), shooting_dist,
                vision_readings.velocity_in_component,
                vision_readings.velocity_orth_component,
                super_.teleop_shooter_height_.value().to<double>() / 12.0)
            .launch_angle;

    if (theta >= 0.07_deg) {
      frc846::util::ShareTables::SetString("shooting_state", "kReady");
    } else {
      frc846::util::ShareTables::SetString("shooting_state", "kUnready");
    }

    msequencer_.execute(
        "super_shoot",
        {{[&]() -> void {
            wrist_target.wrist_output =
                90_deg +
                units::degree_t(
                    frc846::util::ShareTables::GetDouble("pivot_position")) -
                pivot_.pivot_home_offset_.value() -
                wrist_.wrist_home_offset_.value() + units::degree_t(theta);
          },
          [&]() -> bool { return false; }}},
        [&]() -> void { ms_adj = 0_deg; });
  } else if (ci_readings_.running_source) {
    msequencer_.execute("source",
                        {{[&]() -> void {
                            wrist_target.wrist_output =
                                super_.getSourceSetpoint().wrist;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { ms_adj = 0_deg; });
  } else if (ci_readings_.running_intake) {
    msequencer_.execute("intake",
                        {{[&]() -> void {
                            wrist_target.wrist_output =
                                super_.getIntakeSetpoint().wrist;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { ms_adj = 0_deg; });
  } else if (ci_readings_.running_amp) {
    msequencer_.execute("amp",
                        {{[&]() -> void {
                            wrist_target.wrist_output =
                                super_.getAmpSetpoint().wrist;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { ms_adj = 0_deg; });
  } else if (ci_readings_.running_pass) {
    msequencer_.execute("pass",
                        {{[&]() -> void {
                            wrist_target.wrist_output =
                                super_.getIntakeSetpoint().wrist;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { ms_adj = 0_deg; });
  } else {
    msequencer_.execute("stow",
                        {{[&]() -> void {
                            wrist_target.wrist_output =
                                super_.getStowSetpoint().wrist;
                          },
                          [&]() -> bool { return false; }}},
                        [&]() -> void { ms_adj = 0_deg; });
  }

  if (auto *wristTarget =
          std::get_if<units::degree_t>(&wrist_target.wrist_output)) {
    wrist_target.wrist_output = *wristTarget + ms_adj;
  }

  if (auto *wristTarget =
          std::get_if<units::degree_t>(&wrist_target.wrist_output)) {
    auto nextSumOutOfBounds = InverseKinematics::sumOutOfBounds(
        InverseKinematics::degree_toCoordinate(
            {frc846::util::ShareTables::GetDouble("pivot_position"),
             (*wristTarget + ms_adj + adj).to<double>(),
             frc846::util::ShareTables::GetDouble("telescope_extension")}));
    auto currentSumOutOfBounds = InverseKinematics::sumOutOfBounds(
        InverseKinematics::degree_toCoordinate(
            {frc846::util::ShareTables::GetDouble("pivot_position"),
             (*wristTarget + ms_adj).to<double>(),
             frc846::util::ShareTables::GetDouble("telescope_extension")}));
    if (nextSumOutOfBounds < 0.05 ||
        nextSumOutOfBounds <= currentSumOutOfBounds) {
      ms_adj += adj;
    }
    wrist_target.wrist_output = *wristTarget + ms_adj;
  }

  prev_ci_readings_ = ci_readings_;
  wrist_.SetTarget(wrist_target);
}

bool WristCommand::IsFinished() { return false; }