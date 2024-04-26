#include "commands/teleop/wrist_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "subsystems/swerve_module.h"

WristCommand::WristCommand(RobotContainer &container)
    : control_input_(container.control_input_),
      wrist_(container.wrist_),
      pivot_(container.pivot_),
      drivetrain_(container.drivetrain_),
      scorer_(container.scorer_),
      super_(container.super_structure_),
      vision_(container.vision_) {
  AddRequirements({&wrist_});
  SetName("wrist_command");
}

void WristCommand::Execute() {
  ControlInputReadings ci_readings_{control_input_.readings()};

  WristTarget wrist_target = wrist_.ZeroTarget();

  if (std::abs(ci_readings_.wrist_manual_adjust) >
      super_.manual_control_deadband_.value()) {
    ms_adj += wrist_.max_adjustment_rate_.value() / 50.0_Hz *
              ci_readings_.wrist_manual_adjust;
  }

  frc846::util::ShareTables::SetString("shooting_state", "kNone");

  if (ci_readings_.stageOfTrap != 0) {
    // TRAP LOGIC TODO FIX

    if (prev_ci_readings_.stageOfTrap == 0) ms_adj = 0_deg;
    wristHasRun = true;
  } else if (ci_readings_.running_prep_shoot) {
    units::degree_t pivotAngle = pivot_.readings().pivot_position;
    auto pivotTarget = pivot_.GetTarget();
    if (auto *pivotTargetAngle =
            std::get_if<units::angle::degree_t>(&pivotTarget.pivot_output)) {
      pivotAngle = *pivotTargetAngle;
    }

    wrist_target.wrist_output =
        90_deg + pivotAngle - pivot_.pivot_home_offset_.value() -
        wrist_.wrist_home_offset_.value() + super_.getShootSetpoint().wrist;

    if (!prev_ci_readings_.running_prep_shoot) ms_adj = 0_deg;
    wristHasRun = true;
  } else if (ci_readings_.running_super_shoot) {
    VisionReadings vision_readings = vision_.readings();
    double shooting_dist = vision_readings.est_dist_from_speaker.to<double>();

    shooting_dist =
        shooting_dist + super_.teleop_shooter_x_.value().to<double>() / 12.0;

    units::degree_t theta =
        shooting_calculator
            .calculateLaunchAngles(
                scorer_.shooting_exit_velocity_.value(), shooting_dist,
                vision_readings.velocity_in_component,
                vision_readings.velocity_orth_component,
                super_.teleop_shooter_height_.value().to<double>() / 12.0)
            .launch_angle;

    if (theta >= 0.07_deg) {
      frc846::util::ShareTables::SetString("shooting_state", "kReady");
    } else {
      frc846::util::ShareTables::SetString("shooting_state", "kUnready");
    }

    wrist_target.wrist_output =
        90_deg +
        units::degree_t(
            frc846::util::ShareTables::GetDouble("pivot_position")) -
        pivot_.pivot_home_offset_.value() - wrist_.wrist_home_offset_.value() +
        units::degree_t(theta);

    if (!prev_ci_readings_.running_super_shoot) ms_adj = 0_deg;
    wristHasRun = true;
  } else if (ci_readings_.running_source) {
    wrist_target.wrist_output = super_.getSourceSetpoint().wrist;

    if (!prev_ci_readings_.running_source) ms_adj = 0_deg;
    wristHasRun = true;
  } else if (ci_readings_.running_intake) {
    wrist_target.wrist_output = super_.getIntakeSetpoint().wrist;

    if (!prev_ci_readings_.running_intake) ms_adj = 0_deg;
    wristHasRun = true;
  } else if (ci_readings_.running_amp) {
    wrist_target.wrist_output = super_.getAmpSetpoint().wrist;

    if (!prev_ci_readings_.running_amp) ms_adj = 0_deg;
    wristHasRun = true;
  } else if (ci_readings_.running_pass) {
    wrist_target.wrist_output = super_.getIntakeSetpoint().wrist;

    if (!prev_ci_readings_.running_pass) ms_adj = 0_deg;
    wristHasRun = true;
  } else {
    wrist_target.wrist_output = super_.getStowSetpoint().wrist;

    if (wristHasRun) ms_adj = 0_deg;
    wristHasRun = false;
  }

  if (auto *wristTarget =
          std::get_if<units::degree_t>(&wrist_target.wrist_output)) {
    wrist_target.wrist_output = *wristTarget + ms_adj;
  }

  prev_ci_readings_ = ci_readings_;
  wrist_.SetTarget(wrist_target);
}

bool WristCommand::IsFinished() { return false; }