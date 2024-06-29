#include "subsystems/hardware/intake.h"

#include <units/math.h>

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"
#include "initializer_list"

IntakeSubsystem::IntakeSubsystem(bool init)
    : frc846::Subsystem<IntakeReadings, IntakeTarget>{"intake", init} {
  if (init) {
    intake_esc_.Configure(frc846::control::REVSparkType::kSparkMAX,
                          {frc846::control::DataTag::kVelocityData,
                           frc846::control::DataTag::kCurrentData,
                           frc846::control::DataTag::kFaultData});

    if (auto esc = intake_esc_.getESC()) {
      in_limit_switch.emplace(esc->GetForwardLimitSwitch(
          rev::SparkLimitSwitch::Type::kNormallyOpen));
      out_limit_switch.emplace(esc->GetReverseLimitSwitch(
          rev::SparkLimitSwitch::Type::kNormallyOpen));
      in_limit_switch.value().EnableLimitSwitch(true);
      out_limit_switch.value().EnableLimitSwitch(false);
    }
  }
}

IntakeTarget IntakeSubsystem::ZeroTarget() const {
  IntakeTarget target;
  target.target_state = kHold;
  return target;
}

bool IntakeSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;
    FRC846_VERIFY(intake_esc_.VerifyConnected(), ok, "not connected");
    return ok;
  }
  return true;
}

IntakeReadings IntakeSubsystem::GetNewReadings() {
  IntakeReadings readings;

  has_piece_ = in_limit_switch.has_value() && in_limit_switch.value().Get();

  frc846::util::ShareTables::SetBoolean("scorer_has_piece", has_piece_);
  readings_has_piece_graph.Graph(has_piece_);

  auto intake_velocity = intake_esc_.GetVelocity();

  readings_intake_speed_.Graph(intake_velocity.to<double>());

  if (units::math::abs(target_intaking_speed) > 0.05_fps) {
    intake_error_.Graph((target_intaking_speed - intake_velocity) /
                        target_intaking_speed);
  } else {
    intake_error_.Graph(0.0);
  }

  intake_current_draw_.Graph(intake_esc_.GetCurrent().to<double>());

  return readings;
}

void IntakeSubsystem::DirectWrite(IntakeTarget target) {
  target_intaking_speed = 0.0_fps;

  if (target.target_state == kIntake) {
    if (in_limit_switch.has_value() &&
        !in_limit_switch.value().IsLimitSwitchEnabled()) {
      in_limit_switch.value().EnableLimitSwitch(true);
    };

    target_intaking_speed =
        base_intake_speed_.value() +
        frc846::util::ShareTables::GetDouble("velocity") * 1_fps;
    intake_esc_.WriteVelocity(target_intaking_speed);

  } else if (target.target_state == kFeed) {
    if (in_limit_switch.has_value() &&
        in_limit_switch.value().IsLimitSwitchEnabled()) {
      in_limit_switch.value().EnableLimitSwitch(false);
    }

    intake_esc_.WriteVelocity(intake_feed_speed_.value());
  } else if (target.target_state == kPull) {
    if (in_limit_switch.has_value() &&
        in_limit_switch.value().IsLimitSwitchEnabled()) {
      in_limit_switch.value().EnableLimitSwitch(false);
    }

    intake_esc_.WriteDC(retract_speed_.value());

  } else if (target.target_state == kRelease) {
    if (out_limit_switch.has_value() &&
        out_limit_switch.value().IsLimitSwitchEnabled()) {
      out_limit_switch.value().EnableLimitSwitch(false);
    }

    intake_esc_.WriteDC(release_speed_.value());
  } else {
    intake_esc_.WriteDC(0.0);
  }
}