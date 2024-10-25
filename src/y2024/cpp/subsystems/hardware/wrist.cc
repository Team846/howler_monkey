#include "subsystems/hardware/wrist.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"
#include "units/math.h"

WristSubsystem::WristSubsystem(bool init)
    : frc846::robot::GenericSubsystem<WristReadings, WristTarget>{"wrist",
                                                                  init} {
  wrist_esc_.Init(frc846::control::REVSparkType::kSparkMAX);
  wrist_esc_.Configure({frc846::control::DataTag::kPositionData,
                        frc846::control::DataTag::kVelocityData});
  wrist_esc_.ZeroEncoder(wrist_home_offset_.value());
}

void WristSubsystem::Setup() {}

WristTarget WristSubsystem::ZeroTarget() const {
  WristTarget target;
  target.target_position = wrist_home_offset_.value();
  return target;
}

bool WristSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    FRC846_VERIFY(wrist_esc_.VerifyConnected(), ok, "not connected");

    return ok;
  }
  return true;
}

WristReadings WristSubsystem::ReadFromHardware() {
  WristReadings readings;

  readings.position = wrist_esc_.GetPosition();
  Graph("readings/position", readings.position.to<double>());

  readings.cg_position =
      1_deg * frc846::util::ShareTables::GetDouble("pivot_position") -
      GetReadings().position + wrist_cg_offset_.value();
  Graph("readings/cg_position", readings.cg_position.to<double>());

  Graph("readings/error",
        (GetTarget().target_position - readings.position).to<double>());

  Graph("readings/velocity", wrist_esc_.GetVelocity().to<double>());

  if (!hasZeroed) {
    if (units::math::abs(wrist_esc_.GetVelocity()) <
        homing_velocity_tolerance_.value()) {
      Graph("readings/within_homing_velocity", true);
      homing_counter_ += 1;
    } else {
      Graph("readings/within_homing_velocity", false);
      homing_counter_ = 0;
    }
    if (homing_counter_ > 30) {
      hasZeroed = true;
      wrist_esc_.ZeroEncoder(wrist_home_offset_.value());
      Log("Wrist homed successfully.");
    }
  }

  return readings;
}

void WristSubsystem::WriteToHardware(WristTarget target) {
  if (!hasZeroed) {
    hard_limits_.OverrideLimits(true);

    double output = homing_speed_.value();
    if (units::math::abs(wrist_esc_.GetVelocity()) >
        homing_max_speed_.value()) {
      output = output / homing_dc_cut_.value();
      Graph("target/homing_too_fast", true);
    } else {
      Graph("target/homing_too_fast", false);
    }
    Graph("target/homing_output", output);
    wrist_esc_.WriteDC(output);

  } else {
    hard_limits_.OverrideLimits(false);

    double output = dyFPID.calculate(
        target.target_position, GetReadings().position,
        wrist_esc_.GetVelocityPercentage(), gains_prefs_dyFPID.get());

    Graph("target/fpid_output", output);
    wrist_esc_.WriteDC(output);
  }
}