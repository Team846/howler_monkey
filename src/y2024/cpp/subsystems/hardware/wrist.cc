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
  target.wrist_output = wrist_home_offset_.value();
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

  readings.wrist_position = wrist_esc_.GetPosition();

  frc846::util::ShareTables::SetDouble("wrist_position",
                                       readings.wrist_position.to<double>());

  wrist_pos_graph.Graph(readings.wrist_position);

  auto target_output = GetTarget().wrist_output;
  if (auto target_angle = std::get_if<units::degree_t>(&target_output)) {
    wrist_error_graph.Graph(*target_angle - readings.wrist_position);
  }

  readings.wrist_velocity = wrist_esc_.GetVelocity();

  return readings;
}

void WristSubsystem::WriteToHardware(WristTarget target) {
  wrist_weight_pos_graph.Graph(
      1_deg * frc846::util::ShareTables::GetDouble("pivot_position") -
      GetReadings().wrist_position + wrist_cg_offset_.value());

  hard_limits_.OverrideLimits(target.override_limits);
  if (auto pos = std::get_if<units::degree_t>(&target.wrist_output)) {
    double output = dyFPID.calculate(*pos, GetReadings().wrist_position,
                                     wrist_esc_.GetVelocityPercentage(),
                                     gains_prefs_dyFPID.get());
    // if (units::math::abs(*pos - readings().wrist_position) < 5_deg) {
    //   output = dyFPIDClose.calculate(*pos, readings().wrist_position,
    //                                  wrist_esc_.GetVelocityPercentage(),
    //                                  config_helper_.updateAndGetGains());
    // }

    wrist_esc_.WriteDC(output);

    target_wrist_pos_graph.Graph(*pos);

  } else if (auto output = std::get_if<double>(&target.wrist_output)) {
    if (units::math::abs(wrist_esc_.GetVelocity()) >
        homing_max_speed_.value()) {
      *output = (*output) / homing_dc_cut_.value();
    }
    wrist_esc_.WriteDC(*output);

    target_wrist_duty_cycle_graph.Graph(*output);
  }
}