#include "subsystems/hardware/wrist.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"
#include "units/math.h"

WristSubsystem::WristSubsystem(bool init)
    : frc846::Subsystem<WristReadings, WristTarget>{"wrist", init} {
  if (init) {
    wrist_esc_.Configure({frc846::control::kPositionData});
  }
}

WristTarget WristSubsystem::ZeroTarget() const {
  WristTarget target;
  target.wrist_output = 0.0;
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

WristReadings WristSubsystem::GetNewReadings() {
  WristReadings readings;

  readings.wrist_position = wrist_esc_.GetPosition();

  frc846::util::ShareTables::SetDouble("wrist_position",
                                       readings.wrist_position.to<double>());

  wrist_pos_graph.Graph(readings.wrist_position);

  if (auto target_angle = std::get_if<units::degree_t>(&target_.wrist_output)) {
    wrist_error_graph.Graph(*target_angle - readings.wrist_position);
  }

  return readings;
}

void WristSubsystem::DirectWrite(WristTarget target) {
  if (auto pos = std::get_if<units::degree_t>(&target.wrist_output)) {
    units::degree_t pivot_angle =
        units::degree_t(
            frc846::util::ShareTables::GetDouble("pivot_position")) -
        17_deg;
    units::degree_t wrist_angle =
        readings().wrist_position +
        (180_deg - (wrist_home_offset_.value() + wrist_cg_offset_.value()));
    units::degree_t shooting_angle = 180_deg + pivot_angle - wrist_angle;

    double f = k_.value() * units::math::abs(units::math::cos(shooting_angle));

    double p = p_.value() * (*pos - readings().wrist_position).to<double>();

    double d = d_.value() * (wrist_esc_.GetVelocity()).to<double>();

    wrist_esc_.WriteDC(f + p - d);

    target_wrist_pos_graph.Graph(*pos);

  } else if (auto output = std::get_if<double>(&target.wrist_output)) {
    wrist_esc_.WriteDC(*output);

    target_wrist_duty_cycle_graph.Graph(*output);
  }
}