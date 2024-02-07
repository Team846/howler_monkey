#include "subsystems/wrist.h"
#include "frc846/control/control.h"

WristSubsystem::WristSubsystem(bool init)
    : frc846::Subsystem<WristReadings, WristTarget>{"wrist", init} {
    if (init) {
        wrist_esc_.Setup(&wrist_esc_gains_, false);
        wrist_esc_.SetupConverter(1_deg);
    }
}

WristTarget WristSubsystem::ZeroTarget() const {
  WristTarget target;
  target.wrist_output = 0.0;
  return target;
}

WristTarget WristSubsystem::MakeTarget(
    std::variant<units::degree_t, double> wrist_out) {
      WristTarget target;
      target.wrist_output = wrist_out;
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

  wrist_pos_graph.Graph(readings.wrist_position);

  return readings;
}

void WristSubsystem::PositionWrist(WristTarget target) {
  if (auto pos = std::get_if<units::degree_t>(&target.wrist_output)) {
    wrist_esc_.Write(frc846::control::ControlMode::Position, *pos);

    target_wrist_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.wrist_output)) {
    wrist_esc_.Write(frc846::control::ControlMode::Percent, *output);

    target_wrist_duty_cycle_graph.Graph(*output);
  }
}

void WristSubsystem::DirectWrite(WristTarget target) {
  PositionWrist(target);
}