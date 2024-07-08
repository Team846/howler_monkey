#include "subsystems/hardware/wrist.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"
#include "units/math.h"

WristSubsystem::WristSubsystem(bool init)
    : frc846::Subsystem<WristReadings, WristTarget>{"wrist", init} {
  if (init) {
    wrist_esc_.Configure(frc846::control::REVSparkType::kSparkMAX,
                         {frc846::control::kPositionData});

    wrist_esc_.ZeroEncoder(wrist_home_offset_.value());
  }
}

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
    double output = dyFPID.calculate(*pos, readings().wrist_position,
                                     wrist_esc_.GetVelocityPercentage(),
                                     config_helper_.updateAndGetGains());

    wrist_esc_.WriteDC(output);

    target_wrist_pos_graph.Graph(*pos);

  } else if (auto output = std::get_if<double>(&target.wrist_output)) {
    wrist_esc_.WriteDC(*output);

    target_wrist_duty_cycle_graph.Graph(*output);
  }
}