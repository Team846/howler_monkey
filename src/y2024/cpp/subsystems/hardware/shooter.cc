#include "subsystems/hardware/shooter.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

ShooterSubsystem::ShooterSubsystem(bool init)
    : frc846::Subsystem<ShooterReadings, ShooterTarget>{"shooter", init} {
  if (init) {
    shooter_esc_one_.OverrideInvert();

    shooter_esc_one_.Init(frc846::control::REVSparkType::kSparkFLEX);
    shooter_esc_two_.Init(frc846::control::REVSparkType::kSparkFLEX);
  }
}

void ShooterSubsystem::Setup() {
  shooter_esc_one_.Configure({frc846::control::DataTag::kVelocityData});
  shooter_esc_two_.Configure({frc846::control::DataTag::kVelocityData});
}

ShooterTarget ShooterSubsystem::ZeroTarget() const {
  ShooterTarget target;
  target.target_state = kIdle;
  return target;
}

bool ShooterSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;
    FRC846_VERIFY(shooter_esc_one_.VerifyConnected(), ok,
                  "shooter one not connected");
    FRC846_VERIFY(shooter_esc_two_.VerifyConnected(), ok,
                  "shooter two not connected");
    return ok;
  }
  return true;
}

ShooterReadings ShooterSubsystem::GetNewReadings() {
  ShooterReadings readings;

  double shooter_one_velocity = shooter_esc_one_.GetVelocity().to<double>();
  double shooter_two_velocity = shooter_esc_two_.GetVelocity().to<double>();

  double leftError =
      (shooter_speed_.value().to<double>() * (1 + spin_.value()) -
       shooter_one_velocity) /
      (shooter_speed_.value().to<double>() * (1 + spin_.value()));
  double rightError =
      (shooter_speed_.value().to<double>() * (1 - spin_.value()) -
       shooter_two_velocity) /
      (shooter_speed_.value().to<double>() * (1 - spin_.value()));

  readings_shooting_speed_left_graph.Graph(shooter_one_velocity);
  readings_shooting_speed_right_graph.Graph(shooter_two_velocity);
  shooter_left_error_graph_.Graph(leftError);
  shooter_right_error_graph_.Graph(rightError);

  readings.error_percent = (leftError + rightError) / 2.0;

  return readings;
}

void ShooterSubsystem::DirectWrite(ShooterTarget target) {
  if (target.target_state == kRun) {
    shooter_esc_one_.WriteDC(
        braking_v_FPID.calculate(shooter_speed_.value() * (1 + spin_.value()),
                                 shooter_esc_one_.GetVelocity(),
                                 shooter_esc_one_.GetVelocityPercentage(),
                                 config_helper_.updateAndGetGains()));
    shooter_esc_two_.WriteDC(
        braking_v_FPID.calculate(shooter_speed_.value() * (1 - spin_.value()),
                                 shooter_esc_two_.GetVelocity(),
                                 shooter_esc_two_.GetVelocityPercentage(),
                                 config_helper_.updateAndGetGains()));
  } else {
    shooter_esc_one_.WriteDC(0.0);
    shooter_esc_two_.WriteDC(0.0);
  }
}