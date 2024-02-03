#include "subsystems/shintake.h"
#include "frc846/control/control.h"

ShintakeSubsystem::ShintakeSubsystem(bool init)
    : frc846::Subsystem<ShintakeReadings, ShintakeTarget>{"Shintake", init} {
    if (init) {
        intake_shooter_esc_.Setup(&shooter_esc_gains_, false);
        intake_shooter_esc_.SetupConverter(1_tr);

        note_detector_.SetLimitsRaw(200, 4090);
    }
}

ShintakeTarget ShintakeSubsystem::ZeroTarget() const {
  ShintakeTarget target;
  target.run_intake = false;
  target.run_shooter = false;
  target.shooter_speed = 0.0_tps;
  return target;
}

ShintakeTarget ShintakeSubsystem::MakeTarget(bool intake, bool shoot, units::turns_per_second_t shoot_spd) {
  ShintakeTarget target;
  target.run_intake = intake;
  target.run_shooter = shoot;
  target.shooter_speed = shoot_spd;
  return target;
}

bool ShintakeSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;
    FRC846_VERIFY(intake_shooter_esc_.VerifyConnected(), ok, "not connected");
    return ok;
  }
  return true;
}

ShintakeReadings ShintakeSubsystem::GetNewReadings() {
  if (!has_piece_) {
    has_piece_ = !note_detector_.GetTriggerState();
  }

  readings_has_piece_graph.Graph(has_piece_);

  ShintakeReadings readings;
  return readings;
}

void ShintakeSubsystem::DirectWrite(ShintakeTarget target) {
  if (target.run_intake && !has_piece_) {
    intake_shooter_esc_.Write(frc846::control::ControlMode::Percent, intake_speed_.value());
  } else if (target.run_shooter) {
    has_piece_ = false;
    intake_shooter_esc_.Write(frc846::control::ControlMode::Velocity, shooter_speed_.value());
  } else {
    intake_shooter_esc_.Write(frc846::control::ControlMode::Percent, 0);
  }
}