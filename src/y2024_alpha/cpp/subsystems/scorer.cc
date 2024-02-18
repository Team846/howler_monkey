#include "subsystems/scorer.h"
#include "frc846/control/control.h"

ScorerSubsystem::ScorerSubsystem(bool init)
    : frc846::Subsystem<ScorerReadings, ScorerTarget>{"scorer", init},
    note_detection{intake_shooter_esc_.esc_.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)} {
    if (init) {
        intake_shooter_esc_.Setup(&shooter_esc_gains_, false);
        intake_shooter_esc_.SetupConverter(1_tr);

        note_detection.EnableLimitSwitch(true);
    }
}

ScorerTarget ScorerSubsystem::ZeroTarget() const {
  ScorerTarget target;
  target.run_intake = false;
  target.run_shooter = false;
  target.shooter_speed = 0.0_tps;
  return target;
}

ScorerTarget ScorerSubsystem::MakeTarget(bool intake, bool shoot, units::turns_per_second_t shoot_spd) {
  ScorerTarget target;
  target.run_intake = intake;
  target.run_shooter = shoot;
  target.shooter_speed = shoot_spd;
  return target;
}

bool ScorerSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;
    FRC846_VERIFY(intake_shooter_esc_.VerifyConnected(), ok, "not connected");
    return ok;
  }
  return true;
}

ScorerReadings ScorerSubsystem::GetNewReadings() {
  if (!has_piece_) {
    has_piece_ = note_detection.Get();
  }

  readings_has_piece_graph.Graph(has_piece_);

  ScorerReadings readings;
  return readings;
}

void ScorerSubsystem::DirectWrite(ScorerTarget target) {
  if (target.run_intake) {
    if (!note_detection.IsLimitSwitchEnabled()) {
      note_detection.EnableLimitSwitch(true);
    };
    intake_shooter_esc_.Write(frc846::control::ControlMode::Percent, intake_speed_.value());
  } else if (target.run_shooter) {
    if (has_piece_ == true && note_detection.IsLimitSwitchEnabled()) {
      note_detection.EnableLimitSwitch(false);
    };
    intake_shooter_esc_.Write(frc846::control::ControlMode::Velocity, shooter_speed_.value());
  } else {
    intake_shooter_esc_.Write(frc846::control::ControlMode::Percent, 0);
  }
}