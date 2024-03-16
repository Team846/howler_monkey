#include "subsystems/scorer.h"
#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

ScorerSubsystem::ScorerSubsystem(bool init)
    : frc846::Subsystem<ScorerReadings, ScorerTarget>{"scorer", init},
    note_detection{intake_shooter_esc_.esc_.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)},
    note_detection_other{intake_shooter_esc_.esc_.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)} {
    if (init) {
        intake_shooter_esc_.Setup(&intake_esc_gains_, false);
        intake_shooter_esc_.SetupConverter(1_tr);

        shooter_esc_one_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus0, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus2, 
          rev::CANSparkBase::PeriodicFrame::kStatus3,
          rev::CANSparkBase::PeriodicFrame::kStatus1,
          rev::CANSparkBase::PeriodicFrame::kStatus5});
        shooter_esc_two_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus0, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus2, 
          rev::CANSparkBase::PeriodicFrame::kStatus3,
          rev::CANSparkBase::PeriodicFrame::kStatus1,
          rev::CANSparkBase::PeriodicFrame::kStatus5});
        intake_shooter_esc_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus2, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus3});

        shooter_esc_one_.Setup(&shooter_esc_gains_, true, frc846::control::kCoast);
        shooter_esc_one_.SetupConverter(1_tr);

        shooter_esc_two_.Setup(&shooter_esc_gains_, false, frc846::control::kCoast);
        shooter_esc_two_.SetupConverter(1_tr);

        note_detection.EnableLimitSwitch(true);


        note_detection_other.EnableLimitSwitch(false);

    }
}

ScorerTarget ScorerSubsystem::ZeroTarget() const {
  ScorerTarget target;
  target.target_state = kIdle;
  return target;
}

ScorerTarget ScorerSubsystem::MakeTarget(ScorerState target_state) {
  ScorerTarget target;
  target.target_state = target_state;
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
  ScorerReadings readings;
  has_piece_ = note_detection.Get();

  frc846::util::ShareTables::SetBoolean("scorer_has_piece", note_detection.Get());
  readings_has_piece_graph.Graph(has_piece_);

  readings.kLeftErrorPercent = (shooter_speed_.value().to<double>() * (1 + spin_.value()) - shooter_esc_one_.GetVelocity().to<double>()) / (shooter_speed_.value().to<double>() * (1 + spin_.value()));

  readings_shooting_speed_left_graph.Graph(shooter_esc_one_.GetVelocity().to<double>());

  readings_intake_speed_.Graph(intake_shooter_esc_.GetVelocity().to<double>());

  return readings;
}

void ScorerSubsystem::DirectWrite(ScorerTarget target) {
  if (target.target_state == kIntake) {
    if (!note_detection.IsLimitSwitchEnabled()) {
      note_detection.EnableLimitSwitch(true);
    };
    frc846::util::ShareTables::SetBoolean("scorer_has_piece", note_detection.Get());
    intake_shooter_esc_.Write(frc846::control::ControlMode::Velocity, intake_speed_.value() +
       frc846::util::ShareTables::GetDouble("velocity") * 1_tps / (1.8 * 3.1415926 / 12.0));
    shooter_esc_one_.Write(frc846::control::ControlMode::Percent, 0);
    shooter_esc_two_.Write(frc846::control::ControlMode::Percent, 0);
  } else if (target.target_state == kShoot) {
    if (has_piece_ == true || note_detection.IsLimitSwitchEnabled()) {
      note_detection.EnableLimitSwitch(false);
      // has_piece_ = false;
    }
    note_detection.EnableLimitSwitch(false);

    frc846::util::ShareTables::SetBoolean("scorer_has_piece", false);

    intake_shooter_esc_.Write(frc846::control::ControlMode::Velocity, intake_feed_speed_.value());

    shooter_esc_one_.Write(frc846::control::ControlMode::Velocity, shooter_speed_.value() * (1 + spin_.value()));
    shooter_esc_two_.Write(frc846::control::ControlMode::Velocity, shooter_speed_.value() * (1 - spin_.value()));
  } else if (target.target_state == kSpinUp) {
    intake_shooter_esc_.Write(frc846::control::ControlMode::Percent, 0);
    shooter_esc_one_.Write(frc846::control::ControlMode::Velocity, shooter_speed_.value() * (1 + spin_.value()));
    shooter_esc_two_.Write(frc846::control::ControlMode::Velocity, shooter_speed_.value() * (1 - spin_.value()));
  } else if (target.target_state == kRelease) {
    // note_detection.EnableLimitSwitch(false);

    intake_shooter_esc_.Write(frc846::control::ControlMode::Percent, release_speed_.value());
    shooter_esc_one_.Write(frc846::control::ControlMode::Percent, 0);
    shooter_esc_two_.Write(frc846::control::ControlMode::Percent, 0);

    // has_piece_ = false;
  } else {
    frc846::util::ShareTables::SetBoolean("scorer_has_piece", note_detection.Get());
    intake_shooter_esc_.Write(frc846::control::ControlMode::Percent, 0);
    shooter_esc_one_.Write(frc846::control::ControlMode::Percent, 0);
    shooter_esc_two_.Write(frc846::control::ControlMode::Percent, 0);
  }
}