#include "subsystems/hardware/intake.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

IntakeSubsystem::IntakeSubsystem(bool init)
    : frc846::Subsystem<IntakeReadings, IntakeTarget>{"intake", init},
      note_detection{intake_esc_.esc_.GetForwardLimitSwitch(
          rev::SparkLimitSwitch::Type::kNormallyOpen)},
      note_detection_other{intake_esc_.esc_.GetReverseLimitSwitch(
          rev::SparkLimitSwitch::Type::kNormallyOpen)} {
  if (init) {
    intake_esc_.Setup(&intake_esc_gains_, true);
    intake_esc_.SetupConverter(1_tr);

    // intake_esc_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus2,
    //   rev::CANSparkBase::PeriodicFrame::kStatus4,
    //   rev::CANSparkBase::PeriodicFrame::kStatus3});

    note_detection.EnableLimitSwitch(true);

    note_detection_other.EnableLimitSwitch(false);
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
  has_piece_ = note_detection.Get();

  frc846::util::ShareTables::SetBoolean("scorer_has_piece",
                                        note_detection.Get());
  readings_has_piece_graph.Graph(has_piece_);

  double intake_velocity = intake_esc_.GetVelocity().to<double>();

  readings_intake_speed_.Graph(intake_velocity);

  if (std::abs(lastIntakeTarget) > 0.05) {
    intake_error_.Graph((lastIntakeTarget - intake_velocity) /
                        lastIntakeTarget);
  } else {
    intake_error_.Graph(-intake_velocity);
  }

  intake_current_draw_.Graph(intake_esc_.GetCurrent().to<double>());

  return readings;
}

void IntakeSubsystem::DirectWrite(IntakeTarget target) {
  if (target.target_state == kIntake) {
    if (!note_detection.IsLimitSwitchEnabled()) {
      note_detection.EnableLimitSwitch(true);
    };
    frc846::util::ShareTables::SetBoolean("scorer_has_piece",
                                          note_detection.Get());
    intake_esc_.Write(frc846::control::ControlMode::Velocity,
                      intake_speed_.value() +
                          frc846::util::ShareTables::GetDouble("velocity") *
                              1_tps / (1.8 * 3.1415926 / 12.0));
  } else if (target.target_state == kFeed) {
    if (has_piece_ == true || note_detection.IsLimitSwitchEnabled()) {
      note_detection.EnableLimitSwitch(false);
    }

    intake_esc_.Write(frc846::control::ControlMode::Velocity,
                      intake_feed_speed_.value());
  } else if (target.target_state == kPull) {
    if (note_detection.IsLimitSwitchEnabled())
      note_detection.EnableLimitSwitch(false);
    if (note_detection_other.IsLimitSwitchEnabled())
      note_detection_other.EnableLimitSwitch(false);

    intake_esc_.Write(frc846::control::ControlMode::Percent,
                      retract_speed_.value());

  } else if (target.target_state == kRelease) {
    if (note_detection.IsLimitSwitchEnabled())
      note_detection.EnableLimitSwitch(false);
    if (note_detection_other.IsLimitSwitchEnabled())
      note_detection_other.EnableLimitSwitch(false);

    intake_esc_.Write(frc846::control::ControlMode::Percent,
                      release_speed_.value());
  } else {
    frc846::util::ShareTables::SetBoolean("scorer_has_piece",
                                          note_detection.Get());
    intake_esc_.Write(frc846::control::ControlMode::Percent, 0);

    lastIntakeTarget = 0.0;
  }
}