#include "subsystems/scoring_positioner.h"
#include "frcLib846/control/control.h"

ScoringPositionerSubsystem::ScoringPositionerSubsystem(bool init)
    : frcLib846::Subsystem<ScoringPositionerReadings, ScoringPositionerTarget>{"scoring_positioner", init} {
    if (init) {
        // pivot_tandem.AddESC(pivot_one_esc_.that());
        // pivot_tandem.AddESC(pivot_two_esc_.that());
        // pivot_tandem.AddESC(pivot_three_esc_.that());
        // pivot_tandem.AddESC(pivot_four_esc_.that());
        // pivot_tandem.SetupAll(&pivot_esc_gains_, {false, false, true, true});
        // pivot_tandem.SetupAllConversions(1.0_deg); //TODO add conversion
        // pivot_tandem.ZeroEncoders();

        // pivot_tandem.Setup(&pivot_esc_gains_);
        // pivot_tandem.SetInverted({false, false, true, true});
        // pivot_tandem.SetupConverter(1.0_deg);
        // pivot_tandem.ZeroEncoder();

        pivot_one_.Setup(&pivot_esc_gains_, true);
        pivot_two_.Setup(&pivot_esc_gains_, true);
        pivot_three_.Setup(&pivot_esc_gains_, false);
        pivot_four_.Setup(&pivot_esc_gains_, false);
        pivot_one_.SetupConverter(10_deg);
        pivot_two_.SetupConverter(10_deg);
        pivot_three_.SetupConverter(10_deg);
        pivot_four_.SetupConverter(10_deg);
        pivot_one_.ZeroEncoder();
        pivot_two_.ZeroEncoder();
        pivot_three_.ZeroEncoder();
        pivot_four_.ZeroEncoder();

        // pivot_tandem_.Setup(&pivot_esc_gains_);
        // pivot_tandem_.SetInverted({true, true, false, false});
        // pivot_tandem_.SetupConverter(10_deg);
        // pivot_tandem_.ZeroEncoder();

        wrist_esc_.Setup(&wrist_esc_gains_, false);
        wrist_esc_.SetupConverter(1_deg);

        tele_one_.Setup(&tele_esc_gains_, true);
        tele_two_.Setup(&tele_esc_gains_, true);
        tele_one_.SetupConverter(2_in);
        tele_two_.SetupConverter(2_in);
        tele_one_.ZeroEncoder();
        tele_two_.ZeroEncoder();

        // tele_tandem_.Setup(&tele_esc_gains_);
        // tele_tandem_.SetInverted({true, true});
        // tele_tandem_.SetupConverter(2_in);
        // tele_tandem_.ZeroEncoder();

        // tele_tandem.Setup(&tele_esc_gains_);
        // tele_tandem.SetupConverter(1.0_in);
        // tele_tandem.ZeroEncoder();

        // tele_tandem.AddESC(tele_one_esc_.that());
        // tele_tandem.AddESC(tele_two_esc_.that());
        // tele_tandem.SetupAll(&tele_esc_gains_, false);
        // tele_tandem.SetupAllConversions(1_in);
        // tele_tandem.ZeroEncoders();
    }
}

ScoringPositionerTarget ScoringPositionerSubsystem::ZeroTarget() const {
  ScoringPositionerTarget target;
  target.pivot_output = 0.0;
  target.wrist_output = 0.0;
  target.extension = 0.0;
  return target;
}

ScoringPositionerTarget ScoringPositionerSubsystem::MakeTarget(
    std::variant<units::degree_t, double> pivot_out,
    std::variant<units::degree_t, double> wrist_out, 
    std::variant<units::inch_t, double> tele_out) {
      ScoringPositionerTarget target;
      target.pivot_output = pivot_out;
      target.wrist_output = wrist_out;
      target.extension = tele_out;
      return target;
}

bool ScoringPositionerSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    frcLib846_VERIFY(tele_one_.VerifyConnected(), ok, "Telescope not connected");
    frcLib846_VERIFY(tele_two_.VerifyConnected(), ok, "Tele two not connected");

    frcLib846_VERIFY(wrist_esc_.VerifyConnected(), ok, "not connected");

    frcLib846_VERIFY(pivot_one_.VerifyConnected(), ok, "Pivot one not connected");
    frcLib846_VERIFY(pivot_two_.VerifyConnected(), ok, "Pivot two not connected");
    frcLib846_VERIFY(pivot_three_.VerifyConnected(), ok, "Pivot three not connected");
    frcLib846_VERIFY(pivot_four_.VerifyConnected(), ok, "Pivot four not connected");

    return ok;
  }
  return true;
}

ScoringPositionerReadings ScoringPositionerSubsystem::GetNewReadings() {
  ScoringPositionerReadings readings;

  readings.pivot_position = pivot_one_.GetPosition();
  readings.wrist_position = wrist_esc_.GetPosition();
  readings.extension = tele_one_.GetPosition();

  return readings;
}

void ScoringPositionerSubsystem::PositionTelescope(ScoringPositionerTarget target) {
  if (auto pos = std::get_if<units::inch_t>(&target.extension)) {
    tele_one_.Write(frcLib846::control::ControlMode::Position, *pos);
    tele_two_.Write(frcLib846::control::ControlMode::Position, *pos);
  } else if (auto output = std::get_if<double>(&target.extension)) {
    tele_one_.Write(frcLib846::control::ControlMode::Percent, *output);
    tele_two_.Write(frcLib846::control::ControlMode::Percent, *output);
  }
}

void ScoringPositionerSubsystem::PositionPivot(ScoringPositionerTarget target) {
  if (auto pos = std::get_if<units::degree_t>(&target.pivot_output)) {
    pivot_one_.Write(frcLib846::control::ControlMode::Position, *pos);
    pivot_two_.Write(frcLib846::control::ControlMode::Position, *pos);
    pivot_three_.Write(frcLib846::control::ControlMode::Position, *pos);
    pivot_four_.Write(frcLib846::control::ControlMode::Position, *pos);
  } else if (auto output = std::get_if<double>(&target.pivot_output)) {
    pivot_one_.Write(frcLib846::control::ControlMode::Percent, *output);
    pivot_two_.Write(frcLib846::control::ControlMode::Percent, *output);
    pivot_three_.Write(frcLib846::control::ControlMode::Percent, *output);
    pivot_four_.Write(frcLib846::control::ControlMode::Percent, *output);
  }
}

void ScoringPositionerSubsystem::PositionWrist(ScoringPositionerTarget target) {
  if (auto pos = std::get_if<units::degree_t>(&target.wrist_output)) {
    wrist_esc_.Write(frcLib846::control::ControlMode::Position, *pos);
  } else if (auto output = std::get_if<double>(&target.wrist_output)) {
    wrist_esc_.Write(frcLib846::control::ControlMode::Percent, *output);
  }
}

void ScoringPositionerSubsystem::DirectWrite(ScoringPositionerTarget target) {
  PositionTelescope(target);
  PositionPivot(target);
  PositionWrist(target);
}