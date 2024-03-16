#include "subsystems/pivot.h"
#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

PivotSubsystem::PivotSubsystem(bool init)
    : frc846::Subsystem<PivotReadings, PivotTarget>{"pivot", init} {
    if (init) {
        pivot_one_.Setup(&pivot_esc_gains_, true, frc846::control::kBrake);
        pivot_two_.Setup(&pivot_esc_gains_, true, frc846::control::kBrake);
        pivot_three_.Setup(&pivot_esc_gains_, false, frc846::control::kBrake);
        pivot_four_.Setup(&pivot_esc_gains_, false, frc846::control::kBrake);
        pivot_one_.SetupConverter(1.0_tr / (68.0/7.0 * 50.0/18.0 * 64.0/10.0));
        pivot_two_.SetupConverter(1.0_tr / (68.0/7.0 * 50.0/18.0 * 64.0/10.0));
        pivot_three_.SetupConverter(1.0_tr / (68.0/7.0 * 50.0/18.0 * 64.0/10.0));
        pivot_four_.SetupConverter(1.0_tr / (68.0/7.0 * 50.0/18.0 * 64.0/10.0));
        pivot_one_.ZeroEncoder();
        pivot_two_.ZeroEncoder();
        pivot_three_.ZeroEncoder();
        pivot_four_.ZeroEncoder();

        pivot_one_.ConfigurePositionLimits(120_deg, -8_deg);
        pivot_two_.ConfigurePositionLimits(120_deg, -8_deg);
        pivot_three_.ConfigurePositionLimits(120_deg, -8_deg);
        pivot_four_.ConfigurePositionLimits(120_deg, -8_deg);

        pivot_one_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus0, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus3});
        pivot_two_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus0, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus3});
        pivot_three_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus0, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus3});
        pivot_four_.DisableStatusFrames({rev::CANSparkBase::PeriodicFrame::kStatus0, 
          rev::CANSparkBase::PeriodicFrame::kStatus4, 
          rev::CANSparkBase::PeriodicFrame::kStatus3});
    }
}

PivotTarget PivotSubsystem::ZeroTarget() const {
  PivotTarget target;
  target.pivot_output = 0.0;
  return target;
}

PivotTarget PivotSubsystem::MakeTarget(
    std::variant<units::degree_t, double> pivot_out) {
      PivotTarget target;
      target.pivot_output = pivot_out;
      return target;
}

bool PivotSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    FRC846_VERIFY(pivot_one_.VerifyConnected(), ok, "Pivot one not connected");
    FRC846_VERIFY(pivot_two_.VerifyConnected(), ok, "Pivot two not connected");
    FRC846_VERIFY(pivot_three_.VerifyConnected(), ok, "Pivot three not connected");
    FRC846_VERIFY(pivot_four_.VerifyConnected(), ok, "Pivot four not connected");

    return ok;
  }
  return true;
}

PivotReadings PivotSubsystem::GetNewReadings() {
  PivotReadings readings;

  readings.pivot_position = pivot_one_.GetPosition();

  frc846::util::ShareTables::SetDouble("pivot_position", readings.pivot_position.to<double>());

  pivot_pos_graph.Graph(readings.pivot_position);

  return readings;
}

void PivotSubsystem::PositionPivot(PivotTarget target) {
  if (auto pos = std::get_if<units::degree_t>(&target.pivot_output)) {
    // if (*pos < 1.0_deg && readings().pivot_position < -0.5_deg) {
    //   pivot_one_.Write(frc846::control::ControlMode::Percent, 0.0);
    //   pivot_two_.Write(frc846::control::ControlMode::Percent, 0.0);
    //   pivot_three_.Write(frc846::control::ControlMode::Percent, 0.0);
    //   pivot_four_.Write(frc846::control::ControlMode::Percent, 0.0);
    // } else {
      pivot_one_.Write(frc846::control::ControlMode::Position, *pos);
      pivot_two_.Write(frc846::control::ControlMode::Position, *pos);
      pivot_three_.Write(frc846::control::ControlMode::Position, *pos);
      pivot_four_.Write(frc846::control::ControlMode::Position, *pos);

    // }

    target_pivot_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.pivot_output)) {
    pivot_one_.Write(frc846::control::ControlMode::Percent, *output);
    pivot_two_.Write(frc846::control::ControlMode::Percent, *output);
    pivot_three_.Write(frc846::control::ControlMode::Percent, *output);
    pivot_four_.Write(frc846::control::ControlMode::Percent, *output);

    target_pivot_duty_cycle_graph.Graph(*output);
  }
}

void PivotSubsystem::DirectWrite(PivotTarget target) {
  PositionPivot(target);
}