#include "subsystems/hardware/pivot.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

PivotSubsystem::PivotSubsystem(bool init)
    : frc846::robot::GenericSubsystem<PivotReadings, PivotTarget>{"pivot",
                                                                  init} {
  if (init) {
    pivot_three_.OverrideInvert();
    pivot_four_.OverrideInvert();

    pivot_one_.Init(frc846::control::REVSparkType::kSparkFLEX);
    pivot_two_.Init(frc846::control::REVSparkType::kSparkFLEX);
    pivot_three_.Init(frc846::control::REVSparkType::kSparkFLEX);
    pivot_four_.Init(frc846::control::REVSparkType::kSparkFLEX);

    // if (auto esc = pivot_two_.getESC()) {
    //   if (auto leader_esc = pivot_one_.getESC()) {
    //     esc->Follow(*leader_esc);
    //   }
    // }
    // if (auto esc = pivot_four_.getESC()) {
    //   if (auto leader_esc = pivot_three_.getESC()) {
    //     esc->Follow(*leader_esc);  // Not sure whether to invert or
    //                                // not. TODO: check this.
    //   }
    // }
  }
}

void PivotSubsystem::Setup() {
  pivot_one_.Configure({frc846::control::DataTag::kVelocityData,
                        frc846::control::DataTag::kPositionData});
  pivot_two_.Configure({});
  pivot_three_.Configure({});
  pivot_four_.Configure({});

  pivot_one_.ZeroEncoder(pivot_home_offset_.value());
  pivot_two_.ZeroEncoder(pivot_home_offset_.value());
  pivot_three_.ZeroEncoder(pivot_home_offset_.value());
  pivot_four_.ZeroEncoder(pivot_home_offset_.value());
}

PivotTarget PivotSubsystem::ZeroTarget() const {
  PivotTarget target;
  target.pivot_output = pivot_home_offset_.value();
  return target;
}

bool PivotSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    FRC846_VERIFY(pivot_one_.VerifyConnected(), ok, "Pivot one not connected");
    FRC846_VERIFY(pivot_two_.VerifyConnected(), ok, "Pivot two not connected");
    FRC846_VERIFY(pivot_three_.VerifyConnected(), ok,
                  "Pivot three not connected");
    FRC846_VERIFY(pivot_four_.VerifyConnected(), ok,
                  "Pivot four not connected");

    return ok;
  }
  return true;
}

PivotReadings PivotSubsystem::ReadFromHardware() {
  PivotReadings readings;

  readings.pivot_position = pivot_one_.GetPosition();

  readings.both_hooks_engaged = left_switch_.Get() && right_switch_.Get();

  frc846::util::ShareTables::SetDouble("pivot_position",
                                       readings.pivot_position.to<double>());

  pivot_pos_graph.Graph(readings.pivot_position);

  auto target_pivot_output = GetTarget().pivot_output;
  if (auto target_angle = std::get_if<units::degree_t>(&target_pivot_output)) {
    pivot_error_graph.Graph(*target_angle - readings.pivot_position);
  }

  return readings;
}

void PivotSubsystem::WriteToHardware(PivotTarget target) {
  if (target.climb_mode) {
    hard_limits_.OverrideLimits(true);

    pivot_one_.WriteDC(climb_duty_cycle_.value());
    pivot_two_.WriteDC(climb_duty_cycle_.value());
    pivot_three_.WriteDC(climb_duty_cycle_.value());
    pivot_four_.WriteDC(climb_duty_cycle_.value());

  } else if (auto pos = std::get_if<units::degree_t>(&target.pivot_output)) {
    hard_limits_.OverrideLimits(false);

    double output = dyFPID.calculate(*pos, GetReadings().pivot_position,
                                     pivot_one_.GetVelocityPercentage(),
                                     config_helper_.updateAndGetGains());

    pivot_one_.WriteDC(output);
    pivot_two_.WriteDC(output);
    pivot_three_.WriteDC(output);
    pivot_four_.WriteDC(output);

    target_pivot_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.pivot_output)) {
    hard_limits_.OverrideLimits(false);

    pivot_one_.WriteDC(*output);
    pivot_two_.WriteDC(*output);
    pivot_three_.WriteDC(*output);
    pivot_four_.WriteDC(*output);

    target_pivot_duty_cycle_graph.Graph(*output);
  }
}