#include "subsystems/hardware/pivot.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

PivotSubsystem::PivotSubsystem(bool init)
    : frc846::Subsystem<PivotReadings, PivotTarget>{"pivot", init} {
  if (init) {
    pivot_one_.Configure({frc846::control::DataTag::kLeader,
                          frc846::control::DataTag::kPositionData});
    pivot_three_.Configure({frc846::control::DataTag::kLeader,
                            frc846::control::DataTag::kPositionData});
    pivot_two_.Configure({});
    pivot_four_.Configure({});

    pivot_three_.OverrideInvert(!config_helper_.getMotorConfig().invert);

    if (auto esc = pivot_two_.getESC()) {
      if (auto leader_esc = pivot_one_.getESC()) {
        esc->Follow(*pivot_one_.getESC());
      }
    }
    if (auto esc = pivot_four_.getESC()) {
      if (auto leader_esc = pivot_three_.getESC()) {
        esc->Follow(*pivot_one_.getESC());  // Not sure whether to invert or
                                            // not. TODO: check this.
      }
    }
  }
}

PivotTarget PivotSubsystem::ZeroTarget() const {
  PivotTarget target;
  target.pivot_output = 0.0;
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

PivotReadings PivotSubsystem::GetNewReadings() {
  PivotReadings readings;

  readings.pivot_position = pivot_one_.GetPosition();

  frc846::util::ShareTables::SetDouble("pivot_position",
                                       readings.pivot_position.to<double>());

  pivot_pos_graph.Graph(readings.pivot_position);

  if (auto target_angle = std::get_if<units::degree_t>(&target_.pivot_output)) {
    pivot_error_graph.Graph(*target_angle - readings.pivot_position);
  }

  return readings;
}

void PivotSubsystem::DirectWrite(PivotTarget target) {
  if (auto pos = std::get_if<units::degree_t>(&target.pivot_output)) {
    pivot_one_.WritePosition(*pos);
    pivot_three_.WritePosition(*pos);

    target_pivot_pos_graph.Graph(*pos);
  } else if (auto output = std::get_if<double>(&target.pivot_output)) {
    pivot_one_.WriteDC(*output);
    pivot_three_.WriteDC(*output);

    target_pivot_duty_cycle_graph.Graph(*output);
  }
}