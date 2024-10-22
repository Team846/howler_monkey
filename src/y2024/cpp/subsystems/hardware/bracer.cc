#include "subsystems/hardware/bracer.h"

#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

BracerSubsystem::BracerSubsystem(bool init)
    : frc846::robot::GenericSubsystem<BracerReadings, BracerTarget>{"bracer",
                                                                    init} {
  if (init) {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", false);
  }
}

BracerTarget BracerSubsystem::ZeroTarget() const {
  BracerTarget target;
  target.state = BracerState::kStow;
  return target;
}

bool BracerSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    return ok;
  }
  return true;
}

BracerReadings BracerSubsystem::ReadFromHardware() {
  BracerReadings readings;

  return readings;
}

void BracerSubsystem::WriteToHardware(BracerTarget target) {
  if (target.state == BracerState::kExtend) {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", true);
    bracer_.Set(1.0);
  } else if (target.state == BracerState::kRetract) {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", false);
    bracer_.Set(-0.7);
  } else {
    bracer_.Set(0.0);
  }
}