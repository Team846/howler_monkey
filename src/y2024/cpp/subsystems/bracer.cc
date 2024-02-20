#include "subsystems/bracer.h"
#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"

BracerSubsystem::BracerSubsystem(bool init)
    : frc846::Subsystem<BracerReadings, BracerTarget>{"bracer", init} {
    if (init) {}
}

BracerTarget BracerSubsystem::ZeroTarget() const {
  BracerTarget target;
  target.extend = false;
  return target;
}

BracerTarget BracerSubsystem::MakeTarget(bool extend) {
      BracerTarget target;
      target.extend = extend;
      return target;
}

bool BracerSubsystem::VerifyHardware() {
  if (is_initialized()) {
    bool ok = true;

    return ok;
  }
  return true;
}

BracerReadings BracerSubsystem::GetNewReadings() {
  BracerReadings readings;


  return readings;
}


void BracerSubsystem::DirectWrite(BracerTarget target) {
  if (target.extend) {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", true);
    bracer_left_.SetAngle(180);
    bracer_right_.SetAngle(180);
  } else {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", false);
  }
}