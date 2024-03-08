#include "subsystems/bracer.h"
#include "frc846/control/control.h"
#include "frc846/util/share_tables.h"


BracerSubsystem::BracerSubsystem(bool init)
    : frc846::Subsystem<BracerReadings, BracerTarget>{"bracer", init} {
    if (init) {
    }
}

BracerTarget BracerSubsystem::ZeroTarget() const {
  BracerTarget target;
  target.state = BracerState::kStow;
  return target;
}

BracerTarget BracerSubsystem::MakeTarget(BracerState state) {
      BracerTarget target;
      target.state = state;
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

  left_climb_.Graph(left_switch_.Get());
  right_climb_.Graph(right_switch_.Get());

  frc846::util::ShareTables::SetBoolean("climb_hooks_engaged", left_switch_.Get() && right_switch_.Get());

  return readings;
}


void BracerSubsystem::DirectWrite(BracerTarget target) {
  if (target.state != lastState) {
    counter = 50;
    lastState = target.state;
  }
  if (target.state == BracerState::kExtend) {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", true);
    if (counter > 0) {
      bracer_left_.Set(1.0);
      // bracer_right_.SetVoltage(6_V);
      counter -= 1;
    } else {
      bracer_left_.Set(0.0);

      // bracer_left_.SetAngle(180);
      // bracer_right_.SetAngle(180);
    }
  } else if (target.state == BracerState::kRetract) {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", false);
      bracer_left_.Set(-0.5);;
  } else {
    frc846::util::ShareTables::SetBoolean("is_climb_sequence", false);
    counter = 50;
  }
}