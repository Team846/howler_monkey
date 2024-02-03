#include "subsystems/operator.h"

OperatorSubsystem::OperatorSubsystem()
    : frcLib846::Subsystem<OperatorReadings, OperatorTarget>{"operator", true} {}

OperatorTarget OperatorSubsystem::ZeroTarget() const {
  OperatorTarget target;
  target.rumble = false;
  return target;
}

bool OperatorSubsystem::VerifyHardware() {
  bool ok = true;
  // frcLib846_VERIFY(xbox_.IsConnected(), ok, "not connected");
  return ok;
}

OperatorReadings OperatorSubsystem::GetNewReadings() {
  OperatorReadings readings{xbox_, trigger_threshold_.value()};

  return readings;
}

void OperatorSubsystem::DirectWrite(OperatorTarget target) {
  target_rumble_graph_.Graph(target.rumble);

  auto rumble = target.rumble ? rumble_strength_.value() : 0;

  xbox_.SetRumble(frc::XboxController::RumbleType::kLeftRumble, rumble);
  xbox_.SetRumble(frc::XboxController::RumbleType::kRightRumble, rumble);
}