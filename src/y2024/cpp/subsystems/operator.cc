#include "subsystems/operator.h"
#include "frc846/util/share_tables.h"

OperatorSubsystem::OperatorSubsystem()
    : frc846::Subsystem<OperatorReadings, OperatorTarget>{"operator", true} {}

OperatorTarget OperatorSubsystem::ZeroTarget() const {
  OperatorTarget target;
  target.rumble = false;
  return target;
}

bool OperatorSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(xbox_.IsConnected(), ok, "not connected");
  return ok;
}

OperatorReadings OperatorSubsystem::GetNewReadings() {
  OperatorReadings readings{xbox_, trigger_threshold_.value()};

  frc846::util::ShareTables::SetBoolean("amp", readings.left_stick_x > 0.5);
  frc846::util::ShareTables::SetBoolean("coopertition", readings.left_stick_x < -0.5);

  return readings;
}

void OperatorSubsystem::DirectWrite(OperatorTarget target) {
  target_rumble_graph_.Graph(target.rumble);

  auto rumble = target.rumble ? rumble_strength_.value() : 0;

  xbox_.SetRumble(frc::XboxController::RumbleType::kLeftRumble, rumble);
  xbox_.SetRumble(frc::XboxController::RumbleType::kRightRumble, rumble);
}