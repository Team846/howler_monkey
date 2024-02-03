#include "subsystems/driver.h"

DriverSubsystem::DriverSubsystem()
    : frcLib846::Subsystem<DriverReadings, DriverTarget>{"driver", true} {}

DriverTarget DriverSubsystem::ZeroTarget() const {
  DriverTarget target;
  target.rumble = false;
  return target;
}

bool DriverSubsystem::VerifyHardware() {
  bool ok = true;
  // frcLib846_VERIFY(xbox_.IsConnected(), ok, "not connected");
  return ok;
}

DriverReadings DriverSubsystem::GetNewReadings() {
  DriverReadings readings{xbox_, trigger_threshold_.value()};

  return readings;
}

void DriverSubsystem::DirectWrite(DriverTarget target) {
  target_rumble_graph_.Graph(target.rumble);

  auto rumble = target.rumble ? rumble_strength_.value() : 0;

  xbox_.SetRumble(frc::XboxController::RumbleType::kLeftRumble, rumble);
  xbox_.SetRumble(frc::XboxController::RumbleType::kRightRumble, rumble);
}