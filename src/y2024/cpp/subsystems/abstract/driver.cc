#include "subsystems/abstract/driver.h"

#include "frc846/util/share_tables.h"

DriverSubsystem::DriverSubsystem()
    : frc846::Subsystem<DriverReadings, DriverTarget>{"driver", true} {}

DriverTarget DriverSubsystem::ZeroTarget() const {
  DriverTarget target;
  target.rumble = false;
  return target;
}

bool DriverSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(xbox_.IsConnected(), ok, "not connected");
  return ok;
}

DriverReadings DriverSubsystem::GetNewReadings() {
  DriverReadings readings{xbox_, trigger_threshold_.value()};

  previous_readings_ = readings;

  return readings;
}

void DriverSubsystem::DirectWrite(DriverTarget target) {
  target_rumble_graph_.Graph(target.rumble);

  auto rumble =
      (target.rumble || frc846::util::ShareTables::GetBoolean("ready_to_shoot"))
          ? rumble_strength_.value()
          : 0;

  xbox_.SetRumble(frc::XboxController::RumbleType::kLeftRumble, rumble);
  xbox_.SetRumble(frc::XboxController::RumbleType::kRightRumble, rumble);
}