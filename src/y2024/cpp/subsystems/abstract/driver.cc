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

  if (readings.a_button != previous_readings_.a_button) {
    Log("Driver [A button] state changed to {}", readings.a_button ? 1 : 0);
  }
  if (readings.b_button != previous_readings_.b_button) {
    Log("Driver [B button] state changed to {}", readings.b_button ? 1 : 0);
  }
  if (readings.x_button != previous_readings_.x_button) {
    Log("Driver [X button] state changed to {}", readings.x_button ? 1 : 0);
  }
  if (readings.y_button != previous_readings_.y_button) {
    Log("Driver [Y button] state changed to {}", readings.y_button ? 1 : 0);
  }

  if (readings.pov != previous_readings_.pov) {
    std::string k = "none";
    switch (readings.pov) {
      case (frc846::XboxPOV::kUp):
        k = "up";
        break;

      case (frc846::XboxPOV::kLeft):
        k = "left";
        break;

      case (frc846::XboxPOV::kRight):
        k = "right";
        break;

      case (frc846::XboxPOV::kDown):
        k = "down";
        break;

      default:
        break;
    }
    Log("Driver [POV] state changed to {}", k);
  }

  if (readings.right_trigger != previous_readings_.right_trigger) {
    Log("Driver [Right Trigger] state changed to {}",
        readings.right_trigger ? 1 : 0);
  }
  if (readings.right_bumper != previous_readings_.right_bumper) {
    Log("Driver [Right Bumper] state changed to {}",
        readings.right_bumper ? 1 : 0);
  }
  if (readings.left_trigger != previous_readings_.left_trigger) {
    Log("Driver [Left Trigger] state changed to {}",
        readings.left_trigger ? 1 : 0);
  }
  if (readings.left_bumper != previous_readings_.left_bumper) {
    Log("Driver [Left Bumper] state changed to {}",
        readings.left_bumper ? 1 : 0);
  }

  if (readings.start_button != previous_readings_.start_button) {
    Log("Driver [Start Button] state changed to {}",
        readings.start_button ? 1 : 0);
  }
  if (readings.back_button != previous_readings_.back_button) {
    Log("Driver [Back Button] state changed to {}",
        readings.back_button ? 1 : 0);
  }

  // if ((std::abs(readings.left_stick_x) > 0.5) !=
  // (std::abs(previous_readings_.left_stick_x) > 0.5)) {
  //   Log("Driver [Left Stick X] value changed to {}", readings.left_stick_x);
  // }
  // if ((std::abs(readings.left_stick_y) > 0.5) !=
  // (std::abs(previous_readings_.left_stick_y) > 0.5)) {
  //   Log("Driver [Left Stick Y] value changed to {}", readings.left_stick_y);
  // }
  // if ((std::abs(readings.right_stick_x) > 0.5) !=
  // (std::abs(previous_readings_.right_stick_x) > 0.5)) {
  //   Log("Driver [Right Stick X] value changed to {}",
  //   readings.right_stick_x);
  // }
  // if ((std::abs(readings.right_stick_y) > 0.5) !=
  // (std::abs(previous_readings_.right_stick_y) > 0.5)) {
  //   Log("Driver [Right Stick Y] value changed to {}",
  //   readings.right_stick_y);
  // }

  previous_readings_ = readings;

  return readings;
}

void DriverSubsystem::DirectWrite(DriverTarget target) {
  target_rumble_graph_.Graph(target.rumble);

  auto rumble =
      (target.rumble || frc846::util::ShareTables::GetString("shooting_state")
                                .compare("kReady") == 0)
          ? rumble_strength_.value()
          : 0;

  xbox_.SetRumble(frc::XboxController::RumbleType::kLeftRumble, rumble);
  xbox_.SetRumble(frc::XboxController::RumbleType::kRightRumble, rumble);
}