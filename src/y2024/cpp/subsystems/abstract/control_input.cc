#include "subsystems/abstract/control_input.h"

ControlInputSubsystem::ControlInputSubsystem()
    : frc846::Subsystem<ControlInputReadings, ControlInputTarget>{
          "control_input", true} {}

ControlInputTarget ControlInputSubsystem::ZeroTarget() const {
  ControlInputTarget target;
  return target;
}

bool ControlInputSubsystem::VerifyHardware() { return true; }

ControlInputReadings ControlInputSubsystem::GetNewReadings() {
  ControlInputReadings readings{this_readings_};

  if (readings.stageOfTrap != previous_readings_.stageOfTrap) {
    Log("ControlInput [Trap Stage] state changed to {}", readings.stageOfTrap);
  }
  if (readings.running_intake != previous_readings_.running_intake) {
    Log("ControlInput [Running Intake] state changed to {}",
        readings.running_intake ? 1 : 0);
  }
  if (readings.running_source != previous_readings_.running_source) {
    Log("ControlInput [Running Source] state changed to {}",
        readings.running_source ? 1 : 0);
  }
  if (readings.running_pass != previous_readings_.running_pass) {
    Log("ControlInput [Running Pass] state changed to {}",
        readings.running_pass ? 1 : 0);
  }
  if (readings.running_prep_shoot != previous_readings_.running_prep_shoot) {
    Log("ControlInput [Running Prep Shoot] state changed to {}",
        readings.running_prep_shoot ? 1 : 0);
  }
  if (readings.running_super_shoot != previous_readings_.running_super_shoot) {
    Log("ControlInput [Running Super Shoot] state changed to {}",
        readings.running_super_shoot ? 1 : 0);
  }
  if (readings.running_amp != previous_readings_.running_amp) {
    Log("ControlInput [Running Amp] state changed to {}",
        readings.running_amp ? 1 : 0);
  }

  if (readings.shooting != previous_readings_.shooting) {
    Log("ControlInput [Shooting] state changed to {}",
        readings.shooting ? 1 : 0);
  }
  if (readings.manual_spin_up != previous_readings_.manual_spin_up) {
    Log("ControlInput [Manual Spin Up] state changed to {}",
        readings.manual_spin_up ? 1 : 0);
  }
  if (readings.manual_feed != previous_readings_.manual_feed) {
    Log("ControlInput [Manual Feed] state changed to {}",
        readings.manual_feed ? 1 : 0);
  }
  if (readings.eject != previous_readings_.eject) {
    Log("ControlInput [Eject] state changed to {}", readings.eject ? 1 : 0);
  }

  if (readings.amping_leds != previous_readings_.amping_leds) {
    Log("ControlInput [Amping Leds] state changed to {}",
        readings.amping_leds ? 1 : 0);
  }
  if (readings.coopertition_leds != previous_readings_.coopertition_leds) {
    Log("ControlInput [Co-op Leds] state changed to {}",
        readings.coopertition_leds ? 1 : 0);
  }

  if (readings.amping_leds != previous_readings_.amping_leds) {
    Log("ControlInput [Amping Leds] state changed to {}",
        readings.amping_leds ? 1 : 0);
  }
  if (readings.coopertition_leds != previous_readings_.coopertition_leds) {
    Log("ControlInput [Co-op Leds] state changed to {}",
        readings.coopertition_leds ? 1 : 0);
  }

  if (std::abs(readings.pivot_manual_adjust) > 0.01 &&
      !(std::abs(previous_readings_.pivot_manual_adjust) > 0.01)) {
    Log("ControlInput [Pivot Manual Adjustment]");
  }
  if (std::abs(readings.wrist_manual_adjust) > 0.01 &&
      !(std::abs(previous_readings_.wrist_manual_adjust) > 0.01)) {
    Log("ControlInput [Wrist Manual Adjustment]");
  }
  if (std::abs(readings.telescope_manual_adjust) > 0.01 &&
      !(std::abs(previous_readings_.telescope_manual_adjust) > 0.01)) {
    Log("ControlInput [Telescope Manual Adjustment]");
  }

  previous_readings_ = readings;

  return readings;
}

void ControlInputSubsystem::DirectWrite(ControlInputTarget target) {}

void ControlInputSubsystem::UpdateWithInput(ControlInputReadings newReadings) {
  this_readings_ = newReadings;
}