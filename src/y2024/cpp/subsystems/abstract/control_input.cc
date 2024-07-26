#include "subsystems/abstract/control_input.h"

#include "frc846/util/share_tables.h"

ControlInputSubsystem::ControlInputSubsystem()
    : frc846::Subsystem<ControlInputReadings, ControlInputTarget>{
          "control_input", true} {}

ControlInputTarget ControlInputSubsystem::ZeroTarget() const {
  ControlInputTarget target;
  target.driver_rumble = false;
  target.operator_rumble = false;
  return target;
}

bool ControlInputSubsystem::VerifyHardware() { return true; }

ControlInputReadings ControlInputSubsystem::GetNewReadings() {
  driver_.UpdateReadings();
  operator_.UpdateReadings();

  ControlInputReadings readings = UpdateWithInput();

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
  if (readings.running_super_amp != previous_readings_.running_super_amp) {
    Log("ControlInput [Running Super Amp] state changed to {}",
        readings.running_super_amp ? 1 : 0);
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

  if (readings.zero_bearing != previous_readings_.zero_bearing) {
    Log("ControlInput [Drivetrain Zeroing] state changed to {}",
        readings.zero_bearing ? 1 : 0);
  }
  if (readings.home_wrist != previous_readings_.home_wrist) {
    Log("ControlInput [Wrist Homing] state changed to {}",
        readings.home_wrist ? 1 : 0);
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

void ControlInputSubsystem::DirectWrite(ControlInputTarget target) {
  driver_.SetTarget({target.driver_rumble});
  operator_.SetTarget({target.operator_rumble});

  driver_.UpdateHardware();
  operator_.UpdateHardware();
}

ControlInputReadings ControlInputSubsystem::UpdateWithInput() {
  ControlInputReadings ci_readings_{};
  DriverReadings dr_readings{driver_.readings()};
  OperatorReadings op_readings{operator_.readings()};

  // MOTION
  ci_readings_.translate_x = dr_readings.left_stick_x;
  ci_readings_.translate_y = dr_readings.left_stick_y;
  ci_readings_.rotation = dr_readings.right_stick_x;

  // TRAP
  if (op_readings.left_trigger && !previous_operator_.left_trigger) {
    ci_readings_.stageOfTrap = std::min(ci_readings_.stageOfTrap + 1, 6);
  } else if (op_readings.left_bumper && !previous_operator_.left_bumper) {
    ci_readings_.stageOfTrap = std::max(ci_readings_.stageOfTrap - 1, 0);
  }

  // PREP SHOOT
  ci_readings_.running_prep_shoot = dr_readings.right_trigger;
  // SUPER SHOOT
  ci_readings_.running_super_shoot = dr_readings.y_button;

  // AMP
  ci_readings_.running_amp = dr_readings.left_bumper;
  // SUPER AMP
  ci_readings_.running_super_amp = dr_readings.a_button;

  // INTAKE
  ci_readings_.running_intake = dr_readings.left_trigger;
  // SOURCE
  ci_readings_.running_source = dr_readings.x_button;
  // PASS
  ci_readings_.running_pass = op_readings.x_button;

  // SHOOT
  ci_readings_.shooting =
      ((ci_readings_.running_prep_shoot || ci_readings_.running_super_shoot ||
        ci_readings_.running_pass) &&
       dr_readings.right_bumper) ||
      op_readings.pov == frc846::XboxPOV::kRight;
  // EJECT
  ci_readings_.eject =
      ((ci_readings_.running_amp) && dr_readings.right_bumper) ||
      op_readings.pov == frc846::XboxPOV::kUp;
  // MANUAL FEED
  ci_readings_.manual_feed = op_readings.pov == frc846::XboxPOV::kLeft;
  // MANUAL SPIN UP
  ci_readings_.manual_spin_up = op_readings.pov == frc846::XboxPOV::kDown;

  // MANUAL ADJUSTMENTS
  ci_readings_.pivot_manual_adjust = op_readings.left_stick_y;
  ci_readings_.telescope_manual_adjust = op_readings.left_stick_x;
  ci_readings_.wrist_manual_adjust = op_readings.right_stick_y;

  // LEDS
  ci_readings_.amping_leds = op_readings.right_trigger;
  frc846::util::ShareTables::SetBoolean("amp", ci_readings_.amping_leds);
  ci_readings_.coopertition_leds = op_readings.right_bumper;
  frc846::util::ShareTables::SetBoolean("coopertition",
                                        ci_readings_.coopertition_leds);

  // RESETS
  ci_readings_.home_wrist = dr_readings.b_button;
  ci_readings_.zero_bearing = dr_readings.back_button;

  previous_operator_ = op_readings;
  previous_driver_ = dr_readings;

  return ci_readings_;
}