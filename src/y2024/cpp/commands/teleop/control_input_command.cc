#include "commands/teleop/control_input_command.h"

#include <utility>

#include "constants.h"
#include "field.h"
#include "frc846/loggable.h"
#include "frc846/util/math.h"
#include "frc846/util/share_tables.h"
#include "subsystems/hardware/drivetrain.h"
#include "subsystems/hardware/swerve_module.h"

ControlInputCommand::ControlInputCommand(RobotContainer& container)
    : driver_(container.driver_),
      operator_(container.operator_),
      control_input_(container.control_input_),
      super_(container.super_structure_) {
  AddRequirements({&control_input_});
  SetName("control_input_command");
}

void ControlInputCommand::Execute() {
  DriverReadings dr_readings{driver_.readings()};
  OperatorReadings op_readings{operator_.readings()};

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

  // INTAKE
  ci_readings_.running_intake = dr_readings.left_trigger;
  // SOURCE
  ci_readings_.running_source = dr_readings.x_button;
  // PASS
  ci_readings_.running_pass = dr_readings.a_button;

  // SHOOT
  ci_readings_.shooting =
      ((ci_readings_.running_prep_shoot || ci_readings_.running_super_shoot) &&
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

  previous_operator_ = op_readings;
  previous_driver_ = dr_readings;
  control_input_.UpdateWithInput(ci_readings_);
}

bool ControlInputCommand::IsFinished() { return false; }